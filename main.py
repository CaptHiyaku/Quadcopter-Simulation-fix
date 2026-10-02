"""
Simulator Terbang Quadcopter 3D  -  main.py

File ini punya dua peran:
  1. Di browser (Pyodide): dimuat oleh script.js. Berisi fisika, motor mixing, dan
     flight controller PID. Fungsi press() dan frame() dipanggil dari JavaScript.
  2. Di komputer (python main.py): menjalankan server lokal dan membuka halaman HTML.
"""
import numpy as np

# ------------------------------------------------------------
G, MASS, ARM = 9.81, 1.2, 0.6
DM = ARM / np.sqrt(2)
TMAX, KM, TAU = 7.0, 0.03, 0.045          # thrust maks/motor (N), koef. torsi yaw, lag motor (s)
INERTIA = np.array([0.05, 0.05, 0.09])    # Ix, Iy, Iz
GZ, DT = 0.08, 0.004                      # tinggi badan di tanah, langkah fisika (250 Hz)
FRAME_DT = 0.02                           # 50 FPS -> 5 langkah fisika per frame
MPOS = np.array([[DM, -DM], [DM, DM], [-DM, DM], [-DM, -DM]])  # M1 depan-kanan, M2 depan-kiri, M3 belakang-kiri, M4 belakang-kanan
MSPIN = np.array([-1, 1, -1, 1])
MAXTILT, YAWMAX = np.radians(32), np.radians(80)
KANG, RMAX = 7.0, 4.0


class PID:
    def __init__(self, kp, ki, kd, lim, ilim):
        self.kp, self.ki, self.kd, self.lim, self.ilim = kp, ki, kd, lim, ilim
        self.reset()

    def reset(self):
        self.i, self.pm, self.df = 0.0, None, 0.0

    def update(self, err, meas, dt):
        self.i = np.clip(self.i + err * dt, -self.ilim, self.ilim)   # anti wind-up
        if self.pm is None:
            self.pm = meas
        self.df += ((-(meas - self.pm) / dt) - self.df) * 0.2        # turunan pada pengukuran, difilter
        self.pm = meas
        return float(np.clip(self.kp * err + self.ki * self.i + self.kd * self.df, -self.lim, self.lim))


class Quadcopter:
    def __init__(self):
        self.mode, self.wind_on = "alt", False
        self.pid_r = PID(0.8, 0.5, 0.008, 3, 1.5)
        self.pid_p = PID(0.8, 0.5, 0.008, 3, 1.5)
        self.pid_y = PID(0.18, 0.12, 0, 0.3, 1)
        self.reset()

    def reset(self):
        self.pos = np.array([0.0, 0.0, GZ])
        self.vel = np.zeros(3)
        self.R = np.eye(3)                 # kolom = sumbu body (x depan, y kiri, z atas) dalam koordinat dunia
        self.w = np.zeros(3)               # laju sudut body (p, q, r)
        self.mot = np.zeros(4)             # kecepatan motor ternormalisasi 0..1
        self.ang = np.zeros(4)
        self.thr, self.zsp, self.iz, self.t = 0.0, GZ, 0.0, 0.0
        self.armed = False
        self.st = dict(roll=0.0, pitch=0.0, yaw=0.0, thr=0.0)
        for p in (self.pid_r, self.pid_p, self.pid_y):
            p.reset()

    def euler(self):
        R = self.R
        return (np.arctan2(R[2, 1], R[2, 2]),                  # roll
                np.arcsin(np.clip(-R[2, 0], -1, 1)),           # theta (positif = hidung turun)
                np.arctan2(R[1, 0], R[0, 0]))                  # yaw

    def wind(self):
        if not self.wind_on:
            return np.zeros(3)
        t = self.t
        return np.array([2.2 * (np.sin(t * .31) + .5 * np.sin(t * 1.1 + 1)),
                         2.2 * (np.cos(t * .23) + .5 * np.sin(t * .9)),
                         .4 * np.sin(t * .7)])

    def level(self):
        yaw = np.arctan2(self.R[1, 0], self.R[0, 0])
        c, s = np.cos(yaw), np.sin(yaw)
        self.R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])
        self.w = np.zeros(3)

    def step(self, dt, keys):
        self.t += dt
        k = lambda n: 1.0 if n in keys else 0.0
        tg = dict(roll=k("right") - k("left"), pitch=k("up") - k("down"),
                  yaw=k("a") - k("d"), thr=k("w") - k("s"))
        a = 1 - np.exp(-dt * 8)                                   # stick dihaluskan
        for n in tg:
            self.st[n] += (tg[n] - self.st[n]) * a

        roll, th, _ = self.euler()
        p, q, r = self.w
        Tc = tx = ty = tz = 0.0
        if self.armed:
            # loop luar: sudut -> laju sudut; loop dalam: PID laju sudut -> torsi
            p_sp = np.clip(KANG * (self.st["roll"] * MAXTILT - roll), -RMAX, RMAX)
            q_sp = np.clip(KANG * (self.st["pitch"] * MAXTILT - th), -RMAX, RMAX)
            tx = self.pid_r.update(p_sp - p, p, dt)
            ty = self.pid_p.update(q_sp - q, q, dt)
            tz = self.pid_y.update(self.st["yaw"] * YAWMAX - r, r, dt)
            if self.mode == "stab":
                self.thr = float(np.clip(self.thr + self.st["thr"] * 0.35 * dt, 0, 1))
                Tc = self.thr * 4 * TMAX
            else:
                if self.pos[2] <= GZ + 0.01 and self.st["thr"] <= 0.05:
                    Tc, self.zsp, self.iz = 0.0, self.pos[2], 0.0
                else:
                    if abs(self.st["thr"]) > 0.05:
                        vsp, self.zsp = self.st["thr"] * 3, self.pos[2]
                    else:
                        vsp = np.clip(1.5 * (self.zsp - self.pos[2]), -1.5, 1.5)
                    ev = vsp - self.vel[2]
                    self.iz = float(np.clip(self.iz + ev * dt * 2, -4, 4))
                    Tc = float(np.clip(MASS * (G + 3.5 * ev + self.iz) / max(0.6, self.R[2, 2]), 0, 4 * TMAX))

        # motor mixing (konfigurasi X) -> perintah motor -> lag motor
        tgt = np.zeros(4)
        if self.armed:
            d2 = 4 * DM * DM
            F = Tc / 4 + MPOS[:, 1] * tx / d2 - MPOS[:, 0] * ty / d2 + MSPIN * tz / (4 * KM)
            tgt = np.sqrt(np.clip(F, TMAX * 0.01, TMAX) / TMAX)
        self.mot += (tgt - self.mot) * (1 - np.exp(-dt / TAU))
        F = TMAX * self.mot ** 2
        Tt = F.sum()
        torque = np.array([(MPOS[:, 1] * F).sum(), -(MPOS[:, 0] * F).sum(), (MSPIN * KM * F).sum()])

        # di tanah: landing gear menahan badan
        if self.pos[2] <= GZ + 0.003 and self.vel[2] <= 0.05 and Tt < MASS * G:
            self.pos[2], self.vel[2] = GZ, 0.0
            self.vel[:2] *= np.exp(-6 * dt)
            self.level()
            for pid in (self.pid_r, self.pid_p, self.pid_y):
                pid.reset()
            return

        # translasi: thrust + gravitasi + drag (linear + kuadratik) terhadap udara
        vr = self.vel - self.wind()
        cd = 0.25 + 0.04 * np.linalg.norm(vr)
        acc = (Tt * self.R[:, 2] - cd * vr) / MASS - np.array([0, 0, G])
        self.vel += acc * dt
        self.pos += self.vel * dt

        # rotasi: persamaan Euler benda tegar + redaman
        I = INERTIA
        w = self.w
        self.w = w + np.array([
            (torque[0] - 0.01 * w[0] - (I[2] - I[1]) * w[1] * w[2]) / I[0],
            (torque[1] - 0.01 * w[1] - (I[0] - I[2]) * w[2] * w[0]) / I[1],
            (torque[2] - 0.01 * w[2] - (I[1] - I[0]) * w[0] * w[1]) / I[2]]) * dt
        ww = self.R @ w
        self.R = self.R + np.cross(ww, self.R.T).T * dt
        u, _, vt = np.linalg.svd(self.R)                          # jaga R tetap ortonormal
        self.R = u @ vt

        # tabrakan tanah
        if self.pos[2] < GZ:
            if self.vel[2] < -3.5 and self.armed:
                self.armed = False
                return "Mendarat terlalu keras. Motor dimatikan."
            self.pos[2] = GZ
            self.vel[2] = -self.vel[2] * 0.15 if self.vel[2] < -0.5 else 0.0


# ------------------------------------------------------------
# API yang dipanggil dari JavaScript
# ------------------------------------------------------------
quad = Quadcopter()
_acc = 0.0


def press(k):
    """Tombol sekali-tekan. Mengembalikan pesan untuk HUD."""
    q = quad
    if k == "space":
        if q.armed:
            q.armed = False
            return "DISARM: motor mati" + (" (drone jatuh bebas!)" if q.pos[2] > GZ + 0.1 else "")
        q.armed, q.thr, q.zsp, q.iz = True, 0.0, q.pos[2], 0.0
        return "ARMED: motor menyala idle"
    if k == "r":
        q.reset()
        return "Simulasi di-reset"
    if k == "m":
        q.mode = "stab" if q.mode == "alt" else "alt"
        q.thr = 0.0
        return "Mode ALT HOLD: W/S mengatur laju naik-turun" if q.mode == "alt" else "Mode STABILIZE: W/S mengatur gas langsung"
    if k == "g":
        q.wind_on = not q.wind_on
        return "Angin aktif" if q.wind_on else "Angin mati"
    return ""


def frame(keys, dt):
    """Majukan simulasi sebesar dt detik, kembalikan state sebagai list datar."""
    global _acc
    keys, msg, q = set(keys), "", quad
    _acc += min(dt, 0.05)
    n = 0
    while _acc >= DT and n < 20:
        m = q.step(DT, keys)
        if m:
            msg = m
        _acc -= DT
        n += 1
    if n >= 20:
        _acc = 0.0
    return (q.pos.tolist() + q.vel.tolist() + q.R.flatten().tolist() + q.w.tolist() + q.mot.tolist()
            + [1.0 if q.armed else 0.0, float(q.thr), 0.0 if q.mode == "alt" else 1.0,
               1.0 if q.wind_on else 0.0, float(q.t), msg])


# ------------------------------------------------------------
# Server lokal (hanya dijalankan lewat `python main.py`, bukan di browser)
# ------------------------------------------------------------
def serve():
    import http.server
    import os
    import socketserver
    import threading
    import webbrowser
    from urllib.parse import quote

    root = os.path.dirname(os.path.abspath(__file__))
    page = "Simulator Terbang Quadcopter 3D.html"

    class Handler(http.server.SimpleHTTPRequestHandler):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, directory=root, **kwargs)

        def do_GET(self):
            if self.path in ("", "/"):
                self.send_response(302)
                self.send_header("Location", "/" + quote(page))
                self.end_headers()
                return
            super().do_GET()

        def end_headers(self):
            self.send_header("Cache-Control", "no-cache")
            super().end_headers()

        def log_message(self, *args):
            pass

    socketserver.TCPServer.allow_reuse_address = True
    for port in range(8000, 8011):
        try:
            httpd = socketserver.TCPServer(("127.0.0.1", port), Handler)
            break
        except OSError:
            continue
    else:
        raise SystemExit("Port 8000-8010 sedang dipakai. Tutup program lain lalu coba lagi.")

    url = f"http://127.0.0.1:{port}/"
    print(f"Simulator berjalan di {url}")
    print("Tekan Ctrl+C untuk berhenti.")
    threading.Timer(0.5, lambda: webbrowser.open(url)).start()
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nServer dihentikan.")
    finally:
        httpd.server_close()


import sys

if sys.platform != "emscripten" and __name__ == "__main__":
    serve()