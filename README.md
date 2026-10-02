# Simulasi Quadcopter PID 3D

## Struktur
- `main.py` : program utama simulasi
- `requirements.txt` : library yang dibutuhkan

## Menjalankan di VS Code

1. Pastikan Python sudah terpasang.
2. Buka folder project di VS Code.
3. Buka Terminal.
4. Buat virtual environment:

```bash
python -m venv venv
```

5. Aktifkan virtual environment di Windows:

```bash
venv\Scripts\activate
```

6. Install library:

```bash
python -m pip install --upgrade pip
pip install -r requirements.txt
```

7. Jalankan:

```bash
python main.py
```

Program akan menampilkan:
- grafik respons Roll, Pitch, dan Yaw,
- grafik error PID,
- animasi kinematika 3D quadcopter.

Parameter PID dapat diubah pada bagian `KP`, `KI`, dan `KD`.
