from pymavlink import mavutil
import time
import math

# Hubungkan ke SpeedyBee F405 Wing via port serial (ubah sesuai dengan port COM yang sesuai)
connection = mavutil.mavlink_connection('COM17', baud=115200)

# Sinkronisasi waktu (menunggu untuk memastikan MAVLink terhubung)
connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (connection.target_system, connection.target_component))

# Inisialisasi offset akselerometer
offset_x, offset_y, offset_z = 0.0, 0.0, 0.0

# Ambil beberapa sampel awal untuk menghitung offset ketika sensor diam
num_samples = 20
print("Kalibrasi Sensor")
for i in range(num_samples):
    msg = connection.recv_match(type='RAW_IMU', blocking=True)
    if msg is not None:
        offset_x += msg.xacc / 1000  # m/s²
        offset_y += msg.yacc / 1000  # m/s²
        offset_z += msg.zacc / 1000  # m/s²

# Hitung rata-rata untuk offset
offset_x /= num_samples
offset_y /= num_samples
offset_z /= num_samples

print("Offset akselerometer terdeteksi:")
print("offset_x: {:.2f} m/s², offset_y: {:.2f} m/s², offset_z: {:.2f} m/s²".format(offset_x, offset_y, offset_z))

# Inisialisasi variabel untuk menyimpan data akselerasi sebelumnya
prev_accs = [(0.0, 0.0, 0.0), (0.0, 0.0, 0.0)]

# Loop untuk terus membaca nilai IMU
while True:
    msg = connection.recv_match(type='RAW_IMU', blocking=True)
    if msg is not None:
        # Konversi data akselerometer dari mm/s² ke m/s² dan kurangi offset
        acc_x = (msg.xacc / 1000) - offset_x  # m/s²
        acc_y = (msg.yacc / 1000) - offset_y  # m/s²
        acc_z = (msg.zacc / 1000) - offset_z  # m/s²

        # Hitung rata-rata dari dua data akselerasi sebelumnya
        avg_prev_acc_x = (prev_accs[0][0] + prev_accs[1][0]) / 2
        avg_prev_acc_y = (prev_accs[0][1] + prev_accs[1][1]) / 2
        avg_prev_acc_z = (prev_accs[0][2] + prev_accs[1][2]) / 2

        # Cek apakah rata-rata data sebelumnya sama dengan data sekarang
        if math.isclose(acc_x, avg_prev_acc_x, abs_tol=0.01) and \
           math.isclose(acc_y, avg_prev_acc_y, abs_tol=0.01) and \
           math.isclose(acc_z, avg_prev_acc_z, abs_tol=0.01):
            acc_x, acc_y, acc_z = 0.0, 0.0, 0.0

        # Simpan data akselerasi saat ini ke dalam buffer data sebelumnya
        prev_accs.pop(0)  # Hapus data paling lama
        prev_accs.append((acc_x, acc_y, acc_z))  # Tambahkan data baru

        # Tampilkan nilai akselerometer yang sudah dikurangi offset atau di-reset ke 0
        print("accX: {:.2f} m/s², accY: {:.2f} m/s², accZ: {:.2f} m/s²".format(acc_x, acc_y, acc_z))
        print("------")

    time.sleep(0.01)  # Delay yang kecil untuk mencegah pembacaan terlalu cepat
