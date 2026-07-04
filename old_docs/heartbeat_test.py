from pymavlink import mavutil

PORT = "/dev/ttyACM0"
BAUD = 115200

print("[INFO] Pixhawk bağlantısı deneniyor...")

master = mavutil.mavlink_connection(PORT, baud=BAUD)

print("[INFO] Heartbeat bekleniyor...")
master.wait_heartbeat(timeout=15)

print("[OK] Heartbeat alındı.")
print(f"System ID: {master.target_system}")
print(f"Component ID: {master.target_component}")
