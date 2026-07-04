from pymavlink import mavutil
import time

PORT = "/dev/ttyACM0"
BAUD = 115200

master = mavutil.mavlink_connection(PORT, baud=BAUD)

print("[INFO] Heartbeat bekleniyor...")
master.wait_heartbeat()
print("[OK] Pixhawk bağlı.")

# Component ID 0 görünüyorsa komutlarda 1 kullanmak daha güvenli
target_system = master.target_system
target_component = 1

print(f"[INFO] Target system: {target_system}, component: {target_component}")

# Pixhawk'tan veri akışı iste
master.mav.request_data_stream_send(
    target_system,
    target_component,
    mavutil.mavlink.MAV_DATA_STREAM_POSITION,
    2,   # 2 Hz
    1    # start
)

master.mav.request_data_stream_send(
    target_system,
    target_component,
    mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
    2,
    1
)

print("[INFO] GPS / position mesajları bekleniyor...")

while True:
    msg = master.recv_match(
        type=["GPS_RAW_INT", "GLOBAL_POSITION_INT", "VFR_HUD", "ATTITUDE"],
        blocking=True,
        timeout=5
    )

    if msg is None:
        print("[WARN] Mesaj gelmedi. Pixhawk GPS veya position verisi göndermiyor olabilir.")
        continue

    msg_type = msg.get_type()

    if msg_type == "GPS_RAW_INT":
        print("\n--- GPS_RAW_INT ---")
        print(f"Fix Type       : {msg.fix_type}")
        print(f"Satellites     : {msg.satellites_visible}")
        print(f"Latitude       : {msg.lat / 1e7}")
        print(f"Longitude      : {msg.lon / 1e7}")
        print(f"Altitude MSL   : {msg.alt / 1000.0} m")

    elif msg_type == "GLOBAL_POSITION_INT":
        print("\n--- GLOBAL_POSITION_INT ---")
        print(f"Latitude       : {msg.lat / 1e7}")
        print(f"Longitude      : {msg.lon / 1e7}")
        print(f"Relative Alt   : {msg.relative_alt / 1000.0} m")
        print(f"Heading        : {msg.hdg / 100.0 if msg.hdg != 65535 else None}")

    elif msg_type == "VFR_HUD":
        print("\n--- VFR_HUD ---")
        print(f"Groundspeed    : {msg.groundspeed} m/s")
        print(f"Airspeed       : {msg.airspeed} m/s")
        print(f"Altitude       : {msg.alt} m")
        print(f"Heading        : {msg.heading}")

    elif msg_type == "ATTITUDE":
        print("\n--- ATTITUDE ---")
        print(f"Roll           : {msg.roll}")
        print(f"Pitch          : {msg.pitch}")
        print(f"Yaw            : {msg.yaw}")

    time.sleep(0.2)
