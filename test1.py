import time
import csv
from robomaster import robot

# ---------- โซนสำหรับเก็บข้อมูลเซนเซอร์ ----------
sensor_log = []  # อาเรย์สำหรับเก็บข้อมูลเซนเซอร์ทุกตัว

# ฟังก์ชัน callback สำหรับ attitude (ทิศทาง/มุม)
def sub_info_attitude(sub_info):
    # เก็บข้อมูล attitude ลงใน sensor_log พร้อม timestamp
    sensor_log.append({
        "timestamp": time.time(),
        "sensor": "attitude",
        "data": sub_info
    })

# ฟังก์ชัน callback สำหรับ position (ตำแหน่ง)
def sub_info_position(sub_info):
    sensor_log.append({
        "timestamp": time.time(),
        "sensor": "position",
        "data": sub_info
    })

# ฟังก์ชัน callback สำหรับ IMU (ข้อมูลความเร่ง/หมุน)
def sub_info_imu(sub_info):
    sensor_log.append({
        "timestamp": time.time(),
        "sensor": "imu",
        "data": sub_info
    })

# ฟังก์ชัน callback สำหรับ ESC (ข้อมูลมอเตอร์)
def sub_info_status(sub_info):
    sensor_log.append({
        "timestamp": time.time(),
        "sensor": "esc",
        "data": sub_info
    })

# ฟังก์ชัน callback สำหรับ status (สถานะอื่นๆ)
def sub_info_esc(sub_info):
    sensor_log.append({
        "timestamp": time.time(),
        "sensor": "status",
        "data": sub_info
    })

# ---------- โซนสำหรับควบคุมการเดินของหุ่นยนต์ ----------
if __name__ == '__main__':
    ep_robot = robot.Robot()  # สร้างอ็อบเจ็กต์หุ่นยนต์
    ep_robot.initialize(conn_type="ap")  # เชื่อมต่อหุ่นยนต์
    ep_chassis = ep_robot.chassis  # เข้าถึงส่วนควบคุมการเคลื่อนที่

    # Subscribe ข้อมูลเซนเซอร์ทุกตัว (เริ่มเก็บข้อมูลตั้งแต่เริ่มงาน)
    ep_chassis.sub_attitude(freq=5, callback=sub_info_attitude)   # attitude 5Hz
    ep_chassis.sub_position(freq=5, callback=sub_info_position)   # position 5Hz
    ep_chassis.sub_imu(freq=10, callback=sub_info_imu)            # imu 10Hz
    ep_chassis.sub_esc(freq=10, callback=sub_info_status)         # esc 10Hz
    ep_chassis.sub_status(freq=10, callback=sub_info_esc)         # status 10Hz

    # ---------- โซนการเดินวนกระเบื้อง 4 แผ่น ----------
    TILE_SIZE = 0.6   # ขนาดกระเบื้อง 60 ซม. (เมตร)
    NUM_TILES = 4     # จำนวนกระเบื้อง
    NUM_ROUNDS = 1    # จำนวนรอบที่ต้องการเดินวน
    SPEED = 0.8    # ความเร็วในการเดิน (เมตร/วินาที)

    for round_idx in range(NUM_ROUNDS):  # วนตามจำนวนรอบ
        for tile_idx in range(NUM_TILES):  # วนตามจำนวนกระเบื้อง
            # เดินตรงไปข้างหน้า TILE_SIZE เมตร
            ep_chassis.move(x=TILE_SIZE, y=0, z=0, xy_speed=SPEED).wait_for_completed(timeout=6.0)
            time.sleep(0.5)  # พักครู่เพื่อให้หุ่นยนต์หยุดนิ่ง
            # หมุนขวา 90 องศา (z=-90)
            ep_chassis.move(x=0, y=0, z=-90.5, z_speed=45).wait_for_completed(timeout=6.0)
            time.sleep(0.5)
        print(f"Completed round {round_idx + 1}")  # แสดงผลเมื่อจบรอบ

    # ---------- ปิดการ subscribe ข้อมูลเซนเซอร์ ----------
    ep_chassis.unsub_status()
    ep_chassis.unsub_esc()
    ep_chassis.unsub_imu()
    ep_chassis.unsub_attitude()
    ep_chassis.unsub_position()

    ep_robot.close()  # ปิดการเชื่อมต่อหุ่นยนต์

    # ---------- โซนบันทึกข้อมูลเซนเซอร์ลงไฟล์ CSV ----------
    # สร้างไฟล์ sensor_log.csv และเขียนข้อมูลทั้งหมดลงไฟล์
    with open("sensor_log.csv", "w", newline='', encoding="utf-8") as csvfile:
        fieldnames = ["timestamp", "sensor", "data"]
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()
        for entry in sensor_log:
            writer.writerow(entry)

    print("บันทึกข้อมูลเซนเซอร์ทั้งหมดลง sensor_log.csv เรียบร้อยแล้ว")