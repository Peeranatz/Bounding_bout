from robomaster import robot
import time
import pandas as pd
import os
import signal
import sys

# ==============================================
# ส่วนตั้งค่าและกำหนดค่าพารามิเตอร์
# ==============================================

# ค่าคงที่การเคลื่อนที่
TILE_LEN = 0.587  # ความยาวกระเบื้อง (เมตร)
TURN_ANGLE = -91.5  # มุมการหมุน (องศา)
MOVE_SPEED = 0.7  # ความเร็วเคลื่อนที่ (เมตร/วินาที)
TURN_SPEED = 30  # ความเร็วหมุน (องศา/วินาที)
MOVE_TIMEOUT = 5.0  # เวลารอสูงสุดสำหรับการเคลื่อนที่ (วินาที)
PAUSE_TIME = 1.0  # เวลาหน่วงระหว่างการเคลื่อนที่ (วินาที)

# ค่าคงที่เซ็นเซอร์
SENSOR_FREQ = 5  # ความถี่การรับข้อมูลเซ็นเซอร์ (Hz)
OUTPUT_DIR = r"D:\work\robomaster\RoboMaster-SDK\examples\02_chassis"  # โฟลเดอร์เก็บข้อมูล

# ตัวแปรควบคุมการทำงาน
terminate_program = False  # ตัวแปรควบคุมการหยุดโปรแกรม
start_robot_time = 0  # เวลาเริ่มต้นการทำงาน

# ==============================================
# ส่วนจัดการเซ็นเซอร์
# ==============================================

# ตัวแปรเก็บข้อมูลเซ็นเซอร์
position_records = []
attitude_records = []
imu_records = []
esc_records = []
status_records = []


def signal_handler(sig, frame):
    """จัดการสัญญาณ Ctrl+C"""
    global terminate_program
    terminate_program = True


def sub_info_Position(sub_info):
    """Callback สำหรับข้อมูลตำแหน่ง"""
    global start_robot_time
    current_time = time.time() - start_robot_time
    position_records.append(
        {
            "timestamp": current_time,
            "pos_x_m": sub_info[0],
            "pos_y_m": sub_info[1],
            "pos_z_deg_from_start": sub_info[2],
        }
    )


def sub_info_Attitude(sub_info):
    """Callback สำหรับข้อมูลมุมทิศทาง"""
    global start_robot_time
    current_time = time.time() - start_robot_time
    attitude_records.append(
        {
            "timestamp": current_time,
            "pitch_deg": sub_info[0],
            "roll_deg": sub_info[1],
            "yaw_deg": sub_info[2],
        }
    )


def sub_info_IMU(sub_info):
    """Callback สำหรับข้อมูล IMU"""
    global start_robot_time
    current_time = time.time() - start_robot_time
    imu_records.append(
        {
            "timestamp": current_time,
            "acc_x_g": sub_info[0],
            "acc_y_g": sub_info[1],
            "acc_z_g": sub_info[2],
            "gyro_x_dps": sub_info[3],
            "gyro_y_dps": sub_info[4],
            "gyro_z_dps": sub_info[5],
        }
    )


def sub_info_ESC(sub_info):
    """Callback สำหรับข้อมูล ESC"""
    global start_robot_time
    current_time = time.time() - start_robot_time
    record = {"timestamp": current_time}

    if len(sub_info) >= 4 and all(
        isinstance(item, (list, tuple)) and len(item) >= 4 for item in sub_info[:4]
    ):
        wheel_labels = ["fl", "fr", "bl", "br"]
        for i, label in enumerate(wheel_labels):
            record[f"esc_state_{label}"] = sub_info[0][i]
            record[f"esc_speed_{label}"] = sub_info[1][i]
            record[f"esc_angle_{label}"] = sub_info[2][i]
            record[f"esc_current_{label}"] = sub_info[3][i]
    else:
        record["raw_esc_data"] = str(sub_info)
    esc_records.append(record)


def sub_info_Status(sub_info):
    """Callback สำหรับข้อมูลสถานะ"""
    global start_robot_time
    current_time = time.time() - start_robot_time
    record = {"timestamp": current_time}
    status_keys = [
        "chassis_is_moving",
        "chassis_force_protected",
        "chassis_impact_protected",
        "chassis_is_static",
        "chassis_is_unbalanced",
        "chassis_speed_x",
        "chassis_speed_y",
        "chassis_angle_speed",
        "chassis_temp_overheat",
        "chassis_battery_low",
        "chassis_firmware_error",
    ]
    for i, val in enumerate(sub_info):
        record[f"status_{status_keys[i] if i < len(status_keys) else f'val_{i}'}"] = val
    status_records.append(record)


def save_sensor_data():
    """บันทึกข้อมูลเซ็นเซอร์ลงไฟล์ CSV"""
    data_sources = {
        "position": position_records,
        "attitude": attitude_records,
        "imu": imu_records,
        "esc": esc_records,
        "status": status_records,
    }
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    for name, records in data_sources.items():
        if records:
            pd.DataFrame(records).to_csv(
                os.path.join(OUTPUT_DIR, f"robomaster_{name}_data.csv"), index=False
            )


# ==============================================
# ส่วนควบคุมการเคลื่อนที่
# ==============================================


def custom_movement(chassis):
    """ฟังก์ชันควบคุมการเคลื่อนที่แบบกำหนดเอง"""
    # ส่วนที่ 1
    chassis.move(x=TILE_LEN, y=0, z=0, xy_speed=MOVE_SPEED).wait_for_completed(
        timeout=MOVE_TIMEOUT
    )
    time.sleep(PAUSE_TIME)
    chassis.move(x=0, y=0, z=TURN_ANGLE, z_speed=TURN_SPEED).wait_for_completed(
        timeout=MOVE_TIMEOUT
    )
    time.sleep(PAUSE_TIME)

    # ส่วนที่ 2
    chassis.move(x=TILE_LEN - 0.02, y=0, z=0, xy_speed=MOVE_SPEED).wait_for_completed(
        timeout=MOVE_TIMEOUT
    )
    time.sleep(PAUSE_TIME)
    chassis.move(x=0, y=0, z=TURN_ANGLE, z_speed=TURN_SPEED).wait_for_completed(
        timeout=MOVE_TIMEOUT
    )
    time.sleep(PAUSE_TIME)

    # ส่วนที่ 3
    chassis.move(x=TILE_LEN, y=0, z=0, xy_speed=MOVE_SPEED).wait_for_completed(
        timeout=MOVE_TIMEOUT
    )
    time.sleep(PAUSE_TIME)
    chassis.move(x=0, y=0, z=TURN_ANGLE, z_speed=TURN_SPEED).wait_for_completed(
        timeout=MOVE_TIMEOUT
    )
    time.sleep(PAUSE_TIME)

    # ส่วนที่ 4
    chassis.move(x=TILE_LEN, y=0, z=0, xy_speed=MOVE_SPEED).wait_for_completed(
        timeout=MOVE_TIMEOUT
    )
    time.sleep(PAUSE_TIME)
    chassis.move(x=0, y=0, z=TURN_ANGLE, z_speed=TURN_SPEED).wait_for_completed(
        timeout=MOVE_TIMEOUT
    )
    time.sleep(PAUSE_TIME)


# ==============================================
# ส่วนหลักการทำงาน
# ==============================================

if __name__ == "__main__":
    # ตั้งค่าการจัดการสัญญาณ
    signal.signal(signal.SIGINT, signal_handler)

    try:
        # เริ่มการเชื่อมต่อกับหุ่นยนต์
        ep_robot = robot.Robot()
        ep_robot.initialize(conn_type="ap")

        # เตรียมเซ็นเซอร์
        chassis = ep_robot.chassis
        chassis.sub_position(freq=SENSOR_FREQ, callback=sub_info_Position)
        chassis.sub_attitude(freq=SENSOR_FREQ, callback=sub_info_Attitude)
        chassis.sub_imu(freq=SENSOR_FREQ, callback=sub_info_IMU)
        chassis.sub_esc(freq=SENSOR_FREQ, callback=sub_info_ESC)
        chassis.sub_status(freq=SENSOR_FREQ, callback=sub_info_Status)

        # เริ่มบันทึกเวลา
        start_robot_time = time.time()

        # รันการเคลื่อนที่
        custom_movement(chassis)

    except Exception:
        pass

    finally:
        # ส่วนทำความสะอาดทรัพยากร
        if "ep_robot" in locals():
            try:
                # หยุดการเคลื่อนที่
                chassis.move(x=0, y=0, z=0, xy_speed=0, z_speed=0)

                # หยุดรับข้อมูลเซ็นเซอร์
                chassis.unsub_position()
                chassis.unsub_attitude()
                chassis.unsub_imu()
                chassis.unsub_esc()
                chassis.unsub_status()

                # บันทึกข้อมูลเซ็นเซอร์
                save_sensor_data()

                # ปิดการเชื่อมต่อ
                ep_robot.close()
            except Exception:
                pass
