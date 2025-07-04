# -*-coding:utf-8-*-
# Copyright (c) 2020 DJI.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License in the file LICENSE.txt or at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from robomaster import robot
import time
import pandas as pd
import os
import signal
import sys

# --- Configuration ---
TILE_SIZE_M = 0.6  # ขนาดของกระเบื้อง 1 ช่อง (เมตร)
SQUARE_SIDE_LENGTH_M = TILE_SIZE_M * 1  # เดินด้านละ 0.6 เมตร (1 ช่อง)
MOVE_SPEED_MPS = 0.7  # ความเร็วเดินตรง (เมตร/วินาที)
TURN_SPEED_DPS = 45  # ความเร็วหมุน (องศา/วินาที)
PAUSE_AFTER_ACTION_SEC = 0.5  # หยุดนิ่งหลังแต่ละการเคลื่อนที่/หมุน

SENSOR_FREQ = 5  # Hz (ความถี่ subscribe sensor) - จะมีผลกับความถี่ในการรับข้อมูลใน callback

OUTPUT_DIR = (
    r"D:\work\robomaster\RoboMaster-SDK\examples\02_chassis"  # Directory ที่จะบันทึกไฟล์ CSV
)

terminate_program = False
start_robot_time = 0  # จะเก็บเวลาเริ่มต้นเมื่อหุ่นยนต์เริ่มทำงาน


def signal_handler(sig, frame):
    global terminate_program
    print("\n[Ctrl+C ถูกกด] กำลังสั่งหยุดการทำงาน...")
    terminate_program = True


# --- Sensor Data Storage ---
# แต่ละประเภทเซ็นเซอร์จะมีลิสต์ของตัวเองเพื่อเก็บข้อมูลที่ได้รับ
position_records = []
attitude_records = []
imu_records = []
esc_records = []
status_records = []


# --- Callback Functions for each Sensor Type ---
# ฟังก์ชันเหล่านี้จะถูกเรียกโดยอัตโนมัติเมื่อได้รับข้อมูลเซ็นเซอร์ใหม่
def sub_info_Position(sub_info):
    # sub_info: (x, y, z) - x,y ตำแหน่งสัมพัทธ์, z คือมุม Yaw ที่วัดจากจุดเริ่มต้น (reset_position)
    global start_robot_time
    current_relative_time = time.time() - start_robot_time
    position_records.append(
        {
            "timestamp": current_relative_time,
            "pos_x_m": sub_info[0],
            "pos_y_m": sub_info[1],
            "pos_z_deg_from_start": sub_info[2],
        }
    )
    # print(f"Position: {sub_info}") # สามารถเปิดเพื่อ debug ได้


def sub_info_Attitude(sub_info):
    # sub_info: (pitch, roll, yaw) - มุมปัจจุบันของหุ่นยนต์
    global start_robot_time
    current_relative_time = time.time() - start_robot_time
    attitude_records.append(
        {
            "timestamp": current_relative_time,
            "pitch_deg": sub_info[0],
            "roll_deg": sub_info[1],
            "yaw_deg": sub_info[2],
        }
    )
    # print(f"Attitude: {sub_info}")


def sub_info_IMU(sub_info):
    # sub_info: (acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z)
    global start_robot_time
    current_relative_time = time.time() - start_robot_time
    imu_records.append(
        {
            "timestamp": current_relative_time,
            "acc_x_g": sub_info[0],  # g-force
            "acc_y_g": sub_info[1],
            "acc_z_g": sub_info[2],
            "gyro_x_dps": sub_info[3],  # degrees per second
            "gyro_y_dps": sub_info[4],
            "gyro_z_dps": sub_info[5],
        }
    )
    # print(f"IMU: {sub_info}")


def sub_info_ESC(sub_info):
    # ESC data format: ([state], [speed], [angle], [current])
    # sub_info[0] is state, sub_info[1] is speed, sub_info[2] is angle, sub_info[3] is current
    # Each of these is a list/tuple of 4 values (for 4 wheels: front-left, front-right, back-left, back-right)
    global start_robot_time
    current_relative_time = time.time() - start_robot_time
    record = {"timestamp": current_relative_time}

    # ตรวจสอบให้แน่ใจว่า sub_info มี 4 ส่วนหลักและแต่ละส่วนมี 4 ค่าสำหรับล้อ
    if len(sub_info) >= 4 and all(
        isinstance(item, (list, tuple)) and len(item) >= 4 for item in sub_info[:4]
    ):
        wheel_labels = ["fl", "fr", "bl", "br"]
        # Extract and add ESC data for each wheel
        for i, label in enumerate(wheel_labels):
            record[f"esc_state_{label}"] = sub_info[0][i]
            record[f"esc_speed_{label}"] = sub_info[1][i]
            record[f"esc_angle_{label}"] = sub_info[2][i]
            record[f"esc_current_{label}"] = sub_info[3][i]
    else:
        # Fallback if format is unexpected - useful for debugging
        record["raw_esc_data"] = str(sub_info)
    esc_records.append(record)
    # print(f"ESC: {sub_info}")


def sub_info_Status(sub_info):
    # sub_info: tuple ที่มี 11 ค่า (ตามที่คุณเคยแสดงใน Log)
    # ตัวอย่าง: (1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
    global start_robot_time
    current_relative_time = time.time() - start_robot_time
    record = {"timestamp": current_relative_time}

    # กำหนดชื่อคอลัมน์ตามความหมายที่พบเจอในเอกสารหรือจากประสบการณ์
    # หรือตั้งชื่อเป็น general flag หากไม่ทราบความหมายที่ชัดเจน
    status_meaningful_keys = [
        "chassis_is_moving",  # 0: stopped, 1: moving
        "chassis_force_protected",  # 1: protected (e.g., hit obstacle)
        "chassis_impact_protected",  # 1: protected from impact
        "chassis_is_static",  # 1: static
        "chassis_is_unbalanced",  # 1: unbalanced
        "chassis_speed_x",  # Speed on X-axis (m/s)
        "chassis_speed_y",  # Speed on Y-axis (m/s)
        "chassis_angle_speed",  # Angular speed (deg/s)
        "chassis_temp_overheat",  # 1: overheating
        "chassis_battery_low",  # 1: battery low
        "chassis_firmware_error",  # 1: firmware error
    ]

    for i, val in enumerate(sub_info):
        if i < len(status_meaningful_keys):
            record[f"status_{status_meaningful_keys[i]}"] = val
        else:
            record[f"status_val_{i}"] = val  # สำหรับค่าที่เกินกว่าที่ระบุ

    status_records.append(record)
    # print(f"Status: {sub_info}")


if __name__ == "__main__":
    os.makedirs(OUTPUT_DIR, exist_ok=True)  # สร้างโฟลเดอร์สำหรับบันทึกผลลัพธ์ถ้ายังไม่มี
    signal.signal(
        signal.SIGINT, signal_handler
    )  # ตั้งค่าให้รับสัญญาณ Ctrl+C เพื่อหยุดโปรแกรมอย่างสง่างาม

    ep_robot = None  # กำหนดค่าเริ่มต้นเป็น None เพื่อให้จัดการใน finally block ได้

    try:
        ep_robot = robot.Robot()
        # ตรวจสอบการเชื่อมต่อ: "ap" สำหรับ AP mode (เชื่อมต่อโดยตรงกับ RoboMaster hotspot),
        # "sta" สำหรับ Station mode (RoboMaster เชื่อมต่อ Wi-Fi เดียวกันกับคอมพิวเตอร์)
        ep_robot.initialize(conn_type="ap")

        ep_chassis = ep_robot.chassis
        ep_gimbal = ep_robot.gimbal  # จำเป็นสำหรับการ recenter gimbal

        print("RoboMaster เชื่อมต่อแล้วผ่านโหมด AP")
        print("กำลังเตรียมหุ่นยนต์...")
        # Recenter gimbal เพื่อให้มั่นใจว่าอยู่ในตำแหน่งเริ่มต้น
        ep_gimbal.recenter().wait_for_completed()

        # รีเซ็ตค่า odometry ของ chassis (ถ้า SDK รองรับเท่านั้น)
        ep_chassis.sub_position(freq=1, callback=lambda x: None)
        time.sleep(0.5)
        # ep_chassis.reset_position()  # <-- ลบหรือคอมเมนต์บรรทัดนี้ออก
        time.sleep(0.5)
        ep_chassis.unsub_position()
        time.sleep(0.5)

        print("หุ่นยนต์พร้อมทำงาน")

        start_robot_time = (
            time.time()
        )  # บันทึกเวลาเริ่มต้นเมื่อหุ่นยนต์พร้อมทำงานและเซ็นเซอร์เริ่มทำงาน

        # Subscribe sensor data - ตั้งค่าความถี่ตาม SENSOR_FREQ
        ep_chassis.sub_position(freq=SENSOR_FREQ, callback=sub_info_Position)
        ep_chassis.sub_attitude(freq=SENSOR_FREQ, callback=sub_info_Attitude)
        ep_chassis.sub_imu(freq=SENSOR_FREQ, callback=sub_info_IMU)
        ep_chassis.sub_esc(freq=SENSOR_FREQ, callback=sub_info_ESC)
        ep_chassis.sub_status(freq=SENSOR_FREQ, callback=sub_info_Status)

        print("\n--- เริ่มการเคลื่อนที่แบบสี่เหลี่ยม (4 ช่อง) และรับค่าเซนเซอร์ตลอดเวลา ---")
        while_count = 0
        while not terminate_program and while_count < 4:  # ทำซ้ำ 4 ครั้งเพื่อเดินเป็นสี่เหลี่ยม
            print(
                f"  > Segment {while_count + 1}/4: เดินหน้า {SQUARE_SIDE_LENGTH_M} เมตร"
            )
            # สั่งเดินหน้า
            ep_chassis.move(
                x=SQUARE_SIDE_LENGTH_M, y=0, z=0, xy_speed=MOVE_SPEED_MPS
            ).wait_for_completed()
            time.sleep(PAUSE_AFTER_ACTION_SEC)  # หยุดเล็กน้อยหลังจากการเคลื่อนที่

            # ตรวจสอบว่ามีคำสั่งหยุดโปรแกรม (Ctrl+C) หรือไม่ ก่อนที่จะดำเนินการต่อไป
            if terminate_program:
                break  # ออกจาก loop ทันที

            print(f"  > Segment {while_count + 1}/4: หมุนขวา 90 องศา")
            # สั่งหมุนขวา 90 องศา (ใช้ค่า -90 สำหรับการหมุนตามเข็มนาฬิกา)
            ep_chassis.move(
                x=0, y=0, z=-90, z_speed=TURN_SPEED_DPS
            ).wait_for_completed()
            time.sleep(PAUSE_AFTER_ACTION_SEC)  # หยุดเล็กน้อยหลังจากการหมุน
            while_count += 1
        print("--- เดินครบ 4 ช่อง (สี่เหลี่ยมสมบูรณ์) หรือหยุดการทำงานแล้ว ---")

    except Exception as e:
        print(f"เกิดข้อผิดพลาดระหว่างการเคลื่อนที่: {e}")
        # พยายามหยุดหุ่นยนต์หากเกิดข้อผิดพลาด
        if ep_robot and ep_robot.chassis:
            ep_robot.chassis.move(
                x=0, y=0, z=0, xy_speed=0, z_speed=0
            ).wait_for_completed()

    finally:
        if ep_robot:
            print("\nกำลังสั่งหยุดการเคลื่อนที่ของหุ่นยนต์ (สุดท้าย)...")
            if ep_robot.chassis:
                try:
                    ep_robot.chassis.move(
                        x=0, y=0, z=0, xy_speed=0, z_speed=0
                    ).wait_for_completed()
                except Exception:
                    pass
            print("กำลังยกเลิกการ Subscribe เซ็นเซอร์ทั้งหมด...")
            for unsub in [
                ep_chassis.unsub_status,
                ep_chassis.unsub_esc,
                ep_chassis.unsub_imu,
                ep_chassis.unsub_attitude,
                ep_chassis.unsub_position,
            ]:
                try:
                    unsub()
                except Exception:
                    pass
            ep_robot.close()
            print("RoboMaster ตัดการเชื่อมต่อแล้ว")

        # Save sensor data to separate CSV files
        print("\nกำลังบันทึกข้อมูลเซ็นเซอร์ลงไฟล์ CSV...")
        # Dictionary ที่เก็บชื่อไฟล์และลิสต์ข้อมูลที่เกี่ยวข้อง
        data_sources = {
            "position": position_records,
            "attitude": attitude_records,
            "imu": imu_records,
            "esc": esc_records,
            "status": status_records,
        }

        for name, records in data_sources.items():
            if records:
                df = pd.DataFrame(records)
                file_path = os.path.join(OUTPUT_DIR, f"robomaster_{name}_data.csv")
                df.to_csv(file_path, index=False)
                print(f"บันทึก {len(records)} รายการของ {name} ลงในไฟล์: {file_path}")
            else:
                print(f"ไม่มีข้อมูล {name} ที่จะบันทึก")

        sys.exit(0)
