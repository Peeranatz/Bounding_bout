# -*-coding:utf-8-*-
# Copyright (c) 2020 DJI.
#
# Licensed under the Apache License, Version 20 (the "License");
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
import threading
import signal
import sys

# --- Configuration ---
TILE_SIZE_M = 0.6  # ขนาดของกระเบื้อง 1 ช่อง ในหน่วยเมตร (60 cm = 0.6 m)
SQUARE_SIDE_LENGTH_M = TILE_SIZE_M * 1 # ด้านของสี่เหลี่ยม 2x2 ช่อง = 1.2 เมตร

MOVE_SPEED_MPS = 0.7 # ความเร็วในการเดินตรง (เมตร/วินาที)
TURN_SPEED_DPS = 45  # ความเร็วในการหมุน (องศา/วินาที)

LOG_INTERVAL_SEC = 0.1 # ช่วงเวลาในการบันทึกข้อมูลเซ็นเซอร์ (วินาที)
PAUSE_AFTER_ACTION_SEC = 0.5 # หยุดนิ่งชั่วครู่หลังจากแต่ละการเคลื่อนที่/หมุน

# กำหนด Path สำหรับไฟล์ CSV ให้ชัดเจน
OUTPUT_DIR = r"D:\work\robomaster\RoboMaster-SDK\examples\02_chassis"
OUTPUT_CSV_FILE_FULL_PATH = os.path.join(OUTPUT_DIR, "robomaster_sensor_data.csv")
ROTATION_LOG_CSV_FILE_FULL_PATH = os.path.join(OUTPUT_DIR, "robomaster_rotation_log.csv")


# Global flags for controlling threads and program termination
terminate_program = False
stop_collection_flag = False

# Variables for rotation tracking
current_yaw_offset = 0 # เพื่อเก็บค่า yaw ที่สะสม (มากกว่า 360 ได้)
last_recorded_yaw = 0
rotation_count = 0
last_rotation_time = time.time()
rotation_times = [] # To store time taken for each 360-degree rotation

# Signal handler for Ctrl+C
def signal_handler(sig, frame):
    global terminate_program
    print("\n[Ctrl+C ถูกกด] กำลังสั่งหยุดการทำงาน...")
    terminate_program = True

# --- Main Program ---
if __name__ == '__main__':
    # Ensure the output directory exists
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    # Register the signal handler for graceful exit on Ctrl+C
    signal.signal(signal.SIGINT, signal_handler)

    ep_robot = robot.Robot()
    
    # Initialize RoboMaster connection in AP mode
    try:
        ep_robot.initialize(conn_type="ap")
        
        # สร้างตัวแปรอ้างอิงสำหรับโมดูลต่างๆ ของหุ่นยนต์
        ep_chassis = ep_robot.chassis
        ep_gimbal = ep_robot.gimbal
        ep_blaster = ep_robot.blaster
        ep_led = ep_robot.led

        print("RoboMaster เชื่อมต่อแล้วผ่านโหมด AP")
        print("กำลังเตรียมหุ่นยนต์...")
        
        # รีเซ็ตป้อมปืนไปตรงกลาง เพื่อให้หุ่นยนต์อยู่ในสถานะเริ่มต้นที่แน่นอน
        ep_gimbal.recenter().wait_for_completed() 
        print("หุ่นยนต์พร้อมทำงาน")

    except Exception as e:
        print(f"!!! เกิดข้อผิดพลาดในการเชื่อมต่อ RoboMaster: {e} !!!")
        print("โปรดตรวจสอบสิ่งเหล่านี้:")
        print("  1. RoboMaster เปิดอยู่และเชื่อมต่อกับ Router เดียวกับคอมพิวเตอร์ของคุณแล้ว")
        print("  2. คอมพิวเตอร์ของคุณไม่ได้ถูกไฟร์วอลล์บล็อกการเชื่อมต่อ")
        print("หากยังไม่ทำงาน ลองระบุ IP Address ใน initialize() เช่น ep_robot.initialize(conn_type='ap', addr='192.168.1.100')")
        sys.exit(1) # ออกจากโปรแกรมด้วยรหัสข้อผิดพลาดหากเชื่อมต่อไม่ได้

    # Data collection list and start time
    sensor_data_records = []
    start_time = time.time()
    main_loop_count = 0 # ตัวแปรสำหรับนับจำนวนรอบสี่เหลี่ยม

    # Function to collect sensor data in a separate thread
    def collect_data():
        global current_yaw_offset, last_recorded_yaw, rotation_count, last_rotation_time

        current_time_ms = int((time.time() - start_time) * 1000) # Timestamp in milliseconds
        
        # Chassis data: Position (odometry) and Speed
        x, y, z_current = ep_chassis.get_position() # z_current is the Yaw angle
        vx, vy, vz = ep_chassis.get_speed()

        # Gimbal data: Pitch and Yaw angles
        pitch, yaw_gimbal = ep_gimbal.get_attitude()

        # --- Rotation Tracking Logic ---
        # Normalize z_current to be within -180 to 180 for comparison
        z_normalized = (z_current + 180) % 360 - 180

        # Calculate difference in yaw, accounting for 360-degree wrap-around
        yaw_diff = z_normalized - last_recorded_yaw
        if yaw_diff > 180:
            yaw_diff -= 360
        elif yaw_diff < -180:
            yaw_diff += 360
        
        current_yaw_offset += yaw_diff
        last_recorded_yaw = z_normalized

        # Check for full 360-degree rotation (either direction)
        if abs(current_yaw_offset) >= 360:
            rotation_count_before = rotation_count
            rotation_count = int(abs(current_yaw_offset) // 360) # Count how many full 360 rotations
            
            # If a new full rotation is detected
            if rotation_count > rotation_count_before:
                current_rotation_time = time.time()
                time_taken_for_rot = current_rotation_time - last_rotation_time
                rotation_times.append({
                    'rotation_number': rotation_count,
                    'time_taken_sec': round(time_taken_for_rot, 3),
                    'timestamp_ms': current_time_ms
                })
                print(f"[Rotation Log] รอบที่ {rotation_count} ใช้เวลา: {time_taken_for_rot:.3f} วินาที")
                last_rotation_time = current_rotation_time
                # Reset offset for the remaining angle within 360
                current_yaw_offset = current_yaw_offset % 360 
                if current_yaw_offset > 180: current_yaw_offset -= 360 # Keep normalized if it jumped

        # --- End Rotation Tracking Logic ---


        record = {
            'timestamp_ms': current_time_ms,
            'chassis_x_m': x,
            'chassis_y_m': y,
            'chassis_z_deg': z_current, # Raw Yaw angle
            'chassis_vx_mps': vx,
            'chassis_vy_mps': vy,
            'chassis_vz_dps': vz, # Angular velocity (Yaw rate)
            'gimbal_pitch_deg': pitch,
            'gimbal_yaw_deg': yaw_gimbal,
            'accumulated_yaw_deg': current_yaw_offset # Accumulated yaw for debugging rotation
        }
        sensor_data_records.append(record) # Add record to list

    # Thread function for continuous data collection
    def data_collection_loop():
        global stop_collection_flag
        while not stop_collection_flag:
            try:
                collect_data()
            except Exception as e:
                # print(f"Error collecting data: {e}") 
                pass # Allow thread to continue even if a single data collection fails
            time.sleep(LOG_INTERVAL_SEC)

    # Start the data collection thread
    collection_thread = threading.Thread(target=data_collection_loop)
    collection_thread.start()

    try:
        print("\n--- เริ่มการเคลื่อนที่แบบสี่เหลี่ยมตามโจทย์ (1 รอบแล้วหยุด) ---")
        print("หุ่นยนต์จะเคลื่อนที่จากล่างซ้าย, ขึ้นบน, ไปขวา, ลงล่าง, กลับซ้าย")

        # Main loop to run only ONE square
        # Loop for 4 segments to complete one square
        for segment in range(4):
            if terminate_program: break # Check termination flag before each segment/action

            # --- 1. เดินตรงไปข้างหน้า (สัมพันธ์กับทิศทางปัจจุบัน) ---
            print(f"  > Segment {segment + 1}/4: เดินหน้า {SQUARE_SIDE_LENGTH_M} เมตร")
            ep_chassis.move(x=SQUARE_SIDE_LENGTH_M, y=0, z=0, xy_speed=MOVE_SPEED_MPS).wait_for_completed()
            time.sleep(PAUSE_AFTER_ACTION_SEC) # หยุดนิ่งชั่วครู่

            # --- 2. หมุนขวา 90 องศา (ทุกครั้ง รวมถึงครั้งสุดท้ายเพื่อกลับทิศทางเดิม) ---
            print(f"  > Segment {segment + 1}/4: หมุนขวา 90 องศา")
            ep_chassis.move(x=0, y=0, z=-90, z_speed=TURN_SPEED_DPS).wait_for_completed()
            time.sleep(PAUSE_AFTER_ACTION_SEC) # หยุดนิ่งชั่วครู่
        
        # If the code reaches here, one complete square loop has finished
        if not terminate_program: # Only count if not terminated by Ctrl+C
            main_loop_count += 1
            print(f"--- เดินครบ {main_loop_count} รอบ (สี่เหลี่ยมสมบูรณ์) แล้ว ---")
        
        # หลังจากเดินครบ 1 รอบ ให้ตั้งค่า terminate_program เป็น True เพื่อให้โปรแกรมหยุด
        terminate_program = True 

    except Exception as e:
        print(f"เกิดข้อผิดพลาดระหว่างการเคลื่อนที่: {e}")
        # Ensure robot stops completely if an error occurred during movement
        ep_chassis.move(x=0, y=0, z=0, xy_speed=0, z_speed=0).wait_for_completed()
    
    finally:
        # Ensure robot stops completely regardless of how the program ends
        print("\nกำลังสั่งหยุดการเคลื่อนที่ของหุ่นยนต์...")
        ep_chassis.move(x=0, y=0, z=0, xy_speed=0, z_speed=0).wait_for_completed() 

        # Stop the data collection thread gracefully
        stop_collection_flag = True
        if collection_thread.is_alive():
            collection_thread.join(timeout=3) # Give thread some time to finish

        # Disconnect from RoboMaster (important for releasing resources)
        ep_robot.close() 
        print("RoboMaster ตัดการเชื่อมต่อแล้ว")

        print(f"\n--- สรุปผลการทำงาน ---")
        print(f"จำนวนรอบสี่เหลี่ยมที่เดินครบถ้วน: {main_loop_count} รอบ")
        print(f"จำนวนรอบการหมุน 360 องศาที่ตรวจจับได้: {rotation_count} รอบ")


        # Process and Save Data to CSV
        if sensor_data_records:
            df = pd.DataFrame(sensor_data_records)
            
            print("\n--- Sensor Data Log (Full) ---")
            # print(df.to_string()) # Uncomment this line if you want to print all data to console (can be very long)
            
            # Save data to CSV file
            df.to_csv(OUTPUT_CSV_FILE_FULL_PATH, index=False)
            print(f"\nบันทึกข้อมูลเซ็นเซอร์ทั้งหมดลงในไฟล์: {OUTPUT_CSV_FILE_FULL_PATH}")
        else:
            print("ไม่มีข้อมูลเซ็นเซอร์ที่จะบันทึก")

        # Save rotation log data to CSV
        if rotation_times:
            df_rot = pd.DataFrame(rotation_times)
            df_rot.to_csv(ROTATION_LOG_CSV_FILE_FULL_PATH, index=False)
            print(f"บันทึกข้อมูลเวลาการหมุน 360 องศาลงในไฟล์: {ROTATION_LOG_CSV_FILE_FULL_PATH}")
        else:
            print("ไม่มีข้อมูลการหมุน 360 องศาที่จะบันทึก")

        sys.exit(0)