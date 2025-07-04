from robomaster import robot
import time

if __name__ == '_main_':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")

    ep_chassis = ep_robot.chassis

    tile_len = 0.58  # 60 เซนติเมตร
    turn_angle = -91.5  # ปรับมุมเลี้ยวให้มากขึ้น
    speed = 0.7 # ลดความเร็วลง
    #1
    ep_chassis.move(x=tile_len, y=0, z=0, xy_speed=speed).wait_for_completed(timeout=5.0)
    time.sleep(1.0)  # เพิ่มเวลาหน่วงให้หยุดนิ่งมากขึ้น
    ep_chassis.move(x=0, y=0, z=turn_angle, z_speed=30).wait_for_completed(timeout=5.0)
    time.sleep(1.0)  # เพิ่มเวลาหน่วงหลังเลี้ยว
    #2
    ep_chassis.move(x=tile_len-0.02, y=0, z=0, xy_speed=speed).wait_for_completed(timeout=5.0)
    time.sleep(1.0)  # เพิ่มเวลาหน่วงให้หยุดนิ่งมากขึ้น
    ep_chassis.move(x=0, y=0, z=turn_angle, z_speed=30).wait_for_completed(timeout=5.0)
    time.sleep(1.0)  # เพิ่มเวลาหน่วงหลังเลี้ยว
    #3
    ep_chassis.move(x=tile_len, y=0, z=0, xy_speed=speed).wait_for_completed(timeout=5.0)
    time.sleep(1.0)  # เพิ่มเวลาหน่วงให้หยุดนิ่งมากขึ้น
    ep_chassis.move(x=0, y=0, z=turn_angle, z_speed=30).wait_for_completed(timeout=5.0)
    time.sleep(1.0)  # เพิ่มเวลาหน่วงหลังเลี้ยว

    # เดินไปกระเบื้องที่ 4 (ไม่ต้องเลี้ยว)
    ep_chassis.move(x=tile_len, y=0, z=0, xy_speed=speed).wait_for_completed(timeout=5.0)
    time.sleep(1.0)

    # หันกลับไปทางเดิม (เลี้ยวขวาอีกครั้ง)
    ep_chassis.move(x=0, y=0, z=turn_angle, z_speed=30).wait_for_completed(timeout=5.0)
    time.sleep(1.0)

    ep_robot.close()