import time
import robomaster
from robomaster import robot

# ฟังก์ชันสำหรับจัดการข้อมูลของ attitude
def sub_info_attitude(sub_info):
    print("sub info: {0}".format(sub_info))

# ฟังก์ชันสำหรับจัดการข้อมูลของ position
def sub_info_position(sub_info):
    print("sub info: {0}".format(sub_info))

# ฟังก์ชันสำหรับจัดการข้อมูลของ IMU
def sub_info_imu(sub_info):
    print("sub info: {0}".format(sub_info))

# ฟังก์ชันสำหรับจัดการข้อมูลของ ESC
def sub_info_status(sub_info):
    print("sub info: {0}".format(sub_info))

# ฟังก์ชันสำหรับจัดการข้อมูลของสถานะ
def sub_info_esc(sub_info):
    print("sub info: {0}".format(sub_info))


if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="sta")

    ep_chassis = ep_robot.chassis

    
    ep_chassis.sub_position(freq=1, callback=sub_info_attitude)

    
    ep_chassis.sub_attitude(freq=5, callback=sub_info_position)

    
    ep_chassis.sub_imu(freq=10, callback=sub_info_imu)

    
    ep_chassis.sub_esc(freq=20, callback=sub_info_status)

    
    ep_chassis.sub_status(freq=50, callback=sub_info_esc)

    time.sleep(10)

    ep_chassis.unsub_status()
    ep_chassis.unsub_esc()
    ep_chassis.unsub_imu()
    ep_chassis.unsub_attitude()
    ep_chassis.unsub_position()

    ep_robot.close()
