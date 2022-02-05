from __future__ import print_function

#import rclpy
import odrive
from odrive.enums import *
import time
import math

def main(args=None):
    #rclpy.init(args=args)

    #node = rclpy.create_node('nsra_calibration')

    print("Connecting...")
    odrv0 = odrive.find_any(serial_number="205F387F304E")
    print("Connected to ODrive 0")
    odrv1 = odrive.find_any(serial_number="20573882304E")
    print("Connected to ODrive 1")
    odrv2 = odrive.find_any(serial_number="20843881304E")
    print("Connected to ODrive 2")

    odrv1.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
    odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
    odrv0.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE

    while odrv0.axis0.current_state != AXIS_STATE_IDLE:
        time.sleep(0.1)
    odrv1.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

    odrv0.axis1.requested_state = AXIS_STATE_HOMING

    #input("Press enter to continue...")
    while odrv0.axis1.current_state != AXIS_STATE_IDLE:
        time.sleep(0.1)

    odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    odrv0.axis1.controller.input_pos = 0

    time.sleep(3)

    odrv2.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
    odrv2.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
    odrv1.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE

    while odrv2.axis1.current_state != AXIS_STATE_IDLE:
        time.sleep(0.1)
    
    odrv2.axis1.requested_state = AXIS_STATE_HOMING

    #input("Press enter to continue...")
    while odrv2.axis1.current_state != AXIS_STATE_IDLE:
        time.sleep(0.1)

    odrv2.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    odrv2.axis1.controller.input_pos = 0

    odrv2.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    odrv1.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

    print("Calibration done!")

    #rclpy.spin(node)

    #node.destroy_node()
    #rclpy.shutdown()

if __name__ == '__main__':
    main()