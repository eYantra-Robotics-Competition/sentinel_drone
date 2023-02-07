#!/usr/bin/env python3

"""
Driver for the drone
"""

# standard imports
import time

# third-party imports
import rospy
from sentinel_drone_driver.msg import RCMessage
from sentinel_drone_driver.srv import CommandBool, CommandBoolResponse
from yamspy import MSPy

# app imports
from utils import FCDriver

SERIAL_PORT = "/dev/ttyAMA0"
LOGLEVEL = "WARNING"
ELAPSED_TIME_FOR_AUTO_DISARM_ON_INACTIVITY = 1

DEFAULT_ROLL_VALUE = 1500
DEFAULT_PITCH_VALUE = 1500
DEFAULT_YAW_VALUE = 1500
DEFAULT_THROTTLE_VALUE = 900


class DroneDriver:
    def __init__(self, board):
        self.board = board

        self.CMDS = {
            "roll": DEFAULT_ROLL_VALUE,
            "pitch": DEFAULT_PITCH_VALUE,
            "throttle": DEFAULT_THROTTLE_VALUE,
            "yaw": DEFAULT_YAW_VALUE,
            "aux1": 900,
            "aux2": 1500,
            "aux3": 1500,
            "aux4": 1500,
        }

        node_name = "sentinel_drone_driver"
        rospy.init_node(node_name, log_level=rospy.INFO)
        rospy.on_shutdown(self.shutdown_hook)
        rospy.loginfo(f"Node started /{node_name}")

        self.last_msg_received_time = rospy.get_rostime().secs

        rospy.Subscriber(
            "/sentinel_drone/rc_command", RCMessage, self.rc_command_topic_callback
        )
        rospy.Service(
            "/sentinel_drone/cmd/arming", CommandBool, self.arming_service_callback
        )
        rospy.Service(
            "/sentinel_drone/cmd/reboot", CommandBool, self.reboot_service_callback
        )

        try:
            if board == 1:
                raise ("Some error occured")

            self.FC = FCDriver(self.board)

        except Exception as err:
            rospy.logerr(err)
        else:
            self.board = board

    def rc_command_topic_callback(self, msg):
        self.last_msg_received_time = rospy.get_rostime().secs

        self.CMDS["roll"] = msg.rc_roll
        self.CMDS["pitch"] = msg.rc_pitch
        self.CMDS["throttle"] = msg.rc_throttle
        self.CMDS["yaw"] = msg.rc_yaw

        rospy.loginfo(self.last_msg_received_time)

    def reboot_service_callback(self, req):
        if req.value:
            self.FC.reboot()
        # TODO: return appropriate error values
        success = True
        result = 0
        return CommandBoolResponse(success=success, result=result)

    def arming_service_callback(self, req):
        if req.value:
            self.arm()
        else:
            self.disarm()
        # TODO: return appropriate error values
        success = True
        result = 0
        return CommandBoolResponse(success=success, result=result)

    def shutdown_hook(self):
        rospy.loginfo("Calling shutdown hook")
        self.disarm()

    def arm(self):
        rospy.loginfo("Arming drone")
        self.CMDS["aux1"] = 1800

    def disarm(self):
        rospy.loginfo("Disarming drone")
        self.CMDS["aux1"] = 1000
        self.push_to_fc(None)
        self.CMDS["roll"] = DEFAULT_ROLL_VALUE
        self.CMDS["pitch"] = DEFAULT_PITCH_VALUE
        self.CMDS["throttle"] = DEFAULT_THROTTLE_VALUE
        self.CMDS["yaw"] = DEFAULT_YAW_VALUE

    def push_to_fc(self, event):
        self.FC.push_data(self.CMDS)

    @property
    def is_armed(self):
        arm_status = self.board.bit_check(self.board.CONFIG["mode"], 0)
        rospy.logdebug("ARMED: %s", arm_status)
        return arm_status


def main():
    rospy.loginfo("Interfacing with FC...")
    with MSPy(device=SERIAL_PORT, loglevel=LOGLEVEL, baudrate=115200) as board:
        sentinel_drone = DroneDriver(board)
        rospy.Timer(rospy.Duration(0.01), sentinel_drone.push_to_fc)
        while not rospy.is_shutdown():
            if sentinel_drone.is_armed:
                time_diff = (
                    rospy.get_rostime().secs - sentinel_drone.last_msg_received_time
                )
                if time_diff > ELAPSED_TIME_FOR_AUTO_DISARM_ON_INACTIVITY:
                    rospy.loginfo("Inactivity detected")
                    sentinel_drone.disarm()


if __name__ == "__main__":
    main()
