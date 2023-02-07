#!/usr/bin/env python3

"""
Utilities to run the drone
"""

import rospy
from sentinel_drone_driver.msg import RCMessage
from yamspy import MSPy

CMDS_ORDER = ["roll", "pitch", "throttle", "yaw", "aux1", "aux2", "aux3", "aux4"]


class FCDriver:
    def __init__(self, board):
        self.board = board

        command_list = [
            "MSP_API_VERSION",
            "MSP_FC_VARIANT",
            "MSP_FC_VERSION",
            "MSP_BUILD_INFO",
            "MSP_BOARD_INFO",
            "MSP_UID",
            "MSP_ACC_TRIM",
            "MSP_NAME",
            "MSP_STATUS",
            "MSP_STATUS_EX",
            "MSP_BATTERY_CONFIG",
            "MSP_BATTERY_STATE",
            "MSP_BOXNAMES",
        ]
        if self.board.INAV:
            command_list.append("MSPV2_INAV_ANALOG")
            command_list.append("MSP_VOLTAGE_METER_CONFIG")

        for msg in command_list:
            if self.board.send_RAW_msg(MSPy.MSPCodes[msg], data=[]):
                dataHandler = self.board.receive_msg()
                self.board.process_recv_data(dataHandler)

    def reboot(self):
        self.board.reboot()

    def push_data(self, CMDS):
        if self.board.send_RAW_RC([CMDS[ki] for ki in CMDS_ORDER]):
            dataHandler = self.board.receive_msg()
            self.board.process_recv_data(dataHandler)
