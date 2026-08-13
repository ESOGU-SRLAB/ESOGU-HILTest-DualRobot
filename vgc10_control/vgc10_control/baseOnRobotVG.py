#!/usr/bin/env python3

import logging

from vgc10_msgs.msg import OnRobotVGInput

# Mod, ham register'in UST BAYTI olarak verilir; comModbusTcp reg = rmca + rvca
# diye topluyor. Yani %75 grip = 0x0100 + 75 = 0x014B (msg dokumanindaki ornek).
MODE_RELEASE = 0x0000
MODE_GRIP = 0x0100
MODE_IDLE = 0x0200
VALID_MODES = (MODE_RELEASE, MODE_GRIP, MODE_IDLE)


class onrobotbaseVG():
    """Base class (communication protocol agnostic) for sending commands
       and receiving the status of the OnRobot VG gripper.
    """

    def __init__(self):
        # Initiate output message as an empty list
        self.message = []

        # Note: after the instantiation,
        # a ".client" member must be added to the object

    def verifyCommand(self, command):
        """Verifies that the value of each variable satisfy its limits."""

        # Verify that each variable is in its correct range
        command.rvca = max(0, command.rvca)
        command.rvca = min(255, command.rvca)
        command.rvcb = max(0, command.rvcb)
        command.rvcb = min(255, command.rvcb)

        # Verify that the selected mode number is available.
        #
        # ESKİDEN rclpy.signal_shutdown() çağrılıyordu - o ROS1 (rospy) API'si,
        # rclpy'de YOK. Sonuç: geçersiz bir mod gelince sürücü uyarmak yerine
        # AttributeError ile ÖLÜYORDU ve gripper o anki durumunda kalıyordu
        # (13 Ağu 2026, gerçek hücrede yaşandı). Bir sürücünün kötü bir mesaj
        # yüzünden ölmesi, gerçek bir hücrede kabul edilemez.
        #
        # Artık geçersiz komut REDDEDİLİR: kanal boşta bırakılır (ne tutar ne
        # bırakır, yani elindeki parçayı düşürmez) ve sebep loglanır.
        for channel, mode in (("A", command.rmca), ("B", command.rmcb)):
            if mode not in VALID_MODES:
                logging.getLogger("onrobotbaseVG").error(
                    "Ch %s icin gecersiz mod %s (%#06x). Gecerli degerler: "
                    "0x0000 (release), 0x0100 (grip), 0x0200 (idle). "
                    "DIKKAT: 0/1/2 DEGIL - mod ust bayta kaydirilmis olmali. "
                    "Komut yok sayildi, kanal bosta birakiliyor.",
                    channel, mode, mode,
                )
                if channel == "A":
                    command.rmca, command.rvca = MODE_IDLE, 0
                else:
                    command.rmcb, command.rvcb = MODE_IDLE, 0

        # Return the modified command
        return command

    def refreshCommand(self, command):
        """Updates the command which will be sent
           during the next sendCommand() call.
        """

        # Limit the value of each variable
        command = self.verifyCommand(command)

        # Initiate command as an empty list
        self.message = []

        # Build the command with each output variable
        self.message.append(command.rmca)
        self.message.append(command.rvca)
        self.message.append(command.rmcb)
        self.message.append(command.rvcb)

    def sendCommand(self):
        """Sends the command to the Gripper."""

        self.client.sendCommand(self.message)

    def getStatus(self):
        """Requests the status from the gripper and
           return it in the OnRobotVGInput msg type.
        """

        # Acquire status from the Gripper
        status = self.client.getStatus()

        # Message to output
        message = OnRobotVGInput()

        # Assign the values to their respective variables
        message.gvca = status[0]
        message.gvcb = status[1]

        return message
