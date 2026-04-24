import socket
import numpy as np
import os
import re
import json
import threading
import time
from time import sleep
import requests

alarmControllerFile = "files/alarmController.json"
alarmServoFile = "files/alarmServo.json"

#brief dobot_v4_api:CRA\E6\CRAF\NovaLite
#author futingxing
#date 2025-12-15

# Port Feedback
MyType = np.dtype([('len', np.uint16,),
                   ('reserve', np.byte, (6, )),
                   ('DigitalInputs', np.uint64,),
                   ('DigitalOutputs', np.uint64,),
                   ('RobotMode', np.uint64,),
                   ('TimeStamp', np.uint64,),
                   ('RunTime', np.uint64,),
                   ('TestValue', np.uint64,),
                   ('reserve2', np.byte, (8, )),
                   ('SpeedScaling', np.float64,),
                   ('reserve3', np.byte, (16, )),
                   ('VRobot', np.float64, ),      
                   ('IRobot', np.float64,),
                   ('ProgramState', np.float64,),
                   ('SafetyOIn', np.uint16,),
                   ('SafetyOOut', np.uint16,),
                   ('reserve4', np.byte, (76, )),
                   ('QTarget', np.float64, (6, )),
                   ('QDTarget', np.float64, (6, )),
                   ('QDDTarget', np.float64, (6, )),
                   ('ITarget', np.float64, (6, )),
                   ('MTarget', np.float64, (6, )),
                   ('QActual', np.float64, (6, )),
                   ('QDActual', np.float64, (6, )),
                   ('IActual', np.float64, (6, )),
                   ('ActualTCPForce', np.float64, (6, )),
                   ('ToolVectorActual', np.float64, (6, )),
                   ('TCPSpeedActual', np.float64, (6, )),
                   ('TCPForce', np.float64, (6, )),
                   ('ToolVectorTarget', np.float64, (6, )),
                   ('TCPSpeedTarget', np.float64, (6, )),
                   ('MotorTemperatures', np.float64, (6, )),
                   ('JointModes', np.float64, (6, )),
                   ('VActual', np.float64, (6, )),
                   ('HandType', np.byte, (4, )),
                   ('User', np.byte,),
                   ('Tool', np.byte,),
                   ('RunQueuedCmd', np.byte,),
                   ('PauseCmdFlag', np.byte,),
                   ('VelocityRatio', np.byte,),
                   ('AccelerationRatio', np.byte,),
                   ('reserve5', np.byte, ),
                   ('XYZVelocityRatio', np.byte,),
                   ('RVelocityRatio', np.byte,),
                   ('XYZAccelerationRatio', np.byte,),
                   ('RAccelerationRatio', np.byte,),
                   ('reserve6', np.byte,(2,)),
                   ('BrakeStatus', np.byte,),
                   ('EnableStatus', np.byte,),
                   ('DragStatus', np.byte,),
                   ('RunningStatus', np.byte,),
                   ('ErrorStatus', np.byte,),
                   ('JogStatusCR', np.byte,),   
                   ('CRRobotType', np.byte,),
                   ('DragButtonSignal', np.byte,),
                   ('EnableButtonSignal', np.byte,),
                   ('RecordButtonSignal', np.byte,),
                   ('ReappearButtonSignal', np.byte,),
                   ('JawButtonSignal', np.byte,),
                   ('SixForceOnline', np.byte,),
                   ('CollisionState', np.byte,),
                   ('ArmApproachState', np.byte,),
                   ('J4ApproachState', np.byte,),
                   ('J5ApproachState', np.byte,),
                   ('J6ApproachState', np.byte,),
                   ('reserve7', np.byte, (61, )),
                   ('VibrationDisZ', np.float64,),
                   ('CurrentCommandId', np.uint64,),
                   ('MActual', np.float64, (6, )),
                   ('Load', np.float64,),
                   ('CenterX', np.float64,),
                   ('CenterY', np.float64,),
                   ('CenterZ', np.float64,),
                   ('UserValue[6]', np.float64, (6, )),
                   ('ToolValue[6]', np.float64, (6, )),
                   ('reserve8', np.byte, (8, )),
                   ('SixForceValue', np.float64, (6, )),
                   ('TargetQuaternion', np.float64, (4, )),
                   ('ActualQuaternion', np.float64, (4, )),
                   ('AutoManualMode', np.uint16, ),
                   ('ExportStatus', np.uint16, ),
                   ('SafetyState', np.byte, ),
                   ('reserve9', np.byte,(19,))
                   ])


# Read controller and servo alarm files
def alarmAlarmJsonFile():
    currrntDirectory = os.path.dirname(__file__)
    jsonContrellorPath = os.path.join(currrntDirectory, alarmControllerFile)
    jsonServoPath = os.path.join(currrntDirectory, alarmServoFile)

    with open(jsonContrellorPath, encoding='utf-8') as f:
        dataController = json.load(f)
    with open(jsonServoPath, encoding='utf-8') as f:
        dataServo = json.load(f)
    return dataController, dataServo


# TCP communication interface


class DobotApi:
    def __init__(self, ip, port, *args):
        self.ip = ip
        self.port = port
        self.socket_dobot = 0
        self.__globalLock = threading.Lock()
        if args:
            self.text_log = args[0]

        if self.port == 29999 or self.port == 30004 or self.port == 30005:
            try:
                self.socket_dobot = socket.socket()
                self.socket_dobot.connect((self.ip, self.port))
                self.socket_dobot.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 144000)
            except socket.error:
                print(socket.error)

        else:
            print(f"Connect to dashboard server need use port {self.port} !")

    def log(self, text):
        if self.text_log:
            print(text)

    def send_data(self, string):
       # self.log(f"Send to {self.ip}:{self.port}: {string}")
        try:
            self.socket_dobot.send(str.encode(string, 'utf-8'))
        except Exception as e:
            print(e)
            while True:
                try:
                    self.socket_dobot = self.reConnect(self.ip, self.port)
                    self.socket_dobot.send(str.encode(string, 'utf-8'))
                    break
                except Exception:
                    sleep(1)

    def wait_reply(self):
        """
        Read the return value
        """
        data = ""
        try:
            data = self.socket_dobot.recv(1024)
        except Exception as e:
            print(e)
            self.socket_dobot = self.reConnect(self.ip, self.port)

        finally:
            if len(data) == 0:
                data_str = data
            else:
                data_str = str(data, encoding="utf-8")
            # self.log(f'Receive from {self.ip}:{self.port}: {data_str}')
            return data_str

    def close(self):
        """
        Close the port
        """
        if (self.socket_dobot != 0):
            try:
                self.socket_dobot.shutdown(socket.SHUT_RDWR)
                self.socket_dobot.close()
            except socket.error as e:
                print(f"Error while closing socket: {e}")

    def sendRecvMsg(self, string):
        """
        send-recv Sync
        """
        with self.__globalLock:
            self.send_data(string)
            recvData = self.wait_reply()
            return recvData

    def __del__(self):
        self.close()

    def reConnect(self, ip, port):
        while True:
            try:
                socket_dobot = socket.socket()
                socket_dobot.connect((ip, port))
                break
            except Exception:
                sleep(1)
        return socket_dobot


# Control and motion command interface


class DobotApiDashboard(DobotApi):

    def __init__(self, ip, port, *args):
        super().__init__(ip, port, *args)

    def _fmt(self, v):
        if isinstance(v, (list, tuple)):
            return "{" + ",".join([self._fmt(x) for x in v]) + "}"
        if isinstance(v, float):
            return ("{:f}".format(v))
        if isinstance(v, int):
            return ("{:d}".format(v))
        return str(v)

    def _build_cmd(self, name, *args, **kwargs):
        parts = []
        for a in args:
            parts.append(self._fmt(a))
        for k, v in kwargs.items():
            parts.append(f"{k}={self._fmt(v)}")
        return f"{name}(" + ",".join(parts) + ")"

    def EnableRobot(self, load=0.0, centerX=0.0, centerY=0.0, centerZ=0.0, isCheck=-1,):
        """
            Optional parameter
            Parameter name     Type     Description
            load     double     Load weight. The value range should not exceed the load range of corresponding robot models. Unit: kg.
            centerX     double     X-direction eccentric distance. Range: -999 – 999, unit: mm.
            centerY     double     Y-direction eccentric distance. Range: -999 – 999, unit: mm.
            centerZ     double     Z-direction eccentric distance. Range: -999 – 999, unit: mm.
            isCheck     int     Check the load or not. 1: check, 0: not check. If set to 1, the robot arm will check whether the actual load is the same as the set load after it is enabled, and if not, it will be automatically disabled. 0 by default.
            The number of parameters that can be contained is as follows:
            0: no parameter (not set load weight and eccentric parameters when enabling the robot).
            1: one parameter (load weight).
            4: four parameters (load weight and eccentric parameters).
            5: five parameters (load weight, eccentric parameters, check the load or not).
                """
        string = 'EnableRobot('
        if load != 0:
            string = string + "{:f}".format(load)
            if centerX != 0 or centerY != 0 or centerZ != 0:
                string = string + ",{:f},{:f},{:f}".format(
                    centerX, centerY, centerZ)
                if isCheck != -1:
                    string = string + ",{:d}".format(isCheck)
        string = string + ')'
        return self.sendRecvMsg(string)

    def DisableRobot(self):
        """
        Disabled the robot
        """
        string = "DisableRobot()"
        return self.sendRecvMsg(string)

    def ClearError(self):
        """
        Clear controller alarm information
        Clear the alarms of the robot. After clearing the alarm, you can judge whether the robot is still in the alarm status according to RobotMode.
        Some alarms cannot be cleared unless you resolve the alarm cause or restart the controller.
        """
        string = "ClearError()"
        return self.sendRecvMsg(string)

    def PowerOn(self):
        """
        Powering on the robot
        Note: It takes about 10 seconds for the robot to be enabled after it is powered on.
        """
        string = "PowerOn()"
        return self.sendRecvMsg(string)

    def RunScript(self, project_name):
        """
        Run the script file
        project_name ：Script file name
        """
        string = "RunScript({:s})".format(project_name)
        return self.sendRecvMsg(string)

    def Stop(self):
        """
       Stop the delivered motion command queue or the RunScript command from running.
        """
        string = "Stop()"
        return self.sendRecvMsg(string)

    def Pause(self):
        """
       Pause the delivered motion command queue or the RunScript command from running.
        """
        string = "Pause()"
        return self.sendRecvMsg(string)

    def Continue(self):
        """
       Continue the paused motion command queue or the RunScript command from running.
        """
        string = "Continue()"
        return self.sendRecvMsg(string)

    def EmergencyStop(self, mode):
        """
       Stop the robot in an emergency. After the emergency stop, the robot arm will be disabled and then alarm. You need to release the emergency stop and clear the alarm to re-enable the robot arm.
       Required parameter
       Parameter name     Type     Description
        mode     int     E-Stop operation mode. 1: press the E-Stop, 0: release the E-Stop.
        """
        string = "EmergencyStop({:d})".format(mode)
        return self.sendRecvMsg(string)

    def BrakeControl(self, axisID, value):
        """
        Description
        Control the brake of specified joint. The joints automatically brake when the robot is stationary. If you need to drag the joints, you can switch on the brake,
        i.e. hold the joint manually in the disabled status and deliver the command to switch on the brake.
        Joint brake can be controlled only when the robot arm is disabled, otherwise, Error ID will return -1.
        Required parameter:
        Parameter name     Type     Description
        axisID     int     joint ID, 1: J1, 2: J2, and so on
        Value     int     Set the status of brake. 0: switch off brake (joints cannot be dragged). 1: switch on brake (joints can be dragged).
        """
        string = "BrakeControl({:d},{:d})".format(axisID, value)
        return self.sendRecvMsg(string)

    #####################################################################

    def SpeedFactor(self, speed):
        """
           6°/s
        Set the global speed ratio.
           Actual robot acceleration/speed ratio in jogging = value in Jog settings × global speed ratio.
           Example: If the joint speed set in the software is 12°/s and the global speed ratio is 50%, then the actual jog speed is 12°/s x 50% =
           6°/s
           Actual robot acceleration/speed ratio in playback = ratio set in motion command × value in Playback settings
            × global speed ratio.
           Example: If the coordinate system speed set in the software is 2000mm/s, the global speed ratio is 50%, and the speed set in the motion command is
           80%, then the actual speed is 2000mm/s x 50% x 80% = 800mm/s.
        If it is not set, the value set by the software before entering TCP/IP control mode will be adopted.
        Range: [1, 100].
        """
        string = "SpeedFactor({:d})".format(speed)
        return self.sendRecvMsg(string)

    def User(self, index):
        """
        Set the global tool coordinate system. You can select a tool coordinate system while delivering motion commands. If you do not specify the tool coordinate system, the global tool coordinate system will be used.
        If it is not set, the default global user coordinate system is User coordinate system 0.
        """
        string = "User({:d})".format(index)
        return self.sendRecvMsg(string)

    def SetUser(self, index, table):
        """
        Modify the specified user coordinate system.
        Required parameter:
        Parameter name     Type     Description
        index    int     user coordinate system index, range: [0,9]. The initial value of coordinate system 0 refers to the base coordinate system.
        table    string     user coordinate system after modification (format: {x, y, z, rx, ry, rz}), which is recommended to obtain through "CalcUser" command.
        """
        string = "SetUser({:d},{:s})".format(index, table)
        return self.sendRecvMsg(string)

    def CalcUser(self, index, matrix_direction, table):
        """
        Calculate the user coordinate system.
        Required parameter:
        Parameter name     Type     Description
        Index    int     user coordinate system index, range: [0,9]. The initial value of coordinate system 0 refers to the base coordinate system.
        matrix_direction    int    Calculation method. 1: left multiplication, indicating that the coordinate system specified by "index" deflects the value specified by "table" along the base coordinate system.
            0: right multiplication, indicating that the coordinate system specified by "index" deflects the value specified by "table" along itself.
        table    string     user coordinate system offset (format: {x, y, z, rx, ry, rz}).
        """
        string = "CalcUser({:d},{:d},{:s})".format(
            index, matrix_direction, table)
        return self.sendRecvMsg(string)

    def Tool(self, index):
        """
        Set the global tool coordinate system. You can select a tool coordinate system while delivering motion commands. If you do not specify the tool coordinate system, the global tool coordinate system will be used.
        If it is not set, the default global tool coordinate system is Tool coordinate system 0.
        """
        string = "Tool({:d})".format(index)
        return self.sendRecvMsg(string)

    def SetTool(self, index, table):
        """
        Modify the specified tool coordinate system.
        Required parameter:
        Parameter name     Type     Description
        Index    int     tool coordinate system index, range: [0,9]. The initial value of coordinate system 0 refers to the flange coordinate system.
        table    string     tool coordinate system after modification (format: {x, y, z, rx, ry, rz})
        """
        string = "SetTool({:d},{:s})".format(index, table)
        return self.sendRecvMsg(string)

    def CalcTool(self, index, matrix_direction, table):
        """
        Calculate the tool coordinate system.
        Required parameter:
        Parameter name     Type     Description
        Index    int     tool coordinate system index, range: [0,9]. The initial value of coordinate system 0 refers to the flange coordinate system.
        matrix_direction    int    Calculation method.
          1: left multiplication, indicating that the coordinate system specified by "index" deflects the value specified by "table" along the flange coordinate system.
          0: right multiplication, indicating that the coordinate system specified by "index" deflects the value specified by "table" along itself.
        table    string     tool coordinate system offset (format: {x, y, z, rx, ry, rz}).
        """
        string = "CalcTool({:d},{:d},{:s})".format(
            index, matrix_direction, table)
        return self.sendRecvMsg(string)

    def SetPayload(self, load=0.0, X=0.0, Y=0.0, Z=0.0, name='F'):

        Set the load of the robot arm.
        Method 1: Set the load parameters directly.
        Required parameter 1
        Parameter name     Type     Description
        load     double     Load weight. The value range should not exceed the load range of corresponding robot models. Unit: kg.
        Optional parameter 1
        Parameter name     Type     Description
        x     double     X-axis eccentric coordinates of the load. Range: -500 – 500. Unit: mm.
        y     double     Y-axis eccentric coordinates of the load. Range: -500 – 500. Unit: mm.
        z     double     Z-axis eccentric coordinates of the load. Range: -500 – 500. Unit: mm.
        The three parameters need to be set or not set at the same time. The eccentric coordinate is the coordinate of the center of mass of the load (including the fixture) under the default tool coordinate system.
        Refer to the figure below.

        Method 2: Set by the preset load parameter group saved by control software
        Required parameter 2
        Parameter name     Type     Description
        name     string     Name of the preset load parameter group saved by control software.
        '''
        string = 'SetPayload('
        if name != 'F':
            string = string + "{:s}".format(name)
        else:
            if load != 0:
                string = string + "{:f}".format(load)
                if X != 0 or Y != 0 or Z != 0:
                    string = string + ",{:f},{:f},{:f}".format(X, Y, Z)
        string = string + ')'
        return self.sendRecvMsg(string)

    def AccJ(self, speed):
        """
        Set acceleration ratio of joint motion.
        Defaults to 100 if not set.
        """
        string = "AccJ({:d})".format(speed)
        return self.sendRecvMsg(string)

    def AccL(self, speed):
        """
        Set acceleration ratio of linear and arc motion.
        Defaults to 100 if not set.
        """
        string = "AccL({:d})".format(speed)
        return self.sendRecvMsg(string)

    def VelJ(self, speed):
        """
        Set the speed ratio of joint motion.
        Defaults to 100 if not set.
        """
        string = "VelJ({:d})".format(speed)
        return self.sendRecvMsg(string)

    def VelL(self, speed):
        """
        Set the speed ratio of linear and arc motion.
        Defaults to 100 if not set.
        """
        string = "VelL({:d})".format(speed)
        return self.sendRecvMsg(string)

    def CP(self, ratio):
        """
        Set the continuous path (CP) ratio, that is, when the robot arm moves continuously via multiple points, whether it transitions at a right angle or in a curved way when passing through the through point.
        Defaults to 0 if not set.
        Continuous path ratio. Range: [0, 100].
        """
        string = "CP({:d})".format(ratio)
        return self.sendRecvMsg(string)

    def SetCollisionLevel(self, level):
        """
        Set the collision detection level.
        If it is not set, the value set by the software before entering TCP/IP control mode will be adopted.
        Required parameter:
        Parameter name     Type     Description
        level     int     collision detection level, 0: switch off collision detection, 1 – 5: the larger the number, the higher the sensitivity.
        """
        string = "SetCollisionLevel({:d})".format(level)
        return self.sendRecvMsg(string)

    def SetBackDistance(self, distance):
        """
        Set the backoff distance after the robot detects collision.
        If it is not set, the value set by the software before entering TCP/IP control mode will be adopted.
        Required parameter:
        Parameter name     Type     Description
        distance     double     collision backoff distance, range: [0,50], unit: mm.
        """
        string = "SetBackDistance({:d})".format(distance)
        return self.sendRecvMsg(string)

    def SetPostCollisionMode(self, mode):
        """
        Set the backoff distance after the robot detects collision.
        If it is not set, the value set by the software before entering TCP/IP control mode will be adopted.
        Required parameter:
        Parameter name     Type     Description
        mode     int     post-collision processing mode, 0: enter the stop status after the collision is detected, 1: enter the pause status after the collision is detected
        """
        string = "SetPostCollisionMode({:d})".format(mode)
        return self.sendRecvMsg(string)

    def StartDrag(self):
        """
        The robot arm enters the drag mode. The robot cannot enter the drag mode through this command in error status.
        """
        string = "StartDrag()"
        return self.sendRecvMsg(string)

    def StopDrag(self):
        """
        The robot arm enters the drag mode. The robot cannot enter the drag mode through this command in error status.
        """
        string = "StopDrag()"
        return self.sendRecvMsg(string)

    def DragSensivity(self, index, value):
        """
        Set the drag sensitivity.
        If it is not set, the value set by the software before entering TCP/IP control mode will be adopted.
        Required parameter:
        Parameter name     Type     Description
        index     int      axis ID, 1 – 6: J1 – J6, 0: set all axes at the same time.
        value     int     Drag sensitivity. The smaller the value, the greater the force when dragging. Range: [1, 90].
        """
        string = "DragSensivity({:d},{:d})".format(index, value)
        return self.sendRecvMsg(string)

    def EnableSafeSkin(self, status):
        """
        Switch on or off the SafeSkin. Valid only for robot arms equipped with SafeSkin.
        Required parameter:
        Parameter name     Type     Description
        status     int     SafeSkin switch, 0: off, 1: on.
        """
        string = "EnableSafeSkin({:d})".format(status)
        return self.sendRecvMsg(string)

    def SetSafeSkin(self, part, status):
        """
        Set the sensitivity for each part of the SafeSkin. Valid only for robot arms equipped with SafeSkin.
        If it is not set, the value set by the software before entering TCP/IP control mode will be adopted.
        Required parameter:
        Parameter name     Type     Description
        part     int     The part to be set. 3: forearm, 4 – 6: J4 – J6
        status     int     sensitivity, 0: off, 1: low, 2: middle, 3: high
        """
        string = "SetSafeSkin({:d},{:d})".format(part, status)
        return self.sendRecvMsg(string)

    def SetSafeWallEnable(self, index, value):
        """
        Switch on/off the specified safety wall.
        Required parameter:
        Parameter name     Type     Description
        index     int     safety wall index, which needs to be added in the software first. Range: [1.8].
        value      int     SafeSkin switch, 0: off, 1: on.
        """
        string = "SetSafeWallEnable({:d},{:d})".format(index, value)
        return self.sendRecvMsg(string)

    def SetWorkZoneEnable(self, index, value):
        """
        Switch on/off the specified interference area.
        Required parameter:
        Parameter name     Type     Description
        index     int     interference area index, which needs to be added in the software first. Range: [1.6].
        value      int     interference area switch, 0: off, 1: on.
        """
        string = "SetWorkZoneEnable({:d},{:d})".format(index, value)
        return self.sendRecvMsg(string)

    #########################################################################

    def RobotMode(self):
        """
        9 ROBOT_MODE_ERROR
        Get the current status of the robot.
        1 ROBOT_MODE_INIT  Initialized status
        2 ROBOT_MODE_BRAKE_OPEN  Brake switched on
        3 ROBOT_MODE_POWEROFF  Power-off status
        4 ROBOT_MODE_DISABLED  Disabled (no brake switched on
        5 ROBOT_MODE_ENABLE  Enabled and idle
        6 ROBOT_MODE_BACKDRIVE  Drag mode
        7 ROBOT_MODE_RUNNING  Running status (project, TCP queue)
        8 ROBOT_MODE_SINGLE_MOVE  Single motion status (jog, RunTo)
        9 ROBOT_MODE_ERROR
             There are uncleared alarms. This status has the highest priority. It returns 9 when there is an alarm, regardless of the status of the robot arm.
        10 ROBOT_MODE_PAUSE  Pause status
        11 ROBOT_MODE_COLLISION  Collision status
        """
        string = "RobotMode()"
        return self.sendRecvMsg(string)

    def PositiveKin(self, J1, J2, J3, J4, J5, J6, user=-1, tool=-1):
        """
       Description
       Positive solution. Calculate the coordinates of the end of the robot in the specified Cartesian coordinate system, based on the given angle of each joint.
       Required parameter:
       Parameter name     Type     Description
       J1     double     J1-axis position, unit: °
       J2     double     J2-axis position, unit: °
       J3     double     J3-axis position, unit: °
       J4     double     J4-axis position, unit: °
       J5     double     J5-axis position, unit: °
       J6     double     J6-axis position, unit: °
       Optional parameter:
       Parameter name     Type     Description
       Format: "user=index", index: index of the calibrated user coordinate system.
       User     string     The global user coordinate system will be used if it is not specified.
       Tool     string     Format: "tool=index", index: index of the calibrated tool coordinate system. The global tool coordinate system will be used if it is not set.
        """
        string = "PositiveKin({:f},{:f},{:f},{:f},{:f},{:f}".format(
            J1, J2, J3, J4, J5, J6)
        params = []
        if user != -1:
            params.append('user={:d}'.format(user))
        if tool != -1:
            params.append('tool={:d}'.format(tool))
        for ii in params:
            string = string + ','+ii
        string = string + ')'
        return self.sendRecvMsg(string)

    def InverseKin(self, X, Y, Z, Rx, Ry, Rz, user=-1, tool=-1, useJointNear=-1, JointNear=''):
        """
        Description
        Inverse solution. Calculate the joint angles of the robot, based on the given coordinates in the specified Cartesian coordinate system.
        As Cartesian coordinates only define the spatial coordinates and tilt angle of the TCP, the robot arm can reach the same posture through different gestures, which means that one posture variable can correspond to multiple joint variables.
        To get a unique solution, the system requires a specified joint coordinate, and the solution closest to this joint coordinate is selected as the inverse solution。
        Required parameter:
        Parameter name     Type     Description
        X     double     X-axis position, unit: mm
        Y     double     Y-axis position, unit: mm
        Z     double     Z-axis position, unit: mm
        Rx     double     Rx-axis position, unit: °
        Ry     double     Ry-axis position, unit: °
        Rz     double     Rz-axis position, unit: °
        Optional parameter:
        Parameter name     Type     Description
        User      string     Format: "user=index", index: index of the calibrated user coordinate system. The global user coordinate system will be used if it is not set.
        Tool     string     Format: "tool=index", index: index of the calibrated tool coordinate system. The global tool coordinate system will be used if it is not set.
        useJointNear     string     used to set whether JointNear is effective.
            "useJointNear=0" or null: JointNear data is ineffective. The algorithm selects the joint angles according to the current angle.
            "useJointNear=1": the algorithm selects the joint angles according to JointNear data.
        jointNear     string     Format: "jointNear={j1,j2,j3,j4,j5,j6}", joint coordinates for selecting joint angles.
        """
        string = "InverseKin({:f},{:f},{:f},{:f},{:f},{:f}".format(
            X, Y, Z, Rx, Ry, Rz)
        params = []
        if user != -1:
            params.append('user={:d}'.format(user))
        if tool != -1:
            params.append('tool={:d}'.format(tool))
        if useJointNear != -1:
            params.append('useJointNear={:d}'.format(useJointNear))
        if JointNear != '':
            params.append('JointNear={:s}'.format(JointNear))
        for ii in params:
            string = string + ','+ii
        string = string + ')'
        return self.sendRecvMsg(string)

    def GetAngle(self):
        """
        Get the joint coordinates of current posture.
        """
        string = "GetAngle()"
        return self.sendRecvMsg(string)

    def GetPose(self, user=-1, tool=-1):
        """
        Get the Cartesian coordinates of the current posture under the specific coordinate system.
        Optional parameter:
        Parameter name     Type     Description
        User      string     Format: "user=index", index: index of the calibrated user coordinate system.
        Tool     string     Format: "tool=index", index: index of the calibrated tool coordinate system.
        They need to be set or not set at the same time. They are global user coordinate system and global tool coordinate system if not set.
        """
        string = "GetPose("
        params = []
        state = True
        if user != -1:
            params.append('user={:d}'.format(user))
            state = not state
        if tool != -1:
            params.append('tool={:d}'.format(tool))
            state = not state
        if not state:
            return 'need to be set or not set at the same time. They are global user coordinate system and global tool coordinate system if not set'

        for i, param in enumerate(params):
            if i == len(params)-1:
                string = string + param
            else:
                string = string + param+","

        string = string + ')'
        return self.sendRecvMsg(string)

    def GetErrorID(self):
        """
        Get the joint coordinates of current posture.
        """
        string = "GetErrorID()"
        return self.sendRecvMsg(string)

     #################################################################

    def DO(self, index, status, time=-1):
        """
        Set the status of digital output port (queue command).
        Required parameter:
        Parameter name     Type     Description
        index     int     DO index
        status     int     DO index, 1: ON, 0: OFF
        Optional parameter:
        Parameter name     Type     Description
        time     int     continuous output time, range: [25,60000]. Unit: ms.
        If this parameter is set, the system will automatically invert the DO after the specified time.
        The inversion is an asynchronous action, which will not block the command queue. After the DO output is executed, the system will execute the next command.
        """
        string = "DO({:d},{:d}".format(index, status)
        params = []
        if time != -1:
            params.append('{:d}'.format(time))
        for ii in params:
            string = string + ',' + ii
        string = string + ')'
        return self.sendRecvMsg(string)

    def DOInstant(self, index, status):
        """
        Set the status of digital output port (immediate command).
        Required parameter:
        Parameter name     Type     Description
        index     int     DO index
        status     int     DO index, 1: ON, 0: OFF
        """
        string = "DOInstant({:d},{:d})".format(index, status)
        return self.sendRecvMsg(string)

    def GetDO(self, index):
        """
        Get the status of digital output port.
        Required parameter:
        Parameter name     Type     Description
        index     int     DO index
        """
        string = "GetDO({:d})".format(index)
        return self.sendRecvMsg(string)

    def DOGroup(self, *index_value):
        """
        ... ... ...
        ErrorID,{ResultID},DOGroup(index1,value1,index2,value2,...,indexN,valueN);
        DOGroup(4,1,6,0,2,1,7,0)
        Set the status of multiple digital output ports (queue command).
        Required parameter:
        Parameter name     Type     Description
        index1     int     index of the first DO
        value1      int     status of the first DO, 1: ON, 0: OFF
        ... ... ...
        indexN     int     index of the last DO
        valueN      int     status of the last DO, 1: ON, 0: OFF
        Return
        ErrorID,{ResultID},DOGroup(index1,value1,index2,value2,...,indexN,valueN);
        ResultID is algorithm queue ID, used to judge the order in which commands are executed.
        Example
        DOGroup(4,1,6,0,2,1,7,0)
        Set DO_4 to ON, DO_6 to OFF, DO_2 to ON, DO_7 to OFF.
        """
        string = "DOGroup({:d}".format(index_value[0])
        for ii in index_value[1:]:
            string = string + ',' + str(ii)
        string = string + ')'

        return self.sendRecvMsg(string)

    def GetDOGroup(self, *index_value):
        """
        ... ... ...
        ErrorID,{value1,value2,...,valueN},GetDOGroup(index1,index2,...,indexN);
        GetDOGroup(1,2)
        Get the status of multiple digital output ports.
        Required parameter:
        Parameter name     Type     Description
        index     int     index of the first DO
        ... ... ...
        indexN     int     index of the last DO
        Return
        ErrorID,{value1,value2,...,valueN},GetDOGroup(index1,index2,...,indexN);
        {value1,value2,...,valueN}: status of DO_1 – DO_N. 0: OFF, 1: ON.
        Example
        GetDOGroup(1,2)
        Get the status of DO_1 and DO_2.
        """
        string = "GetDOGroup({:d}".format(index_value[0])
        for ii in index_value[1:]:
            string = string + ',' + str(ii)
        string = string + ')'
        return self.sendRecvMsg(string)

    def ToolDO(self, index, status):
        """
        Set the status of tool digital output port (queue command).
        Required parameter:
        Parameter name     Type     Description
        index     int     index of the tool DO
        status     int     status of the tool DO, 1: ON, 0: OFF
        """
        string = "ToolDO({:d},{:d})".format(index, status)
        return self.sendRecvMsg(string)

    def ToolDOInstant(self, index, status):
        """
        Set the status of tool digital output port (immediate command)
        Required parameter:
        Parameter name     Type     Description
        index     int     index of the tool DO
        status     int     status of the tool DO, 1: ON, 0: OFF
        """
        string = "ToolDOInstant({:d},{:d})".format(index, status)
        return self.sendRecvMsg(string)

    def GetToolDO(self, index):
        """
        Set the status of tool digital output port (immediate command)
        Required parameter:
        Parameter name     Type     Description
        index     int     index of the tool DO
        status     int     status of the tool DO, 1: ON, 0: OFF
        """
        string = "GetToolDO({:d})".format(index)
        return self.sendRecvMsg(string)

    def AO(self, index, value):
        """
        Set the value of analog output port (queue command).
        Required parameter:
        Parameter name     Type     Description
        index     int     AO index
        value     double     AO output, voltage range: [0,10], unit: V; current range: [4,20], unit: mA
        """
        string = "AO({:d},{:f})".format(index, value)
        return self.sendRecvMsg(string)

    def AOInstant(self, index, value):
        """
        Set the value of analog output port (immediate command).
        Required parameter:
        Parameter name     Type     Description
        index     int     AO index
        value     double     AO output, voltage range: [0,10], unit: V; current range:
        [4,20], unit: mA
        """
        string = "AOInstant({:d},{:f})".format(index, value)
        return self.sendRecvMsg(string)

    def GetAO(self, index):
        """
        Get the value of analog output port.
        Required parameter:
        Parameter name     Type     Description
        index     int     AO index
        """
        string = "GetAO({:d})".format(index)
        return self.sendRecvMsg(string)

    def DI(self, index):
        """
        Get status of DI port.
        Required parameter:
        Parameter name     Type     Description
        index     int     DI index
        """
        string = "DI({:d})".format(index)
        return self.sendRecvMsg(string)

    def DIGroup(self, *index_value):
        """
        ... ... ...
        ErrorID,{value1,value2,...,valueN},DIGroup(index1,index2,...,indexN);
        DIGroup(4,6,2,7)
        Get status of multiple DI ports.
        Required parameter:
        Parameter name     Type     Description
        index1     int     index of the first DI
        ... ... ...
        indexN     int     index of the last DI
        Return
        ErrorID,{value1,value2,...,valueN},DIGroup(index1,index2,...,indexN);
        {value1,value2,...,valueN}: status of DI_1 – DI_N. 0: OFF, 1: ON.
        Example
        DIGroup(4,6,2,7)
        Get the status of DI_4, DI_6, DI_2 and DI_7.
        """
        string = "DIGroup({:d}".format(index_value[0])
        for ii in index_value[1:]:
            string = string + ',' + str(ii)
        string = string + ')'
        return self.sendRecvMsg(string)

    def ToolDI(self, index):
        """
        Get the status of tool digital input port.
        Required parameter:
        Parameter name     Type     Description
        index     int     index of the tool DI
        """
        string = "ToolDI({:d})".format(index)
        return self.sendRecvMsg(string)

    def AI(self, index):
        """
        Get the value of analog input port.
        Required parameter:
        Parameter name     Type     Description
        index     int     AI index
        """
        string = "AI({:d})".format(index)
        return self.sendRecvMsg(string)

    def ToolAI(self, index):
        """
        Get the value of tool analog input port. You need to set the port to analog-input mode through SetToolMode before use.
        Required parameter:
        Parameter name     Type     Description
        index     int     index of the tool AI
        """
        string = "ToolAI({:d})".format(index)
        return self.sendRecvMsg(string)

    def SetTool485(self, index, parity='', stopbit=-1, identify=-1):
        """
        parity string
        ErrorID,{},SetTool485(baud,parity,stopbit);
        SetTool485(115200,"N",1)
        Description:
        Set the data type corresponding to the RS485 interface of the end tool.
        Required parameter:
        Parameter name     Type     Description
        baud     int     baud rate of RS485 interface
        Optional parameter:
        Parameter name     Type     Description
        parity string     Whether there are parity bits. "O" means odd, "E" means even, and "N" means no parity bit. "N" by default.
        stopbit     int     stop bit length Range: 1, 2 1 by default.
        identify     int     If the robot is equipped with multiple aviation sockets, you need to specify them. 1: aviation 1; 2: aviation 2
        Return
        ErrorID,{},SetTool485(baud,parity,stopbit);
        Example
        SetTool485(115200,"N",1)
        Set the baud rate corresponding to the tool RS485 interface to 115200Hz, parity bit to N, and stop bit length to 1.
        """
        string = "SetTool485({:d}".format(index)
        params = []
        if parity != '':
            params.append(parity)
        if string != -1:
            params.append('{:d}'.format(stopbit))
            if identify != -1:
                params.append('{:d}'.format(identify))
        else:
            if identify != -1:
                params.append('1,{:d}'.format(identify))
        for ii in params:
            string = string + ','+ii
        string = string + ')'
        return self.sendRecvMsg(string)

    def SetToolPower(self, status, identify=-1):
        """
        ErrorID,{},SetToolPower(status);
        SetToolPower(0)
        Set the power status of the end tool, generally used for restarting the end power, such as re-powering and re-initializing the gripper.
        If you need to call the interface continuously, it is recommended to keep an interval of at least 4 ms.
        NOTE:
        This command is not supported on Magician E6 robot, and there is no effect when calling it.
        Required parameter:
        Parameter name     Type     Description
        status    int     power status of end tool. 0: power off; 1: power on.
        Optional parameter:
        Parameter name     Type     Description
        identify     int     If the robot is equipped with multiple aviation sockets, you need to specify them. 1: aviation 1; 2: aviation 2
        Return
        ErrorID,{},SetToolPower(status);
        Example
        SetToolPower(0)
        Power off the tool.
        """
        string = "SetToolPower({:d}".format(status)
        params = []
        if identify != -1:
            params.append('{:d}'.format(identify))
        for ii in params:
            string = string + ','+ii
        string = string + ')'
        return self.sendRecvMsg(string)

    def SetToolMode(self, mode, type, identify=-1):
        """
        ErrorID,{},SetToolMode(mode,type);
        SetToolMode(2,0)
        Description:
        If the AI interface on the end of the robot arm is multiplexed with the 485 interface, you can set the mode of the end multiplex terminal via this interface.
        485 mode by default.
        NOTE:
        The robot arm without tool RS485 interface has no effect when calling this interface.
        Required parameter:
        Parameter name     Type     Description
        mode     int     mode of the multiplex terminal, 1: 485 mode, 2: AI mode
        type     int     When mode is 1, this parameter is invalid. When mode is 2, you can set the mode of AI.
                  The single digit indicates the mode of AI1, the tens digit indicates the mode of AI2. When the tens digit is 0, you can enter only the single digit.
        Mode:
        0: 0 – 10V voltage input mode
        1: Current collection mode
        2: 0 – 5V voltage input mode
        Example:
        0: AI1 and AI2 are 0 – 10V voltage input mode
        1: AI2 is 0 – 10V voltage input mode, AI1 is current collection mode
        11: AI2 and AI1 are current collection mode
        12: AI2 is current collection mode, AI1 is 0 – 5V voltage input mode
        20: AI2 is 0 – 5V voltage input mode, AI1 is 0 – 10V voltage input mode
        Optional parameter:
        Parameter name     Type     Description
        identify     int     If the robot is equipped with multiple aviation sockets, you need to specify them. 1: aviation 1; 2: aviation 2
        Return
        ErrorID,{},SetToolMode(mode,type);
        Example
        SetToolMode(2,0)
        Set the mode of the end multiplex terminal to AI, both are 0 – 10V voltage input mode.
        """
        string = "SetToolMode({:d},{:d}".format(mode, type)
        params = []
        if identify != -1:
            params.append('{:d}'.format(identify))
        for ii in params:
            string = string + ','+ii
        string = string + ')'
        return self.sendRecvMsg(string)

     ##################################################################

    def ModbusCreate(self, ip, port, slave_id, isRTU=-1):
        """
        Create Modbus master, and establish connection with the slave. (support connecting to at most 5 devices).
        Required parameter:
        Parameter name     Type     Description
        ip     string     slave IP address
        port     int     slave port
        slave_id     int     slave ID
        Optional parameter:
        Parameter name     Type     Description
        isRTU     int     null or 0: establish ModbusTCP communication; 1: establish ModbusRTU communication
        """
        string = "ModbusCreate({:s},{:d},{:d}".format(ip, port, slave_id)
        params = []
        if isRTU != -1:
            params.append('{:d}'.format(isRTU))
        for ii in params:
            string = string + ',' + ii
        string = string + ')'
        return self.sendRecvMsg(string)

    def ModbusRTUCreate(self, slave_id, baud, parity='', data_bit=8, stop_bit=-1):
        """
        parity string
        Create Modbus master station based on RS485, and establish connection with slave station (support connecting to at most 5 devices).
        Required parameter:
        Parameter name     Type     Description
        slave_id     int     slave ID
        baud     int     baud rate of RS485 interface.
        Optional parameter:
        Parameter name     Type     Description
        parity string     Whether there are parity bits. "O" means odd, "E" means even, and "N" means no parity bit. "E" by default.
        data_bit     int     data bit length Range: 8 (8 by default).
        stop_bit     int     stop bit length Range: 1, 2 (1 by default).
        """
        string = "ModbusRTUCreate({:d},{:d}".format(slave_id, baud)
        params = []
        if parity != '':
            params.append('{:s}'.format(parity))
        if data_bit != 8:
            params.append('{:d}'.format(data_bit))
        if stop_bit != -1:
            params.append('{:d}'.format(stop_bit))
        for ii in params:
            string = string + ',' + ii
        string = string + ')'
        return self.sendRecvMsg(string)

    def ModbusClose(self, index):
        """
        Disconnect with Modbus slave and release the master.
        Required parameter:
        Parameter name     Type     Description
        index     int     master index
        """
        string = "ModbusClose({:d})".format(index)
        return self.sendRecvMsg(string)

    def GetInBits(self, index, addr, count):
        """
        Read the contact register (discrete input) value from the Modbus slave.
        Required parameter:
        Parameter name     Type     Description
        index     int     master index
        addr     int     starting address of the contact register
        count     int     number of contact registers Range: [1, 16].
        """
        string = "GetInBits({:d},{:d},{:d})".format(index, addr, count)
        return self.sendRecvMsg(string)

    def GetInRegs(self, index, addr, count, valType=''):
        """
        valType string
        Read the input register value with the specified data type from the Modbus slave.
        Required parameter:
        Parameter name     Type     Description
        index     int     master index
        addr     int     starting address of the input register
        count     int     number of input registers Range: [1, 4].
        Optional parameter:
        Parameter name     Type     Description
        valType string
        Data type:
        U16: 16-bit unsigned integer (two bytes, occupy one register)
        U32: 32-bit unsigned integer (four bytes, occupy two register).
        F32: 32-bit single-precision floating-point number (four bytes, occupy two registers)
        F64: 64-bit double-precision floating-point number (eight bytes, occupy four registers)
        U16 by default.
        """
        string = "GetInRegs({:d},{:d},{:d}".format(index, addr, count)
        params = []
        if valType != '':
            params.append('{:s}'.format(valType))
        for ii in params:
            string = string + ',' + ii
        string = string + ')'
        return self.sendRecvMsg(string)

    def GetCoils(self, index, addr, count):
        """
        Read the coil register value from the Modbus slave.
        Required parameter:
        Parameter name     Type     Description
        index     int     master index
        addr     int     starting address of the coil register
        count     int     number of coil registers Range: [1, 16].
        """
        string = "GetCoils({:d},{:d},{:d})".format(index, addr, count)
        return self.sendRecvMsg(string)

    def SetCoils(self, index, addr, count, valTab):
        """
        ErrorID,{},SetCoils(index,addr,count,valTab);
        SetCoils(0,1000,3,{1,0,1})
        Description
        Write the specified value to the specified address of coil register.
        Required parameter:
        Parameter name     Type     Description
        index     int     master index
        addr     int     starting address of the coil register
        count     int     number of values to be written to the coil register. Range: [1, 16].
        valTab     string     values to be written to the register (number of values equals to count)
        Return
        ErrorID,{},SetCoils(index,addr,count,valTab);
        Example
        SetCoils(0,1000,3,{1,0,1})
        Write three values (1 , 0, 1) to the coil register starting from address 1000.
        """
        string = "SetCoils({:d},{:d},{:d},{:s})".format(
            index, addr, count, valTab)
        return self.sendRecvMsg(string)

    def GetHoldRegs(self, index, addr, count, valType=''):
        """
        valType string
        Write the specified value according to the specified data type to the specified address of holding register.
        Required parameter:
        Parameter name     Type     Description
        index     int     master index
        addr     int     starting address of the holding register
        count     int     number of values to be written to the holding register. Range: [1, 4].
        Optional parameter:
        Parameter name     Type     Description
        valType string
        Data type:
        U16: 16-bit unsigned integer (two bytes, occupy one register)
        U32: 32-bit unsigned integer (four bytes, occupy two register)
        F32: 32-bit single-precision floating-point number (four bytes, occupy two registers)
        F64: 64-bit double-precision floating-point number (eight bytes, occupy four registers)
        U16 by default.
        """
        string = "GetHoldRegs({:d},{:d},{:d}".format(index, addr, count)
        params = []
        if valType != '':
            params.append('{:s}'.format(valType))
        for ii in params:
            string = string + ',' + ii
        string = string + ')'
        return self.sendRecvMsg(string)

    def SetHoldRegs(self, index, addr, count, valTab, valType=''):
        """
        valType string
        Write the specified value with specified data type to the specified address of holding register.
        Required parameter:
        Parameter name     Type     Description
        index     int     master index
        addr     int     starting address of the holding register
        count     int     number of values to be written to the holding register. Range: [1, 4].
        valTab     string     values to be written to the register (number of values equals to count).
        Optional parameter:
        Parameter name     Type     Description
        valType string
        Data type:
        U16: 16-bit unsigned integer (two bytes, occupy one register)
        U32: 32-bit unsigned integer (four bytes, occupy two register)
        F32: 32-bit single-precision floating-point number (four bytes, occupy two registers)
        F64: 64-bit double-precision floating-point number (eight bytes, occupy four registers)
        U16 by default.
        """
        string = "SetHoldRegs({:d},{:d},{:d},{:s}".format(
            index, addr, count, valTab)
        params = []
        if valType != '':
            params.append('{:s}'.format(valType))
        for ii in params:
            string = string + ',' + ii
        string = string + ')'
        return self.sendRecvMsg(string)
    ########################################################################

    def GetInputBool(self, address):
        """
        Get the value in bool type from the specified address of input register.
        Required parameter:
        Parameter name     Type     Description
        address     int     register address, range: [0-63]
        """
        string = "GetInputBool({:d})".format(address)
        return self.sendRecvMsg(string)

    def GetInputInt(self, address):
        """
        Get the value in int type from the specified address of input register.
        Required parameter:
        Parameter name     Type     Description
        address     int     register address, range: [0-23]
        """
        string = "GetInputInt({:d})".format(address)
        return self.sendRecvMsg(string)

    def GetInputFloat(self, address):
        """
        Get the value in float type from the specified address of input register.
        Required parameter:
        Parameter name     Type     Description
        address     int     register address, range: [0-23]
        """
        string = "GetInputFloat({:d})".format(address)
        return self.sendRecvMsg(string)

    def GetOutputBool(self, address):
        """
        Get the value in bool type from the specified address of output register.
        Required parameter:
        Parameter name     Type     Description
        address     int     register address, range: [0-63]
        """
        string = "GetOutputBool({:d})".format(address)
        return self.sendRecvMsg(string)

    def GetOutputInt(self, address):
        """
        Get the value in int type from the specified address of output register.
        Required parameter:
        Parameter name     Type     Description
        address     int     register address, range: [0-23]
        """
        string = "GetOutputInt({:d})".format(address)
        return self.sendRecvMsg(string)

    def GetOutputFloat(self, address):
        """
        Get the value in float type from the specified address of output register.
        Required parameter:
        Parameter name     Type     Description
        address     int     register address, range: [0-23]
        """
        string = "GetInputFloat({:d})".format(address)
        return self.sendRecvMsg(string)

    def SetOutputBool(self, address, value):
        """
        Set the value in bool type at the specified address of output register.
        Required parameter:
        Parameter name     Type     Description
        address     int     register address, range: [0-63]
        value     int     value to be set (0 or 1)
        """
        string = "GetInputFloat({:d},{:d})".format(address, value)
        return self.sendRecvMsg(string)

    def SetOutputInt(self, address, value):
        """
        Set the value in int type at the specified address of output register.
        Required parameter:
        Parameter name     Type     Description
        address     int     register address, range: [0-23]
        value     int     value to be set (integer)
        """
        string = "SetOutputInt({:d},{:d})".format(address, value)
        return self.sendRecvMsg(string)

    def SetOutputFloat(self, address, value):
        """
        Set the value in int type at the specified address of output register.
        Required parameter:
        Parameter name     Type     Description
        address     int     register address, range: [0-23]
        value     int     value to be set (integer)
        """
        string = "SetOutputFloat({:d},{:d})".format(address, value)
        return self.sendRecvMsg(string)

    #######################################################################

    def MovJ(self, a1, b1, c1, d1, e1, f1, coordinateMode, user=-1, tool=-1, a=-1, v=-1, cp=-1):
        """
        Description
        Move from the current position to the target position through joint motion.
        Required parameter:
        Parameter name     Type     Description
        P     string     Target point (joint variables or posture variables)
        coordinateMode     int      Coordinate mode of the target point, 0: pose, 1: joint
        Optional parameter:
        Parameter name     Type     Description
        user     int     user coordinate system
        tool     int     tool coordinate system
        a     int     acceleration rate of the robot arm when executing this command. Range: (0,100].
        v     int     velocity rate of the robot arm when executing this command. Range: (0,100].
        cp     int     continuous path rate. Range: [0,100].
        """
        string = ""
        if coordinateMode == 0:
            string = "MovJ(pose={{{:f},{:f},{:f},{:f},{:f},{:f}}}".format(
                a1, b1, c1, d1, e1, f1)
        elif coordinateMode == 1:
            string = "MovJ(joint={{{:f},{:f},{:f},{:f},{:f},{:f}}}".format(
                a1, b1, c1, d1, e1, f1)
        else:
            print("coordinateMode param is wrong")
            return ""
        params = []
        if user != -1:
            params.append('user={:d}'.format(user))
        if tool != -1:
            params.append('tool={:d}'.format(tool))
        if a != -1:
            params.append('a={:d}'.format(a))
        if v != -1:
            params.append('v={:d}'.format(v))
        if cp != -1:
            params.append('cp={:d}'.format(cp))
        for ii in params:
            string = string + ','+ii
        string = string + ')'
        return self.sendRecvMsg(string)

    def MovL(self, a1, b1, c1, d1, e1, f1, coordinateMode, user=-1, tool=-1, a=-1, v=-1, speed=-1, cp=-1, r=-1):
        """
        Description
        Move from the current position to the target position in a linear mode.
        Required parameter:
        Parameter name     Type     Description
        P     string     Target point (joint variables or posture variables)
        coordinateMode     int      Coordinate mode of the target point, 0: pose, 1: joint
        Optional parameter:
        Parameter name     Type     Description
        user     int     user coordinate system
        tool     int     tool coordinate system
        a     int     acceleration rate of the robot arm when executing this command. Range: (0,100].
        v     int     velocity rate of the robot arm when executing this command, incompatible with “speed”. Range: (0,100].
        speed     int     target speed of the robot arm when executing this command, incompatible with “v”. If both "speed" and "v” exist, speed takes precedence. Range: [0, maximum motion speed], unit: mm/s.
        cp     int     continuous path rate, incompatible with “r”. Range: [0,100].
        r     int     continuous path radius, incompatible with “cp”. If both "r" and "cp” exist, r takes precedence. Unit: mm.
        """
        string = ""
        if coordinateMode == 0:
            string = "MovL(pose={{{:f},{:f},{:f},{:f},{:f},{:f}}}".format(
                a1, b1, c1, d1, e1, f1)
        elif coordinateMode == 1:
            string = "MovL(joint={{{:f},{:f},{:f},{:f},{:f},{:f}}}".format(
                a1, b1, c1, d1, e1, f1)
        else:
            print("coordinateMode  param  is wrong")
            return ""
        params = []
        if user != -1:
            params.append('user={:d}'.format(user))
        if tool != -1:
            params.append('tool={:d}'.format(tool))
        if a != -1:
            params.append('a={:d}'.format(a))
        if v != -1 and speed != -1:
            params.append('speed={:d}'.format(speed))
        elif speed != -1:
            params.append('speed={:d}'.format(speed))
        elif v != -1:
            params.append('v={:d}'.format(v))
        if cp != -1 and r != -1:
            params.append('r={:d}'.format(r))
        elif r != -1:
            params.append('r={:d}'.format(r))
        elif cp != -1:
            params.append('cp={:d}'.format(cp))
        for ii in params:
            string = string + ',' + ii
        string = string + ')'
        return self.sendRecvMsg(string)

    def ServoJ(self, J1, J2, J3, J4, J5, J6, t=-1.0,aheadtime=-1.0, gain=-1.0):
        """
        Joint string Target point joint variables
        t float Optional parameter.Running time of the point, unit: s, value range: [0.02,3600.0], default value:0.1
        aheadtime float Optional parameter.Advanced time, similar to the D in PID control. Scalar, no unit, valuerange: [20.0,100.0], default value: 50.
        gain float Optional parameter.Proportional gain of the target position, similar to the P in PID control.Scalar, no unit, value range: [200.0,1000.0], default value: 500.
        """
        string = ""
        string = "ServoJ({:f},{:f},{:f},{:f},{:f},{:f}".format(J1, J2, J3, J4, J5, J6)
        params = []
        if t != -1:
            params.append('t={:f}'.format(t))
        if aheadtime != -1:
            params.append('aheadtime={:f}'.format(aheadtime))
        if gain != -1:
            params.append('gain={:f}'.format(gain))
        for ii in params:
            string = string + ','+ii
        string = string + ')'
        return self.sendRecvMsg(string)
    def ServoP(self, X, Y, Z, RX, RY, RZ, t=-1.0,aheadtime=-1.0, gain=-1.0):
        """
        Pose string  Target point posture variables. The reference coordinate system is the global user and tool coordinate system, see the User and Tool command descriptions in Settings command (the default values are both 0
        t float Optional parameter.Running time of the point, unit: s, value range: [0.02,3600.0], default value:0.1
        aheadtime float Optional parameter.Advanced time, similar to the D in PID control. Scalar, no unit, valuerange: [20.0,100.0], default value: 50.
        gain float Optional parameter.Proportional gain of the target position, similar to the P in PID control.Scalar, no unit, value range: [200.0,1000.0], default value: 500.
        """
        string = ""
        string = "ServoP({:f},{:f},{:f},{:f},{:f},{:f}".format(X, Y, Z, RX, RY, RZ)
        params = []
        if t != -1:
            params.append('t={:f}'.format(t))
        if aheadtime != -1:
            params.append('aheadtime={:f}'.format(aheadtime))
        if gain != -1:
            params.append('gain={:f}'.format(gain))
        for ii in params:
            string = string + ','+ii
        string = string + ')'
        return self.sendRecvMsg(string)

    def MovLIO(self, a1, b1, c1, d1, e1, f1, coordinateMode, Mode, Distance, Index, Status, user=-1, tool=-1, a=-1, v=-1, speed=-1, cp=-1, r=-1):
        """
        Description
        Move from the current position to the target position in a linear mode, and set the status of digital output port when the robot is moving.
        Required parameter:
        Parameter name     Type     Description
        P     string     Target point (joint variables or posture variables)
        coordinateMode     int      Coordinate mode of the target point, 0: pose, 1: joint
        {Mode,Distance,Index,Status}: digital output parameters, used to set the specified DO to be triggered when the robot arm moves to a specified distance or percentage. Multiple groups of parameters can be set.
        Parameter name     Type     Description
        Mode     int     Trigger mode. 0: distance percentage, 1: distance value.
        Distance     int     Specified distance.
        If Distance is positive, it refers to the distance away from the starting point;
        If Distance is negative, it refers to the distance away from the target point;
        If Mode is 0, Distance refers to the percentage of total distance. Range: (0,100];
        If Mode is 1, Distance refers to the distance value. Unit: mm.
        Index     int     DO index
        Status     int     DO status. 0: no signal, 1: have signal.
        Optional parameter:
        Parameter name     Type     Description
        user     int     user coordinate system
        tool     int     tool coordinate system
        a     int     acceleration rate of the robot arm when executing this command. Range: (0,100].
        v     int     velocity rate of the robot arm when executing this command, incompatible with “speed”. Range: (0,100].
        speed     int     target speed of the robot arm when executing this command, incompatible with “v”. If both "speed" and "v” exist, speed takes precedence. Range: [0, maximum motion speed], unit: mm/s.
        cp     int     continuous path rate, incompatible with “r”. Range: [0,100].
        r     int     continuous path radius, incompatible with “cp”. If both "r" and "cp” exist, r takes precedence. Unit: mm.
        """
        string = ""
        if coordinateMode == 0:
            string = "MovLIO(pose={{{:f},{:f},{:f},{:f},{:f},{:f}}},{{{:d},{:d},{:d},{:d}}}".format(
                a1, b1, c1, d1, e1, f1, Mode, Distance, Index, Status)
        elif coordinateMode == 1:
            string = "MovLIO(joint={{{:f},{:f},{:f},{:f},{:f},{:f}}},{{{:d},{:d},{:d},{:d}}}".format(
                a1, b1, c1, d1, e1, f1, Mode, Distance, Index, Status)
        else:
            print("coordinateMode  param  is wrong")
            return ""
        params = []
        if user != -1:
            params.append('user={:d}'.format(user))
        if tool != -1:
            params.append('tool={:d}'.format(tool))
        if a != -1:
            params.append('a={:d}'.format(a))
        if v != -1 and speed != -1:
            params.append('speed={:d}'.format(speed))
        elif speed != -1:
            params.append('speed={:d}'.format(speed))
        elif v != -1:
            params.append('v={:d}'.format(v))
        if cp != -1 and r != -1:
            params.append('r={:d}'.format(r))
        elif r != -1:
            params.append('r={:d}'.format(r))
        elif cp != -1:
            params.append('cp={:d}'.format(cp))
        for ii in params:
            string = string + ',' + ii
        string = string + ')'
        return self.sendRecvMsg(string)

    def MovJIO(self,  a1, b1, c1, d1, e1, f1, coordinateMode, Mode, Distance, Index, Status, user=-1, tool=-1, a=-1, v=-1, cp=-1,):
        """
        Description
        Move from the current position to the target position through joint motion, and set the status of digital output port when the robot is moving.
        Required parameter:
        Parameter name     Type     Description
        P     string     Target point (joint variables or posture variables)
        coordinateMode     int      Coordinate mode of the target point, 0: pose, 1: joint
        {Mode,Distance,Index,Status}: digital output parameters, used to set the specified DO to be triggered when the robot arm moves to a specified distance or percentage. Multiple groups of parameters can be set.
        Parameter name     Type     Description
        Mode     int     Trigger mode. 0: distance percentage, 1: distance value.
        The system will synthesise the joint angles into an angular vector and calculate the angular difference between the end point and the start point as the total distance of the motion.
        Distance     int     Specified distance.
        If Distance is positive, it refers to the distance away from the starting point;
        If Distance is negative, it refers to the distance away from the target point;
        If Mode is 0, Distance refers to the percentage of total distance. Range: (0,100];
        If Mode is 1, Distance refers to the distance value. Unit: °.
        Index     int     DO index
        Status     int     DO status. 0: no signal, 1: have signal.
        Optional parameter:
        Parameter name     Type     Description
        user     int     user coordinate system
        tool     int     tool coordinate system
        a     int     acceleration rate of the robot arm when executing this command. Range: (0,100].
        v     int     velocity rate of the robot arm when executing this command. Range: (0,100].
        cp     int     continuous path rate. Range: [0,100].
        """
        string = ""
        if coordinateMode == 0:
            string = "MovJIO(pose={{{:f},{:f},{:f},{:f},{:f},{:f}}},{{{:d},{:d},{:d},{:d}}}".format(
                a1, b1, c1, d1, e1, f1, Mode, Distance, Index, Status)
        elif coordinateMode == 1:
            string = "MovJIO(joint={{{:f},{:f},{:f},{:f},{:f},{:f}}},{{{:d},{:d},{:d},{:d}}}".format(
                a1, b1, c1, d1, e1, f1, Mode, Distance, Index, Status)
        else:
            print("coordinateMode  param  is wrong")
            return ""
        params = []
        if user != -1:
            params.append('user={:d}'.format(user))
        if tool != -1:
            params.append('tool={:d}'.format(tool))
        if a != -1:
            params.append('a={:d}'.format(a))
        if v != -1:
            params.append('v={:d}'.format(v))
        if cp != -1:
            params.append('cp={:d}'.format(cp))
        for ii in params:
            string = string + ',' + ii
        string = string + ')'
        return self.sendRecvMsg(string)

    def Arc(self, a1, b1, c1, d1, e1, f1,  a2, b2, c2, d2, e2, f2, coordinateMode, user=-1, tool=-1, a=-1, v=-1, speed=-1, cp=-1, r=-1):
        """
        Description
        Move from the current position to the target position in an arc interpolated mode.
        As the arc needs to be determined through the current position, through point and target point, the current position should not be in a straight line determined by P1 and P2.
        Required parameter:
        Parameter name     Type     Description
        P1     string     Through point (joint variables or posture variables)
        P2     string     Target point (joint variables or posture variables)
        coordinateMode     int      Coordinate mode of the target point, 0: pose, 1: joint
        Optional parameter:
        Parameter name     Type     Description
        user     int     user coordinate system
        tool     int     tool coordinate system
        a     int     acceleration rate of the robot arm when executing this command. Range: (0,100].
        v     int     velocity rate of the robot arm when executing this command, incompatible with “speed”. Range: (0,100].
        speed     int     target speed of the robot arm when executing this command, incompatible with “v”. If both "speed" and "v” exist, speed takes precedence. Range: [0, maximum motion speed], unit: mm/s.
        cp     int     continuous path rate, incompatible with “r”. Range: [0,100].
        r     int     continuous path radius, incompatible with “cp”. If both "r" and "cp” exist, r takes precedence. Unit: mm.
        """
        string = ""
        if coordinateMode == 0:
            string = "Arc(pose={{{:f},{:f},{:f},{:f},{:f},{:f}}},pose={{{:f},{:f},{:f},{:f},{:f},{:f}}}".format(
                a1, b1, c1, d1, e1, f1,  a2, b2, c2, d2, e2, f2)
        elif coordinateMode == 1:
            string = "Arc(joint={{{:f},{:f},{:f},{:f},{:f},{:f}}},joint={{{:f},{:f},{:f},{:f},{:f},{:f}}}".format(
                a1, b1, c1, d1, e1, f1,  a2, b2, c2, d2, e2, f2)
        else:
            print("coordinateMode  param  is wrong")
            return ""
        params = []
        if user != -1:
            params.append('user={:d}'.format(user))
        if tool != -1:
            params.append('tool={:d}'.format(tool))
        if a != -1:
            params.append('a={:d}'.format(a))
        if v != -1 and speed != -1:
            params.append('speed={:d}'.format(speed))
        elif speed != -1:
            params.append('speed={:d}'.format(speed))
        elif v != -1:
            params.append('v={:d}'.format(v))
        if cp != -1 and r != -1:
            params.append('r={:d}'.format(r))
        elif r != -1:
            params.append('r={:d}'.format(r))
        elif cp != -1:
            params.append('cp={:d}'.format(cp))
        for ii in params:
            string = string + ',' + ii
        string = string + ')'
        return self.sendRecvMsg(string)

    def Circle(self, a1, b1, c1, d1, e1, f1,  a2, b2, c2, d2, e2, f2, coordinateMode, count, user=-1, tool=-1, a=-1, v=-1, speed=-1, cp=-1, r=-1):
        """
        Description
        Move from the current position in a circle interpolated mode, and return to the current position after moving specified circles.
        As the circle needs to be determined through the current position, P1 and P2, the current position should not be in a straight line determined by P1 and P2, and the circle determined by the three points cannot exceed the motion range of the robot arm.
        Required parameter:
        Parameter name     Type     Description
        P1     string     Through point (joint variables or posture variables)
        P2     string     End point (joint variables or posture variables)
        coordinateMode     int      Coordinate mode of the target point, 0: pose, 1: joint
        count     int     Number of circles, range: [1,999].
        Optional parameter:
        Parameter name     Type     Description
        user     int     user coordinate system
        tool     int     tool coordinate system
        a     int     acceleration rate of the robot arm when executing this command. Range: (0,100].
        v     int     velocity rate of the robot arm when executing this command, incompatible with “speed”. Range: (0,100].
        speed     int     target speed of the robot arm when executing this command, incompatible with “v”. If both "speed" and "v” exist, speed takes precedence. Range: [0, maximum motion speed], unit: mm/s.
        cp     int     continuous path rate, incompatible with “r”. Range: [0,100].
        r     int     continuous path radius, incompatible with “cp”. If both "r" and "cp” exist, r takes precedence. Unit: mm.
        """
        string = ""
        if coordinateMode == 0:
            string = "Circle(pose={{{:f},{:f},{:f},{:f},{:f},{:f}}},pose={{{:f},{:f},{:f},{:f},{:f},{:f}}},{:d}".format(
                a1, b1, c1, d1, e1, f1,  a2, b2, c2, d2, e2, f2, count)
        elif coordinateMode == 1:
            string = "Circle(joint={{{:f},{:f},{:f},{:f},{:f},{:f}}},joint={{{:f},{:f},{:f},{:f},{:f},{:f}}},{:d}".format(
                a1, b1, c1, d1, e1, f1,  a2, b2, c2, d2, e2, f2, count)
        else:
            print("coordinateMode  param  is wrong")
            return ""
        params = []
        if user != -1:
            params.append('user={:d}'.format(user))
        if tool != -1:
            params.append('tool={:d}'.format(tool))
        if a != -1:
            params.append('a={:d}'.format(a))
        if v != -1 and speed != -1:
            params.append('speed={:d}'.format(speed))
        elif speed != -1:
            params.append('speed={:d}'.format(speed))
        elif v != -1:
            params.append('v={:d}'.format(v))
        if cp != -1 and r != -1:
            params.append('r={:d}'.format(r))
        elif r != -1:
            params.append('r={:d}'.format(r))
        elif cp != -1:
            params.append('cp={:d}'.format(cp))
        for ii in params:
            string = string + ',' + ii
        string = string + ')'
        return self.sendRecvMsg(string)

    def MoveJog(self, axis_id='', coordtype=-1, user=-1, tool=-1):
        """
        Joint motion
        axis_id: Joint motion axis, optional string value:
            J1+ J2+ J3+ J4+ J5+ J6+
            J1- J2- J3- J4- J5- J6- 
            X+ Y+ Z+ Rx+ Ry+ Rz+ 
            X- Y- Z- Rx- Ry- Rz-
        *dynParams: Parameter Settings（coord_type, user_index, tool_index）
                    coord_type: 1: User coordinate 2: tool coordinate (default value is 1)
                    user_index: user index is 0 ~ 9 (default value is 0)
                    tool_index: tool index is 0 ~ 9 (default value is 0)
        """
        string = "MoveJog({:s}".format(axis_id)
        params = []
        if coordtype != -1:
            params.append('coordtype={:d}'.format(coordtype))
        if user != -1:
            params.append('user={:d}'.format(user))
        if tool != -1:
            params.append('tool={:d}'.format(tool))
        for ii in params:
            string = string + ',' + ii
        string = string + ')'
        return self.sendRecvMsg(string)

    def GetStartPose(self, trace_name):
        """
        Description
        Get the start point of the trajectory.
        Required parameter:
        Parameter name     Type     Description
        traceName     string     trajectory file name (with suffix)
        The trajectory file is stored in /dobot/userdata/project/process/trajectory/.
        If the name contains Chinese, the encoding of the sender must be set to UTF-8, otherwise
        it will cause an exception for receiving Chinese.
        """
        string = "GetStartPose({:s})".format(trace_name)
        return self.sendRecvMsg(string)

    def StartPath(self, trace_name, isConst=-1, multi=-1.0, user=-1, tool=-1):
        """
        traceName string
        Description
        Move according to the recorded points in the specified trajectory file to play back the recorded trajectory.
        After the trajectory playback command is successfully delivered, you can check the robot status via RobotMode command.
        ROBOT_MODE_RUNNING: the robot is in trajectory playback, ROBOT_MODE_IDLE: trajectory playback is completed,
        ROBOT_MODE_ERROR: alarm.
        Required parameter:
        Parameter name     Type     Description
        traceName string
        trajectory file name (with suffix). The trajectory file is stored in /dobot/userdata/project/process/trajectory/.
        If the name contains Chinese, the encoding of the sender must be set to UTF-8, otherwise it will cause an exception for receiving Chinese.
        Optional parameter:
        Parameter name     Type     Description
        isConst     int     if or not to play back at a constant speed.
           1: the trajectory will be played back at the global rate at a uniform rate by the arm;
           0: the trajectory will be played back at the same speed as when it was recorded, and the motion speed can be scaled equivalently using the multi parameter, where the motion speed of the arm is not affected by the global rate.
        multi     double     Speed multiplier in playback, valid only when isConst=0. Range: [0.25, 2], 1 by default.
        user     int     User coordinate system index corresponding to the specified trajectory point (use the user coordinate system index recorded in the trajectory file if not specified).
        tool     int     tool coordinate system index corresponding to the specified trajectory point (use the tool coordinate system index recorded in the trajectory file if not specified).
        """
        string = "StartPath({:s}".format(trace_name)
        params = []
        if isConst != -1:
            params.append('isConst={:d}'.format(isConst))
        if multi != -1:
            params.append('multi={:f}'.format(multi))
        if user != -1:
            params.append('user={:d}'.format(user))
        if tool != -1:
            params.append('tool={:d}'.format(tool))
        for ii in params:
            string = string + ',' + ii
        string = string + ')'
        return self.sendRecvMsg(string)

    def RelMovJTool(self, offset_x, offset_y, offset_z, offset_rx, offset_ry, offset_rz, user=-1, tool=-1, a=-1, v=-1, cp=-1):
        """
        Description
        Perform relative motion along the tool coordinate system, and the end motion is joint motion.
        Required parameter:
        Parameter name     Type     Description
        offsetX     double     X-axis offset, unit: mm
        offsetY     double     Y-axis offset, unit: mm
        offsetZ     double     Z-axis offset, unit: mm
        offsetRx     double     Rx-axis offset, unit: °
        offsetRy     double     Ry-axis offset, unit: °
        offsetRz     double     Rz-axis offset, unit: °
        Optional parameter:
        Parameter name     Type     Description
        user     int     user coordinate system
        tool     int     tool coordinate system
        a     int     acceleration rate of the robot arm when executing this command. Range: (0,100].
        v     int     velocity rate of the robot arm when executing this command. Range: (0,100].
        cp     int     continuous path rate. Range: [0,100].
        """
        string = "RelMovJTool({:f},{:f},{:f},{:f},{:f},{:f}".format(
            offset_x, offset_y, offset_z, offset_rx, offset_ry, offset_rz)
        params = []
        if user != -1:
            params.append('user={:d}'.format(user))
        if tool != -1:
            params.append('tool={:d}'.format(tool))
        if a != -1:
            params.append('a={:d}'.format(a))
        if v != -1:
            params.append('v={:d}'.format(v))
        if cp != -1:
            params.append('cp={:d}'.format(cp))
        for ii in params:
            string = string + ',' + ii
        string = string + ')'
        return self.sendRecvMsg(string)

    def RelMovLTool(self, offset_x, offset_y, offset_z, offset_rx, offset_ry, offset_rz, user=-1, tool=-1, a=-1, v=-1, speed=-1, cp=-1, r=-1):
        """
        Description
        Perform relative motion along the tool coordinate system, and the end motion is linear motion.
        This command is for 6-axis robots.
        Required parameter:
        Parameter name     Type     Description
        offsetX     double     X-axis offset, unit: mm
        offsetY     double     Y-axis offset, unit: mm
        offsetZ     double     Z-axis offset, unit: mm
        offsetRx     double     Rx-axis offset, unit: °
        offsetRy     double     Ry-axis offset, unit: °
        offsetRz     double     Rz-axis offset, unit: °
        Optional parameter:
        Parameter name     Type     Description
        user     int     user coordinate system
        tool     int     tool coordinate system
        a     int     acceleration rate of the robot arm when executing this command. Range: (0,100].
        v     int     velocity rate of the robot arm when executing this command. Range: (0,100].
        speed     int     target speed of the robot arm when executing this command, incompatible with “v”. If both "speed" and "v” exist, speed takes precedence. Range: [0, maximum motion speed], unit: mm/s.
        cp     int     continuous path rate, incompatible with “r”. Range: [0,100].
        r     int     continuous path radius, incompatible with “cp”. If both "r" and "cp” exist, r takes precedence. Unit: mm.
        """
        string = "RelMovLTool({:f},{:f},{:f},{:f},{:f},{:f}".format(
            offset_x, offset_y, offset_z, offset_rx, offset_ry, offset_rz)
        params = []
        if user != -1:
            params.append('user={:d}'.format(user))
        if tool != -1:
            params.append('tool={:d}'.format(tool))
        if a != -1:
            params.append('a={:d}'.format(a))
        if v != -1 and speed != -1:
            params.append('speed={:d}'.format(speed))
        elif speed != -1:
            params.append('speed={:d}'.format(speed))
        elif v != -1:
            params.append('v={:d}'.format(v))
        if cp != -1 and r != -1:
            params.append('r={:d}'.format(r))
        elif r != -1:
            params.append('r={:d}'.format(r))
        elif cp != -1:
            params.append('cp={:d}'.format(cp))
        for ii in params:
            string = string + ',' + ii
        string = string + ')'
        return self.sendRecvMsg(string)

    def RelMovJUser(self, offset_x, offset_y, offset_z, offset_rx, offset_ry, offset_rz, user=-1, tool=-1, a=-1, v=-1, cp=-1):
        """
        Description
        Perform relative motion along the user coordinate system, and the end motion is joint motion.
        Required parameter:
        Parameter name     Type     Description
        offsetX     double     X-axis offset, unit: mm
        offsetY     double     Y-axis offset, unit: mm
        offsetZ     double     Z-axis offset, unit: mm
        offsetRx     double     Rx-axis offset, unit: °
        offsetRy     double     Ry-axis offset, unit: °
        offsetRz     double     Rz-axis offset, unit: °
        Optional parameter:
        Parameter name     Type     Description
        user     int     user coordinate system
        tool     int     tool coordinate system
        a     int     acceleration rate of the robot arm when executing this command. Range: (0,100].
        v     int     velocity rate of the robot arm when executing this command. Range: (0,100].
        cp     int     continuous path rate. Range: [0,100].
        """
        string = "RelMovJUser({:f},{:f},{:f},{:f},{:f},{:f}".format(
            offset_x, offset_y, offset_z, offset_rx, offset_ry, offset_rz)
        params = []
        if user != -1:
            params.append('user={:d}'.format(user))
        if tool != -1:
            params.append('tool={:d}'.format(tool))
        if a != -1:
            params.append('a={:d}'.format(a))
        if v != -1:
            params.append('v={:d}'.format(v))
        if cp != -1:
            params.append('cp={:d}'.format(cp))
        for ii in params:
            string = string + ',' + ii
        string = string + ')'
        return self.sendRecvMsg(string)

    def RelMovLUser(self, offset_x, offset_y, offset_z, offset_rx, offset_ry, offset_rz, user=-1, tool=-1, a=-1, v=-1, speed=-1, cp=-1, r=-1):
        """
        Description
        Perform relative motion along the user coordinate system, and the end motion is linear motion.
        Required parameter:
        Parameter name     Type     Description
        offsetX     double     X-axis offset, unit: mm
        offsetY     double     Y-axis offset, unit: mm
        offsetZ     double     Z-axis offset, unit: mm
        offsetRx     double     Rx-axis offset, unit: °
        offsetRy     double     Ry-axis offset, unit: °
        offsetRz     double     Rz-axis offset, unit: °
        Optional parameter:
        Parameter name     Type     Description
        user     int     user coordinate system
        tool     int     tool coordinate system
        a     int     acceleration rate of the robot arm when executing this command. Range: (0,100].
        v     int     velocity rate of the robot arm when executing this command. Range: (0,100].
        speed     int     target speed of the robot arm when executing this command, incompatible with “v”. If both "speed" and "v” exist, speed takes precedence. Range: [0, maximum motion speed], unit: mm/s.
        cp     int     continuous path rate, incompatible with “r”. Range: [0,100].
        r     int     continuous path radius, incompatible with “cp”. If both "r" and "cp” exist, r takes precedence. Unit: mm.
        """
        string = "RelMovLUser({:f},{:f},{:f},{:f},{:f},{:f}".format(
            offset_x, offset_y, offset_z, offset_rx, offset_ry, offset_rz)
        params = []
        if user != -1:
            params.append('user={:d}'.format(user))
        if tool != -1:
            params.append('tool={:d}'.format(tool))
        if a != -1:
            params.append('a={:d}'.format(a))
        if v != -1 and speed != -1:
            params.append('speed={:d}'.format(speed))
        elif speed != -1:
            params.append('speed={:d}'.format(speed))
        elif v != -1:
            params.append('v={:d}'.format(v))
        if cp != -1 and r != -1:
            params.append('r={:d}'.format(r))
        elif r != -1:
            params.append('r={:d}'.format(r))
        elif cp != -1:
            params.append('cp={:d}'.format(cp))
        for ii in params:
            string = string + ',' + ii
        string = string + ')'
        return self.sendRecvMsg(string)

    def RelJointMovJ(self, offset_x, offset_y, offset_z, offset_rx, offset_ry, offset_rz, a=-1, v=-1, cp=-1):
        """
        Description
        Perform relative motion along the joint coordinate system, and the end motion is joint motion.
        Required parameter:
        Parameter name     Type     Description
        offset1     double     J1-axis offset, unit: °
        offset2     double     J2-axis offset, unit: °
        offset3     double     J3-axis offset, unit: °
        offset4     double     J4-axis offset, unit: °
        offset5     double     J5-axis offset, unit: °
        offset6     double     J6-axis offset, unit: °
        Optional parameter:
        Parameter name     Type     Description
        a     int     acceleration rate of the robot arm when executing this command. Range: (0,100].
        v     int     velocity rate of the robot arm when executing this command. Range: (0,100].
        cp     int     continuous path rate. Range: [0,100].
        """
        string = "RelJointMovJ({:f},{:f},{:f},{:f},{:f},{:f}".format(
            offset_x, offset_y, offset_z, offset_rx, offset_ry, offset_rz)
        params = []
        if a != -1:
            params.append('a={:d}'.format(a))
        if v != -1:
            params.append('v={:d}'.format(v))
        if cp != -1:
            params.append('cp={:d}'.format(cp))
        for ii in params:
            string = string + ',' + ii
        string = string + ')'
        return self.sendRecvMsg(string)

    def GetCurrentCommandID(self):
        """
        Get the algorithm queue ID of the currently executed command, which can be used to judge which command is currently being executed by the robot.
        """
        string = "GetCurrentCommandID()"
        return self.sendRecvMsg(string)


    

    def SetResumeOffset(self, distance):
        """
        """
        string = "SetResumeOffset({:f})".format(distance)
        return self.sendRecvMsg(string)
    
    def PathRecovery(self):
        """
        """
        string = "PathRecovery()"
        return self.sendRecvMsg(string)

    def PathRecoveryStop(self):
        """
        """
        string = "PathRecoveryStop()"
        return self.sendRecvMsg(string)

    def PathRecoveryStatus(self):
        """
        """
        string = "PathRecoveryStatus()"
        return self.sendRecvMsg(string)
    

    def LogExportUSB(self, range):
        """
        """
        string = "LogExportUSB({:d})".format(range)
        return self.sendRecvMsg(string)
    
    def GetExportStatus(self):
        """
        """
        string = "GetExportStatus()"
        return self.sendRecvMsg(string)


    def EnableFTSensor(self, status):
        """
        """
        string = "EnableFTSensor({:d})".format(status)
        return self.sendRecvMsg(string)

    def SixForceHome(self):
        """
        """
        string = "SixForceHome()"
        return self.sendRecvMsg(string)
    
    def GetForce(self, tool = -1):
        """
        """
        if tool == -1:
            string = "GetForce()"
        else:
            string = "GetForce({:d})".format(tool)
        return self.sendRecvMsg(string)

    def ForceDriveMode(self, x, y, z, rx, ry, rz, user=-1):
        """
        {x,y,z,rx,ry,rz} string
        """
        string = ""
        string = "ForceDriveMode("+"{"+"{:d},{:d},{:d},{:d},{:d},{:d}".format(x,y,z,rx,ry,rz)+"}"
        if user != -1:
            string = string + ',{:d}'.format(user)
        string = string + ')'
        return self.sendRecvMsg(string)
    
    def ForceDriveSpeed(self, speed):
        """
        """
        string = "ForceDriveSpeed({:d})".format(speed)
        return self.sendRecvMsg(string)
    
    def FCForceMode(self, x, y, z, rx, ry, rz, fx,fy,fz,frx,fry,frz, reference=-1, user=-1,tool=-1):
        """
        {x,y,z,rx,ry,rz} 
        {fx,fy,fz,frx,fry,frz} 
        reference 
        user  
        tool  
        """
        string = ""
        string = "FCForceMode("+"{"+"{:d},{:d},{:d},{:d},{:d},{:d}".format(x,y,z,rx,ry,rz)+"},"+"{"+"{:d},{:d},{:d},{:d},{:d},{:d}".format(fx,fy,fz,frx,fry,frz)+"}"
        params = []
        if reference != -1:
            params.append('reference={:d}'.format(reference))
        if user != -1:
            params.append('user={:d}'.format(user))
        if tool != -1:
            params.append('tool={:d}'.format(tool))
        for ii in params:
            string = string + ','+ii
        string = string + ')'
        return self.sendRecvMsg(string)

    def FCSetDeviation(self, x, y, z, rx, ry, rz, controltype=-1):
        """
        x、y、z
        rx、ry、rz
        controltype
        """
        string = ""
        string = "FCSetDeviation("+"{"+"{:d},{:d},{:d},{:d},{:d},{:d}".format(x,y,z,rx,ry,rz)+"}"
        if controltype != -1:
            string = string + ',{:d}'.format(controltype)
        string = string + ')'
        return self.sendRecvMsg(string)
    
    def FCSetForceLimit(self, x, y, z, rx, ry, rz):
        """
        """
        string = ""
        string = "FCSetForceLimit("+"{:d},{:d},{:d},{:d},{:d},{:d}".format(x,y,z,rx,ry,rz)
        string = string + ')'
        return self.sendRecvMsg(string)
    
    def FCSetMass(self, x, y, z, rx, ry, rz):
        """
        """
        string = ""
        string = "FCSetMass("+"{:d},{:d},{:d},{:d},{:d},{:d}".format(x,y,z,rx,ry,rz)
        string = string + ')'
        return self.sendRecvMsg(string)
    
    def FCSetStiffness(self, x, y, z, rx, ry, rz):
        """
        """
        string = ""
        string = "FCSetStiffness("+"{:d},{:d},{:d},{:d},{:d},{:d}".format(x,y,z,rx,ry,rz)
        string = string + ')'
        return self.sendRecvMsg(string)
    
    def FCSetDamping(self, x, y, z, rx, ry, rz):
        """
        """
        string = ""
        string = "FCSetDamping("+"{:d},{:d},{:d},{:d},{:d},{:d}".format(x,y,z,rx,ry,rz)
        string = string + ')'
        return self.sendRecvMsg(string)

    def FCOff(self):
        """
        """
        string = "FCOff()"
        return self.sendRecvMsg(string)

    def FCSetForceSpeedLimit(self, x, y, z, rx, ry, rz):
        """
        """
        string = ""
        string = "FCSetForceSpeedLimit("+"{:d},{:d},{:d},{:d},{:d},{:d}".format(x,y,z,rx,ry,rz)
        string = string + ')'
        return self.sendRecvMsg(string)
    
    def FCSetForce(self, x, y, z, rx, ry, rz):
        """
        """
        string = ""
        string = "FCSetForce("+"{:d},{:d},{:d},{:d},{:d},{:d}".format(x,y,z,rx,ry,rz)
        string = string + ')'
        return self.sendRecvMsg(string)
    
    def RequestControl(self):
        """
        Request control of the robot.
        Note: This function sends a request for the control of the robot, which may be approved or denied.
        """
        string = "RequestControl()"
        return self.sendRecvMsg(string)
    


    def RelPointTool(self, coordinateMode,a1, b1, c1, d1, e1, f1, x, y, z, rx, ry, rz):
        """
        """
        string = ""
        if coordinateMode == 0:
            string = "RelPointTool(pose={{{:f},{:f},{:f},{:f},{:f},{:f}}},".format(
                a1, b1, c1, d1, e1, f1)
        elif coordinateMode == 1:
            string = "RelPointTool(joint={{{:f},{:f},{:f},{:f},{:f},{:f}}},".format(
                a1, b1, c1, d1, e1, f1)
        string = string + "{"+"{:f},{:f},{:f},{:f},{:f},{:f}".format(x,y,z,rx,ry,rz)+"}"
        string = string + ')'
        return self.sendRecvMsg(string)
    
    def RelPointUser(self,coordinateMode,a1, b1, c1, d1, e1, f1, x, y, z, rx, ry, rz):
        """
        """
        string = ""
        string = ""
        if coordinateMode == 0:
            string = "RelPointUser(pose={{{:f},{:f},{:f},{:f},{:f},{:f}}},".format(
                a1, b1, c1, d1, e1, f1)
        elif coordinateMode == 1:
            string = "RelPointUser(joint={{{:f},{:f},{:f},{:f},{:f},{:f}}},".format(
                a1, b1, c1, d1, e1, f1)
        string =string + "{"+"{:f},{:f},{:f},{:f},{:f},{:f}".format(x,y,z,rx,ry,rz)+"}"
        string = string + ')'
        return self.sendRecvMsg(string)

    def RelJoint(self, j1, j2, j3, j4, j5, j6, offset1, offset2, offset3, offset4, offset5, offset6):
        """
        RelJoint command
        """
        string = "RelJoint({:f},{:f},{:f},{:f},{:f},{:f},{{{:f},{:f},{:f},{:f},{:f},{:f}}})".format(
            j1, j2, j3, j4, j5, j6, offset1, offset2, offset3, offset4, offset5, offset6)
        return self.sendRecvMsg(string)
    
    def GetError(self, language="zh_cn"):
        """
        {
            "errMsg": [
                {
                    "id": xxx,
                    "level": xxx,
                    "description": "xxx",
                    "solution": "xxx",
                    "mode": "xxx",
                    "date": "xxxx",
                    "time": "xxxx"
                }
            ]
        }
        """
        try:

            language_url = f"http://{self.ip}:22000/interface/language"
            language_data = {"type": language}
            

            response = requests.post(language_url, json=language_data, timeout=5)
            if response.status_code != 200:
            

            alarm_url = f"http://{self.ip}:22000/protocol/getAlarm"
            response = requests.get(alarm_url, timeout=5)
            
            if response.status_code == 200:
                return response.json()
            else:
                return {"errMsg": []}
                
        except requests.exceptions.RequestException as e:
            return {"errMsg": []}
        except json.JSONDecodeError as e:
            return {"errMsg": []}
        except Exception as e:
            return {"errMsg": []}

    def ArcIO(self, a1, b1, c1, d1, e1, f1, a2, b2, c2, d2, e2, f2, coordinateMode, *io_params, user=-1, tool=-1, a=-1, v=-1, speed=-1, cp=-1, r=-1, mode=-1):
        """
        """
        string = ""
        if coordinateMode == 0:
            string = "ArcIO(pose={{{:f},{:f},{:f},{:f},{:f},{:f}}},pose={{{:f},{:f},{:f},{:f},{:f},{:f}}}".format(
                a1, b1, c1, d1, e1, f1, a2, b2, c2, d2, e2, f2)
        elif coordinateMode == 1:
            string = "ArcIO(joint={{{:f},{:f},{:f},{:f},{:f},{:f}}},joint={{{:f},{:f},{:f},{:f},{:f},{:f}}}".format(
                a1, b1, c1, d1, e1, f1, a2, b2, c2, d2, e2, f2)
        else:
            print("coordinateMode  param  is wrong")
            return ""
        
        for io_param in io_params:
            if isinstance(io_param, (list, tuple)) and len(io_param) == 4:
                string += ",{{{:d},{:d},{:d},{:d}}}".format(*io_param)
            else:
                 print("io_param format is wrong")

        params = []
        if user != -1:
            params.append('user={:d}'.format(user))
        if tool != -1:
            params.append('tool={:d}'.format(tool))
        if a != -1:
            params.append('a={:d}'.format(a))
        if v != -1 and speed != -1:
            params.append('speed={:d}'.format(speed))
        elif speed != -1:
            params.append('speed={:d}'.format(speed))
        elif v != -1:
            params.append('v={:d}'.format(v))
        if cp != -1 and r != -1:
            params.append('r={:d}'.format(r))
        elif r != -1:
            params.append('r={:d}'.format(r))
        elif cp != -1:
            params.append('cp={:d}'.format(cp))
        if mode != -1:
            params.append('mode={:d}'.format(mode))
        
        for ii in params:
            string += ',' + ii
        string += ')'
        return self.sendRecvMsg(string)

    def ArcTrackStart(self):
        return self.sendRecvMsg("ArcTrackStart()")

    def ArcTrackParams(self, sampleTime, coordinateType, upDownCompensationMin, upDownCompensationMax, upDownCompensationOffset, leftRightCompensationMin, leftRightCompensationMax, leftRightCompensationOffset):
        string = "ArcTrackParams({:d},{:d},{:f},{:f},{:f},{:f},{:f},{:f})".format(
            sampleTime, coordinateType, upDownCompensationMin, upDownCompensationMax, upDownCompensationOffset, leftRightCompensationMin, leftRightCompensationMax, leftRightCompensationOffset)
        return self.sendRecvMsg(string)

    def ArcTrackEnd(self):
        return self.sendRecvMsg("ArcTrackEnd()")

    def CheckMovC(self, j1a, j2a, j3a, j4a, j5a, j6a, j1b, j2b, j3b, j4b, j5b, j6b, j1c, j2c, j3c, j4c, j5c, j6c, user=-1, tool=-1, a=-1, v=-1, cp=-1):
        string = "CheckMovC(joint={{{:f},{:f},{:f},{:f},{:f},{:f}}},joint={{{:f},{:f},{:f},{:f},{:f},{:f}}},joint={{{:f},{:f},{:f},{:f},{:f},{:f}}}".format(
            j1a, j2a, j3a, j4a, j5a, j6a, j1b, j2b, j3b, j4b, j5b, j6b, j1c, j2c, j3c, j4c, j5c, j6c)
        params = []
        if user != -1: params.append('user={:d}'.format(user))
        if tool != -1: params.append('tool={:d}'.format(tool))
        if a != -1: params.append('a={:d}'.format(a))
        if v != -1: params.append('v={:d}'.format(v))
        if cp != -1: params.append('cp={:d}'.format(cp))
        if params: string += "," + ",".join(params)
        string += ")"
        return self.sendRecvMsg(string)

    def CheckMovJ(self, j1a, j2a, j3a, j4a, j5a, j6a, j1b, j2b, j3b, j4b, j5b, j6b, user=-1, tool=-1, a=-1, v=-1, cp=-1):
        string = "CheckMovJ(joint={{{:f},{:f},{:f},{:f},{:f},{:f}}},joint={{{:f},{:f},{:f},{:f},{:f},{:f}}}".format(
            j1a, j2a, j3a, j4a, j5a, j6a, j1b, j2b, j3b, j4b, j5b, j6b)
        params = []
        if user != -1: params.append('user={:d}'.format(user))
        if tool != -1: params.append('tool={:d}'.format(tool))
        if a != -1: params.append('a={:d}'.format(a))
        if v != -1: params.append('v={:d}'.format(v))
        if cp != -1: params.append('cp={:d}'.format(cp))
        if params: string += "," + ",".join(params)
        string += ")"
        return self.sendRecvMsg(string)

    def CheckOddMovC(self, j1a, j2a, j3a, j4a, j5a, j6a, j1b, j2b, j3b, j4b, j5b, j6b, j1c, j2c, j3c, j4c, j5c, j6c, user=-1, tool=-1, a=-1, v=-1, cp=-1):
        string = "CheckOddMovC(joint={{{:f},{:f},{:f},{:f},{:f},{:f}}},joint={{{:f},{:f},{:f},{:f},{:f},{:f}}},joint={{{:f},{:f},{:f},{:f},{:f},{:f}}}".format(
            j1a, j2a, j3a, j4a, j5a, j6a, j1b, j2b, j3b, j4b, j5b, j6b, j1c, j2c, j3c, j4c, j5c, j6c)
        params = []
        if user != -1: params.append('user={:d}'.format(user))
        if tool != -1: params.append('tool={:d}'.format(tool))
        if a != -1: params.append('a={:d}'.format(a))
        if v != -1: params.append('v={:d}'.format(v))
        if cp != -1: params.append('cp={:d}'.format(cp))
        if params: string += "," + ",".join(params)
        string += ")"
        return self.sendRecvMsg(string)

    def CheckOddMovJ(self, j1a, j2a, j3a, j4a, j5a, j6a, j1b, j2b, j3b, j4b, j5b, j6b, user=-1, tool=-1, a=-1, v=-1, cp=-1):
        string = "CheckOddMovJ(joint={{{:f},{:f},{:f},{:f},{:f},{:f}}},joint={{{:f},{:f},{:f},{:f},{:f},{:f}}}".format(
            j1a, j2a, j3a, j4a, j5a, j6a, j1b, j2b, j3b, j4b, j5b, j6b)
        params = []
        if user != -1: params.append('user={:d}'.format(user))
        if tool != -1: params.append('tool={:d}'.format(tool))
        if a != -1: params.append('a={:d}'.format(a))
        if v != -1: params.append('v={:d}'.format(v))
        if cp != -1: params.append('cp={:d}'.format(cp))
        if params: string += "," + ",".join(params)
        string += ")"
        return self.sendRecvMsg(string)

    def CheckOddMovL(self, j1a, j2a, j3a, j4a, j5a, j6a, j1b, j2b, j3b, j4b, j5b, j6b, user=-1, tool=-1, a=-1, v=-1, cp=-1):
        string = "CheckOddMovL(joint={{{:f},{:f},{:f},{:f},{:f},{:f}}},joint={{{:f},{:f},{:f},{:f},{:f},{:f}}}".format(
            j1a, j2a, j3a, j4a, j5a, j6a, j1b, j2b, j3b, j4b, j5b, j6b)
        params = []
        if user != -1: params.append('user={:d}'.format(user))
        if tool != -1: params.append('tool={:d}'.format(tool))
        if a != -1: params.append('a={:d}'.format(a))
        if v != -1: params.append('v={:d}'.format(v))
        if cp != -1: params.append('cp={:d}'.format(cp))
        if params: string += "," + ",".join(params)
        string += ")"
        return self.sendRecvMsg(string)

    def CnvInit(self, index):
        """
        CnvInit command
        """
        string = "CnvInit({:d})".format(index)
        return self.sendRecvMsg(string)

    def CnvMovL(self, j1, j2, j3, j4, j5, j6, user=-1, tool=-1, a=-1, v=-1, cp=-1, r=-1):
        string = "CnvMovL(pose={{{:f},{:f},{:f},{:f},{:f},{:f}}}".format(
            j1, j2, j3, j4, j5, j6)
        params = []
        if user != -1: params.append('user={:d}'.format(user))
        if tool != -1: params.append('tool={:d}'.format(tool))
        if a != -1: params.append('a={:d}'.format(a))
        if v != -1: params.append('v={:d}'.format(v))
        if cp != -1: params.append('cp={:d}'.format(cp))
        if r != -1: params.append('r={:d}'.format(r))
        if params: string += "," + ",".join(params)
        string += ")"
        return self.sendRecvMsg(string)

    def CnvMovC(self, j1a, j2a, j3a, j4a, j5a, j6a, j1b, j2b, j3b, j4b, j5b, j6b, user=-1, tool=-1, a=-1, v=-1, cp=-1, r=-1, mode=1):
        string = "CnvMovC(pose={{{:f},{:f},{:f},{:f},{:f},{:f}}},pose={{{:f},{:f},{:f},{:f},{:f},{:f}}}".format(
            j1a, j2a, j3a, j4a, j5a, j6a, j1b, j2b, j3b, j4b, j5b, j6b)
        params = []
        if user != -1: params.append('user={:d}'.format(user))
        if tool != -1: params.append('tool={:d}'.format(tool))
        if a != -1: params.append('a={:d}'.format(a))
        if v != -1: params.append('v={:d}'.format(v))
        if cp != -1: params.append('cp={:d}'.format(cp))
        if r != -1: params.append('r={:d}'.format(r))
        if mode != 1: params.append('mode={:d}'.format(mode))
        if params: string += "," + ",".join(params)
        string += ")"
        return self.sendRecvMsg(string)

    def CreateTray(self, *args, **kwargs):
        """
        CreateTray command
        Due to missing documentation on exact parameters, this function uses dynamic arguments.
        Example: CreateTray(rows=3, cols=4, ...)
        """
        return self.sendRecvMsg(self._build_cmd("CreateTray", *args, **kwargs))

    def EndRTOffset(self):
        return self.sendRecvMsg("EndRTOffset()")

    def StartRTOffset(self):
        """
        StartRTOffset command
        """
        return self.sendRecvMsg("StartRTOffset()")

    def FCCollisionSwitch(self, enable):
        return self.sendRecvMsg("FCCollisionSwitch(enable={:d})".format(enable))

    def SetFCCollision(self, force, torque):
        return self.sendRecvMsg("SetFCCollision({:f},{:f})".format(force, torque))

    def GetCnvObject(self, objId):
        return self.sendRecvMsg("GetCnvObject({:d})".format(objId))

    def DOGroupDEC(self, group, value):
        return self.sendRecvMsg("DOGroupDEC({:d},{:d})".format(group, value))

    def GetDOGroupDEC(self, group, value):
        return self.sendRecvMsg("GetDOGroupDEC({:d},{:d})".format(group, value))

    def DIGroupDEC(self, group, value):
        return self.sendRecvMsg("DIGroupDEC({:d},{:d})".format(group, value))

    def InverseSolution(self, a1, b1, c1, d1, e1, f1, user=-1, tool=-1, isJoint=0):
        """
        InverseSolution command
        """
        string = "InverseSolution(pose={{{:f},{:f},{:f},{:f},{:f},{:f}}}".format(
            a1, b1, c1, d1, e1, f1)
        
        params = []
        if user != -1:
            params.append('user={:d}'.format(user))
        if tool != -1:
            params.append('tool={:d}'.format(tool))
        if isJoint != 0:
            params.append('isJoint={:d}'.format(isJoint))
            
        for ii in params:
            string += ',' + ii
        string += ')'
        return self.sendRecvMsg(string)

    def MoveL(self, a1, b1, c1, d1, e1, f1, user=-1, tool=-1, a=-1, v=-1, speed=-1, cp=-1, r=-1):
        """
        MoveL command
        """
        string = "MoveL(pose={{{:f},{:f},{:f},{:f},{:f},{:f}}}".format(
            a1, b1, c1, d1, e1, f1)
        
        params = []
        if user != -1:
            params.append('user={:d}'.format(user))
        if tool != -1:
            params.append('tool={:d}'.format(tool))
        if a != -1:
            params.append('a={:d}'.format(a))
        if v != -1 and speed != -1:
            params.append('speed={:d}'.format(speed))
        elif speed != -1:
            params.append('speed={:d}'.format(speed))
        elif v != -1:
            params.append('v={:d}'.format(v))
        if cp != -1 and r != -1:
            params.append('r={:d}'.format(r))
        elif r != -1:
            params.append('r={:d}'.format(r))
        elif cp != -1:
            params.append('cp={:d}'.format(cp))
            
        for ii in params:
            string += ',' + ii
        string += ')'
        return self.sendRecvMsg(string)

    def MovS(self, file=None, coordinateMode=-1, points=None, user=-1, tool=-1, v=-1, speed=-1, a=-1, freq=-1):
        """
        MovS command
        """
        string = "MovS("
        if file is not None:
             string += "file={:s}".format(file)
        elif points is not None and coordinateMode != -1:
             # points should be a list of tuples/lists
             pts_str = []
             for pt in points:
                 if coordinateMode == 0:
                     pts_str.append("pose={{{:f},{:f},{:f},{:f},{:f},{:f}}}".format(*pt))
                 elif coordinateMode == 1:
                     pts_str.append("joint={{{:f},{:f},{:f},{:f},{:f},{:f}}}".format(*pt))
             string += ",".join(pts_str)
        else:
             print("MovS param is wrong")
             return ""
        
        params = []
        if user != -1:
            params.append('user={:d}'.format(user))
        if tool != -1:
            params.append('tool={:d}'.format(tool))
        if v != -1 and speed != -1:
            params.append('speed={:d}'.format(speed))
        elif speed != -1:
            params.append('speed={:d}'.format(speed))
        elif v != -1:
            params.append('v={:d}'.format(v))
        if a != -1:
             params.append('a={:d}'.format(a))
        if freq != -1:
             params.append('freq={:d}'.format(freq))
             
        if len(params) > 0:
             if file is not None or (points is not None and len(points) > 0):
                  string += ","
             string += ",".join(params)
             
        string += ")"
        return self.sendRecvMsg(string)

    def OffsetPara(self, x, y, z, rx, ry, rz):
        """
        OffsetPara command
        """
        string = "OffsetPara({:f},{:f},{:f},{:f},{:f},{:f})".format(x, y, z, rx, ry, rz)
        return self.sendRecvMsg(string)


    def GetTrayPoint(self, *args, **kwargs):
        """
        GetTrayPoint command
        Due to missing documentation on exact parameters, this function uses dynamic arguments.
        Example: GetTrayPoint(trayName)
        """
        return self.sendRecvMsg(self._build_cmd("GetTrayPoint", *args, **kwargs))

    def ResetRobot(self):
        return self.sendRecvMsg("ResetRobot()")

    def RunTo(self, a1, b1, c1, d1, e1, f1, moveType, user=-1, tool=-1, a=-1, v=-1):
        """
        RunTo command
        """
        string = ""
        if moveType == 0:
            string = "RunTo(pose={{{:f},{:f},{:f},{:f},{:f},{:f}}},moveType=0".format(
                a1, b1, c1, d1, e1, f1)
        elif moveType == 1:
            string = "RunTo(joint={{{:f},{:f},{:f},{:f},{:f},{:f}}},moveType=1".format(
                a1, b1, c1, d1, e1, f1)
        else:
             print("moveType param is wrong")
             return ""
        
        params = []
        if user != -1:
            params.append('user={:d}'.format(user))
        if tool != -1:
            params.append('tool={:d}'.format(tool))
        if a != -1:
            params.append('a={:d}'.format(a))
        if v != -1:
            params.append('v={:d}'.format(v))
        
        for ii in params:
            string += ',' + ii
        string += ')'
        return self.sendRecvMsg(string)

    def SetArcTrackOffset(self, offsetX, offsetY, offsetZ, offsetRx, offsetRy, offsetRz):
        string = "SetArcTrackOffset({{{:f},{:f},{:f},{:f},{:f},{:f}}})".format(
            offsetX, offsetY, offsetZ, offsetRx, offsetRy, offsetRz)
        return self.sendRecvMsg(string)

    def SetCnvPointOffset(self, xOffset, yOffset):
        return self.sendRecvMsg("SetCnvPointOffset({:f},{:f})".format(xOffset, yOffset))

    def SetCnvTimeCompensation(self, time):
        return self.sendRecvMsg("SetCnvTimeCompensation({:d})".format(time))

    def StartSyncCnv(self):
        return self.sendRecvMsg("StartSyncCnv()")

    def StopSyncCnv(self):
        return self.sendRecvMsg("StopSyncCnv()")

    def TcpSendAndParse(self, cmd):
        """
        TcpSendAndParse command
        """
        return self.sendRecvMsg("TcpSendAndParse(\"{:s}\")".format(cmd))

    def Sleep(self, count):
        return self.sendRecvMsg("Sleep({:d})".format(count))

    def RelPointWeldLine(self, StartX, EndX, Y, Z, WorkAngle, TravelAngle, P1, P2):
        string = "RelPointWeldLine({:f},{:f},{:f},{:f},{:f},{:f},{{{:f},{:f},{:f},{:f},{:f},{:f}}},{{{:f},{:f},{:f},{:f},{:f},{:f}}})".format(
            StartX, EndX, Y, Z, WorkAngle, TravelAngle, P1[0], P1[1], P1[2], P1[3], P1[4], P1[5], P2[0], P2[1], P2[2], P2[3], P2[4], P2[5])
        return self.sendRecvMsg(string)

    def RelPointWeldArc(self, StartX, EndX, Y, Z, WorkAngle, TravelAngle, P1, P2, P3):
        string = "RelPointWeldArc({:f},{:f},{:f},{:f},{:f},{:f},{{{:f},{:f},{:f},{:f},{:f},{:f}}},{{{:f},{:f},{:f},{:f},{:f},{:f}}},{{{:f},{:f},{:f},{:f},{:f},{:f}}})".format(
            StartX, EndX, Y, Z, WorkAngle, TravelAngle, P1[0], P1[1], P1[2], P1[3], P1[4], P1[5], P2[0], P2[1], P2[2], P2[3], P2[4], P2[5], P3[0], P3[1], P3[2], P3[3], P3[4], P3[5])
        return self.sendRecvMsg(string)

    def WeaveStart(self):
        return self.sendRecvMsg("WeaveStart()")

    def WeaveParams(self, weldType, frequency, leftAmplitude, rightAmplitude, direction, stopMode, stopTime1, stopTime2, stopTime3, stopTime4, radius, radian, **kwargs):
        string = "WeaveParams({:d},{:f},{:f},{:f},{:d},{:d},{:d},{:d},{:d},{:d},{:f},{:f}".format(
            weldType, frequency, leftAmplitude, rightAmplitude, direction, stopMode, stopTime1, stopTime2, stopTime3, stopTime4, radius, radian)
        if kwargs:
            for key, value in kwargs.items():
                string += ",{}={}".format(key, value)
        string += ")"
        return self.sendRecvMsg(string)

    def WeaveEnd(self):
        return self.sendRecvMsg("WeaveEnd()")

    def WeldArcSpeedStart(self):
        return self.sendRecvMsg("WeldArcSpeedStart()")

    def WeldArcSpeed(self, speed):
        return self.sendRecvMsg("WeldArcSpeed({:f})".format(speed))

    def WeldArcSpeedEnd(self):
        return self.sendRecvMsg("WeldArcSpeedEnd()")

    def WeldWeaveStart(self, weldType, frequency, leftAmplitude, rightAmplitude, direction, stopMode, stopTime1, stopTime2, stopTime3, stopTime4, radius, radian):
        string = "WeldWeaveStart({:d},{:f},{:f},{:f},{:d},{:d},{:d},{:d},{:d},{:d},{:f},{:f})".format(
            weldType, frequency, leftAmplitude, rightAmplitude, direction, stopMode, stopTime1, stopTime2, stopTime3, stopTime4, radius, radian)
        return self.sendRecvMsg(string)


# Feedback interface



class DobotApiFeedBack(DobotApi):
    def __init__(self, ip, port, *args):
        super().__init__(ip, port, *args)
        self.__MyType = []
        self.last_recv_time = time.perf_counter()
        

    def feedBackData(self):
        """
        Return the robot status
        """
        self.socket_dobot.setblocking(True)
        data = bytes()
        current_recv_time = time.perf_counter()
        temp = self.socket_dobot.recv(144000)
        if len(temp) > 1440:    
            temp = self.socket_dobot.recv(144000)
        #print("get:",len(temp))
        i=0
        if len(temp) < 1440:
            while i < 5 :

                temp = self.socket_dobot.recv(144000)
                if len(temp) > 1440:
                    break
                i+=1
            if i >= 5:
        
        interval = (current_recv_time - self.last_recv_time) * 1000
        self.last_recv_time = current_recv_time
        #print(f"Time interval since last receive: {interval:.3f} ms")
        
        data = temp[0:1440]
        #print(len(data))
        #print(f"Single element size of MyType: {MyType.itemsize} bytes")
        self.__MyType = None   

        if len(data) == 1440:        
            self.__MyType = np.frombuffer(data, dtype=MyType)

        return self.__MyType
        
