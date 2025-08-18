
import sys
from unitree_sdk2py.rpc.client import Client
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
import time
import json


"""
" service name
"""
LOCO_SERVICE_NAME = "loco"


"""
" service api version
"""
LOCO_API_VERSION = "2.0.0.0"


"""
" api id
"""
ROBOT_API_ID_LOCO_GET_FSM_ID = 8001
ROBOT_API_ID_LOCO_GET_FSM_MODE = 8002
ROBOT_API_ID_LOCO_GET_BALANCE_MODE = 8003
ROBOT_API_ID_LOCO_GET_SWING_HEIGHT = 8004
ROBOT_API_ID_LOCO_GET_STAND_HEIGHT = 8005
ROBOT_API_ID_LOCO_GET_PHASE = 8006  # deprecated

ROBOT_API_ID_LOCO_SET_FSM_ID = 8101
ROBOT_API_ID_LOCO_SET_BALANCE_MODE = 8102
ROBOT_API_ID_LOCO_SET_SWING_HEIGHT = 8103
ROBOT_API_ID_LOCO_SET_STAND_HEIGHT = 8104
ROBOT_API_ID_LOCO_SET_VELOCITY = 8105
ROBOT_API_ID_LOCO_SET_PHASE = 8106
ROBOT_API_ID_LOCO_SET_ARM_TASK = 8107

ROBOT_API_ID_LOCO_ENABLE_ODOM = 8201
ROBOT_API_ID_LOCO_DISABLE_ODOM = 8202
ROBOT_API_ID_LOCO_GET_ODOM = 8203
ROBOT_API_ID_LOCO_SET_TARGET_POSITION = 8204


"""
" error code
"""

INTERFACE = "wlp0s20f3"






class OdomClient(Client):
    def __init__(self):
        super().__init__(LOCO_SERVICE_NAME, False)

    def Init(self):
        # set api version
        self._SetApiVerson(LOCO_API_VERSION)

        # regist api
        self._RegistApi(ROBOT_API_ID_LOCO_GET_FSM_ID, 0)
        self._RegistApi(ROBOT_API_ID_LOCO_GET_FSM_MODE, 0)
        self._RegistApi(ROBOT_API_ID_LOCO_GET_BALANCE_MODE, 0)
        self._RegistApi(ROBOT_API_ID_LOCO_GET_SWING_HEIGHT, 0)
        self._RegistApi(ROBOT_API_ID_LOCO_GET_STAND_HEIGHT, 0)
        self._RegistApi(ROBOT_API_ID_LOCO_GET_PHASE, 0)  # deprecated

        self._RegistApi(ROBOT_API_ID_LOCO_SET_FSM_ID, 0)
        self._RegistApi(ROBOT_API_ID_LOCO_SET_BALANCE_MODE, 0)
        self._RegistApi(ROBOT_API_ID_LOCO_SET_SWING_HEIGHT, 0)
        self._RegistApi(ROBOT_API_ID_LOCO_SET_STAND_HEIGHT, 0)
        self._RegistApi(ROBOT_API_ID_LOCO_SET_VELOCITY, 0)
        self._RegistApi(ROBOT_API_ID_LOCO_SET_PHASE, 0)
        self._RegistApi(ROBOT_API_ID_LOCO_SET_ARM_TASK, 0)

        self._RegistApi(ROBOT_API_ID_LOCO_ENABLE_ODOM, 0)
        self._RegistApi(ROBOT_API_ID_LOCO_DISABLE_ODOM, 0)
        self._RegistApi(ROBOT_API_ID_LOCO_GET_ODOM, 0)
        self._RegistApi(ROBOT_API_ID_LOCO_SET_TARGET_POSITION, 0)

    def EnableOdom(self):
        code, date = self._Call(ROBOT_API_ID_LOCO_ENABLE_ODOM, "")
        return code
    
    def DisableOdom(self):
        code, date = self._Call(ROBOT_API_ID_LOCO_DISABLE_ODOM, "")
        return code
    
    def GetOdom(self):
        code, data = self._Call(ROBOT_API_ID_LOCO_GET_ODOM, "")
        return code, data

    # 8101
    def SetFsmId(self, fsm_id: int):
        p = {}
        p["data"] = fsm_id
        parameter = json.dumps(p)
        code, data = self._Call(ROBOT_API_ID_LOCO_SET_FSM_ID, parameter)
        return code

    # 8104
    def SetStandHeight(self, stand_height: float):
        p = {}
        p["data"] = stand_height
        parameter = json.dumps(p)
        code, data = self._Call(ROBOT_API_ID_LOCO_SET_STAND_HEIGHT, parameter)
        return code

    # 8105
    def SetVelocity(
        self, vx: float, vy: float, omega: float, duration: float = 1.0
    ):
        p = {}
        velocity = [vx, vy, omega]
        p["velocity"] = velocity
        p["duration"] = duration
        parameter = json.dumps(p)
        code, data = self._Call(ROBOT_API_ID_LOCO_SET_VELOCITY, parameter)
        return code

    def Damp(self):
        self.SetFsmId(1)

    def Start(self):
        self.SetFsmId(204)

    def StandUp(self):
        self.SetFsmId(2)

    def ZeroTorque(self):
        self.SetFsmId(0)

    def StopMove(self):
        self.SetVelocity(0.0, 0.0, 0.0)

    def HighStand(self):
        UINT32_MAX = (1 << 32) - 1
        self.SetStandHeight(UINT32_MAX)

    def LowStand(self):
        UINT32_MIN = 0
        self.SetStandHeight(UINT32_MIN)

    def Move(
        self, vx: float, vy: float, vyaw: float, continous_move: bool = False
    ):
        duration = 864000.0 if continous_move else 1
        self.SetVelocity(vx, vy, vyaw, duration)


FREQUENCY = 200



def main():
    if len(sys.argv) > 1:
        ChannelFactoryInitialize(0, INTERFACE)
    else:
        ChannelFactoryInitialize(0)
    my_client = OdomClient()
    my_client.Init()
    my_client.SetTimeout(1.0 /  FREQUENCY)
    my_client.EnableOdom()
    time.sleep(1)
    input('Press any key to start')

    while True:
        try:
            print(my_client.GetOdom())
            # time.sleep(1.0 / FREQUENCY)
        except KeyboardInterrupt:
            print('Stop node')
            break
        finally:
            my_client.DisableOdom()


if __name__ == "__main__":
    main()


