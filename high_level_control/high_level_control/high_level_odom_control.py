import sys
from unitree_sdk2py.rpc.client import Client
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
import time


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
ROBOT_API_ID_LOCO_ENABLE_ODOM = 8201
ROBOT_API_ID_LOCO_DISABLE_ODOM = 8202
ROBOT_API_ID_LOCO_GET_ODOM = 8203
ROBOT_API_ID_LOCO_SET_TARGET_POSITION = 8204

"""
" options
"""
INTERFACE = "wlp0s20f3"
FREQUENCY = 10


class OdomClient(Client):
    def __init__(self):
        super().__init__(LOCO_SERVICE_NAME, False)

    def Init(self):
        # set api version
        self._SetApiVerson(LOCO_API_VERSION)

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


def main():
    '''
    Simple odometry reader
    '''
    if len(sys.argv) > 1:
        ChannelFactoryInitialize(0, INTERFACE)
    else:
        ChannelFactoryInitialize(0)
    my_client = OdomClient()
    my_client.Init()
    my_client.SetTimeout(1.0)
    my_client.EnableOdom()
    time.sleep(1)
    input('Press any key to start')

    while True:
        try:
            print(my_client.GetOdom())
            time.sleep(1.0 / FREQUENCY)
        except KeyboardInterrupt:
            print('Stop node')
            break

    my_client.DisableOdom()


if __name__ == "__main__":
    main()
