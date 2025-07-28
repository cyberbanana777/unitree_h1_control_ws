import time

from helloworld import HelloWorld
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher

ChannelFactoryInitialize()

pub = ChannelPublisher("topic", HelloWorld)
pub.Init()

for i in range(30):
    msg = HelloWorld("Hello world. time:" + str(time.time()))
    # msg.data = "Hello world. time:" + str(time.time())

    if pub.Write(msg, 0.5):
        print("publish success. msg:", msg)
    else:
        print("publish error.")

    time.sleep(1)

pub.Close()
