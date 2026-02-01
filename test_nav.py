from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
import numpy as np

pt = np.array([1,0,0])

loco = LocoClient()
loco.init()

pos = np.array([0,0,0])

while True:

    vel = 


