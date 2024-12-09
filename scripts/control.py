from dobot_mujoco import DobotMujoco
import numpy as np
from pathlib import Path
import time

model_path = Path(__file__).parent.parent / 'dobot_mujoco' / 'nova2' / 'scene.xml'
dobot = DobotMujoco(model_path)
dobot.start()
while True:
    dobot.servoj(np.array([0, 0, -90, 0, 90, 90]))
    time.sleep(3)
    print(dobot.get_joint())
    dobot.servoj(np.array([-90, -45, -90, 45, 90, 90]))
    time.sleep(3)
    print(dobot.get_joint())