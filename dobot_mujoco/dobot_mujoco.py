import os
import time
from pathlib import Path
from threading import Thread
import threading
import mujoco
import mujoco.viewer
import numpy as np
import cv2

class DobotMujoco:
    def __init__(self, model_path:str) -> None:
        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        self.data = mujoco.MjData(self.model)
        mujoco.mj_resetDataKeyframe(self.model, self.data, 0)
        self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
        self.renderer = mujoco.Renderer(self.model, 480, 640)
        self.lock = threading.Lock()
        self._thread = threading.Thread(target=self.sim_thread)
        time.sleep(0.5)
        
    def __del__(self):
        self.stop()
        
    def start(self):
        self._thread.start()
    
    def stop(self):
        self._thread.join()
        
    def sim_thread(self):
        while self.viewer.is_running():
            step_start = time.perf_counter()
            with self.lock:
                mujoco.mj_step(self.model, self.data)
                self.renderer.update_scene(self.data, camera="wrist_cam_left")
                cam_img = self.renderer.render()
                cv2.imwrite("cam.png", cam_img)
                self.viewer.sync()
                time_until_next_step = self.model.opt.timestep - (
                time.perf_counter() - step_start
                )
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)
            
    def get_joint(self):
        with self.lock:
            joint = self.data.qpos
        joint_deg = np.rad2deg(joint)
        return joint_deg
    
    def servoj(self, joint:np.ndarray):
        joint = np.deg2rad(joint)
        with self.lock:
            self.data.ctrl = joint
    
            
if __name__=="__main__":
    model_path = Path(__file__).parent.parent / 'dobot_mujoco' / 'nova2' / 'scene.xml'
    dobot_mujoco = DobotMujoco(model_path=model_path)
    dobot_mujoco.start()
    while True:
        dobot_mujoco.servoj(np.array([0, 0, -90, 0, 90, 90, 0]))
        time.sleep(3)
        print(dobot_mujoco.get_joint())
        dobot_mujoco.servoj(np.array([-90, -45, -90, 120, 90, 90, 5]))
        time.sleep(3)
        print(dobot_mujoco.get_joint())