import time
import pybullet as p
import numpy as np
from pybullet_utils.bullet_client import BulletClient

import cv2

from bullet_env.bullet_robot import BulletRobot, BulletGripper
from transform import Affine

# setup
RENDER = True
URDF_PATH = "/home/jovyan/workspace/assets/urdf/robot.urdf"

bullet_client = BulletClient(connection_mode=p.GUI)
bullet_client.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
if not RENDER:
    bullet_client.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)

bullet_client.resetSimulation()

robot = BulletRobot(bullet_client=bullet_client, urdf_path=URDF_PATH)
gripper = BulletGripper(bullet_client=bullet_client, robot_id=robot.robot_id)
robot.home()

# spawn an object
object_urdf_path = "/home/jovyan/workspace/assets/objects/cube/object.urdf"
object_pose = Affine(translation=[0.5, 0, 0.1])
object_id = bullet_client.loadURDF(
    object_urdf_path,
    object_pose.translation,
    object_pose.quat,
    flags=bullet_client.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)

# simulate the scene for 100 steps and wait for the object to settle
for _ in range(100):
    bullet_client.stepSimulation()
    time.sleep(1 / 100)

# implement grasping the object
# keep in mind, that the object pose is defined in the world frame, and the eef points downwards
# also, make sure that before grasping the gripper is open
# consider adding a pre-grasp pose to ensure the object is grasped correctly without collision during approach

# home robot
robot.home()
gripper.open()

# get/update the current object pose (probabbly changed due to gravity)
position, quat = bullet_client.getBasePositionAndOrientation(object_id)
object_pose = Affine(position, quat)

# define target/objet pose with vertical to world orientation and rotated around z-axis
gripper_rotation = Affine(rotation=[0, np.pi, 0])
grip_pose = object_pose * gripper_rotation

# move 0.1 over cube
overObj_pose = grip_pose * Affine(translation=[0, 0, -0.1])
robot.ptp(overObj_pose)

# move down to object
robot.lin(grip_pose)

# close gripper
gripper.close()


# move up
robot.lin(overObj_pose)


# pause/"hold" 
random_noise = np.random.randint(0, 256, (480, 640, 3), dtype=np.uint8)
cv2.imshow("random_noise", random_noise)
cv2.waitKey(0)


# close the simulation
bullet_client.disconnect()
