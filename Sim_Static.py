import pybullet as p
import pybullet_data
import time
import numpy as np
import math
from scipy.optimize import fsolve
from roboticstoolbox import DHRobot, PrismaticDH, RevoluteDH
from math import pi
from math import sqrt
from spatialmath import SE3
from game_input import get_hoop_position
import matplotlib.pyplot as plt

# รับค่าตำแหน่งห่วงบาสจากผู้ใช้โดยเรียกใช้ Pygame
x_position, z_position = get_hoop_position()

# สร้างโมเดลหุ่นยนต์ด้วย DH Parameters
robot = DHRobot([
    PrismaticDH(a=0, alpha=0, theta=0, qlim=[0, 2]),  
    RevoluteDH(a=0, alpha=pi/2, d=0, qlim=[np.radians(0), np.radians(10)]),  
    RevoluteDH(a=0.4, alpha=0, d=0, qlim=[np.radians(50), np.radians(120)]),  
    RevoluteDH(a=0.34, alpha=0, d=0, qlim=[np.radians(-20), np.radians(20)])  
], name="Basketball_Shooting_Robot")
robot.tool = SE3.Tx(0.22)

n_joints = robot.n
print('จำนวน Joint',n_joints)

# ท่า home, shooting และ rest
home = [1.4, np.radians(10), np.radians(120), np.radians(20)]
shooting = [1.4, np.radians(5), np.radians(70), np.radians(-10)]
rest = [1.4, np.radians(0), np.radians(50), np.radians(-20)]

n_paths = 101
def interpolate_joint_angles(home, shooting, rest, n_paths):
    q_path = []
    for i in range(51):
        q_interpolated = [
            np.interp(i, [0, n_paths-1], [home[j], shooting[j]]) for j in range(len(home))
        ]
        q_path.append(q_interpolated)
    for i in range(50):
        q_interpolated = [
            np.interp(i, [0, n_paths-1], [shooting[j], rest[j]]) for j in range(len(shooting))
        ]
        q_path.append(q_interpolated)
    return np.array(q_path)

q_path = interpolate_joint_angles(home, shooting, rest, n_paths)
        
rounded_array = np.round(q_path, 2)
print('q_path 1 :',rounded_array[:51])
print('q_path 2 :',rounded_array[51:])

# ตั้งค่าตำแหน่งเริ่มต้นของลูกบาสและห่วงบาส
basketball_start_pos = [0, 0, 2.73]  
hoop_position = [x_position, 0, z_position]  

def calculate_velocity(basketball_start_pos, hoop_position):
    g = 9.81  
    distance_xy = np.sqrt((hoop_position[0] - basketball_start_pos[0])**2 + 
                          (hoop_position[1] - basketball_start_pos[1])**2)
    z_start = basketball_start_pos[2]
    z_hoop = hoop_position[2]
    time_to_reach_hoop = math.sqrt(2 * (z_hoop - z_start) / g)
    v0 = distance_xy / time_to_reach_hoop
    return v0, time_to_reach_hoop

v0, time_to_reach_hoop = calculate_velocity(basketball_start_pos, hoop_position)

# Parameters
x_start = 0.18 # Start position in x-axis (m)
x_end = 0.62    # End position in x-axis (m)
t_total = 1.0  # Total time (s)
t1, t2, t3 = 0.3, 0.7, 0.3  # Key time points (s)
dt = 0.01      # Time step (s)
m_basketball = 0.625  # Mass of basketball (kg)
r = 0.22       # Distance (m) for torque calculation

t = np.arange(0, t_total + dt, dt)

# Velocity Profile
velocity = np.zeros_like(t)
for i, ti in enumerate(t):
    if ti <= t1:  # Acceleration phase
        velocity[i] = (v0 / t1) * ti
    elif ti <= t2:  # Constant velocity phase
        velocity[i] = v0
    elif ti <= t3:  # Deceleration phase
        velocity[i] = v0 * (1 - (ti - t2) / (t3 - t2))

# Acceleration Profile 
acceleration = np.gradient(velocity, dt)

# Position Profile 
position = np.cumsum(velocity) * dt
position = x_start + (x_end - x_start) * (position / position[-1])

# Calculate Force and Torque at each time step
forces = m_basketball * acceleration
torques = r * forces

# Path in x-axis
x_path = position
vz=0
# ฟังก์ชันคำนวณ Jacobian Matrix
def calculate_jacobian(robot, q):
    J = robot.jacob0(q)  
    return J[0:3, 1:]  
joint_velocities = []

for i, vx in enumerate(velocity): 
    eff_velocity = np.array([vx, 0, vz])  
    J = calculate_jacobian(robot, q_path[i])
    if J.shape[0] != J.shape[1]:
        raise ValueError(f"Jacobian is not square at step {i}: shape={J.shape}")
    if np.linalg.det(J) == 0.01:
        raise ValueError(f"Jacobian is singular at step {i}")
    J_inverse = np.linalg.inv(J)
    q_dot = np.dot(J_inverse, eff_velocity)
    joint_velocities.append(q_dot)  

joint_velocities = np.array(joint_velocities)
print('joint_velocities',joint_velocities)

# เริ่มต้น PyBullet
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")  
p.setGravity(0, 0, -9.81)
cameraDistance = 12.0
cameraYaw = -70
cameraPitch = -10
cameraTarget = [7, 0, 2]
p.resetDebugVisualizerCamera(cameraDistance, cameraYaw, cameraPitch, cameraTarget)

robot_id = p.loadURDF("basketball_robot.urdf", [0, 0, 0], useFixedBase=True)
home = [1.4, np.radians(10), np.radians(120), np.radians(20)]
shooting = [1.4, np.radians(5), np.radians(70), np.radians(-10)]
rest = [1.4, np.radians(0), np.radians(50), np.radians(-20)]

class BallStateTracker:
    def __init__(self):
        self.previous_z = None   
    def check_ball_through_transparent_hoop(self, ball_body, hoop_position, max_distance=0.3):
        ball_pos, _ = p.getBasePositionAndOrientation(ball_body)
        velocity = p.getBaseVelocity(ball_body)[0]
        ball_xy = np.array(ball_pos[:2])
        hoop_xy = np.array(hoop_position[:2])
        distance = np.linalg.norm(ball_xy - hoop_xy)
        is_within_radius = distance <= max_distance  # ลูกบาสอยู่ในรัศมีห่วง
        is_moving_downward = velocity[2] < 0  # ลูกบาสเคลื่อนที่ลง
        passed_hoop = self.previous_z is not None and self.previous_z > hoop_position[2] and ball_pos[2] <= hoop_position[2]
        self.previous_z = ball_pos[2]
        return is_within_radius and is_moving_downward and passed_hoop
    
# สร้างลูกบาส
basketball_radius = 0.11925 
basketball_start_pos = [0, 0, 1.5] 
basketball_collision = p.createCollisionShape(p.GEOM_SPHERE, radius=basketball_radius)
basketball_visual = p.createVisualShape(p.GEOM_SPHERE, radius=basketball_radius, rgbaColor=[1, 0.5, 0, 1])
basketball_id = p.createMultiBody(
    baseMass=0.625,
    baseCollisionShapeIndex=basketball_collision,
    baseVisualShapeIndex=basketball_visual,
    basePosition=basketball_start_pos
)
# สร้างห่วงบาส
hoop_position = [x_position, 0, z_position]
hoop_radius = 0.225
hoop_height = 0.02
hoop_collision = p.createCollisionShape(p.GEOM_CYLINDER, radius=hoop_radius, height=hoop_height)
hoop_body = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=hoop_collision, basePosition=hoop_position)

# ตั้งค่าให้ลูกบาสและห่วงบาสไม่ชนกัน
p.setCollisionFilterPair(basketball_id, hoop_body, -1, -1, enableCollision=0)

# สร้างแป้นบาส
backboard_size = [0.1, 1.27, 0.9]
backboard_position = [x_position + 0.245, 0,z_position + 0.45]
backboard_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[s / 2 for s in backboard_size])
backboard_body = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=backboard_collision, basePosition=backboard_position)
p.changeDynamics(
    backboard_body, -1,
    restitution=0.7, 
    lateralFriction=0.5  
)
p.changeDynamics(
    basketball_id, -1,
    restitution=0.85,  
    lateralFriction=0.6  
)
p.changeDynamics(
    0, -1, 
    restitution=0.5, 
    lateralFriction=0.9  
)
# ตั้งค่าตำแหน่งของแต่ละ joint ให้เป็น home
for i in range(len(home)):
    p.resetJointState(robot_id, i, home[i])
    
def calculate_ee_velocity(robot_id, link_index):
    link_state = p.getLinkState(robot_id, link_index, computeLinkVelocity=1)
    linear_velocity = link_state[6]  # ความเร็วเชิงเส้น
    return linear_velocity

a=0
dt=0.01
end_effector_link_index = 3
# กำหนด PI parameters
kp = 1.05  
ki = 0.01 
previous_error = np.zeros(4)
integral = np.zeros(4)

ball_released = False
link_index = 3
offset_to_tip = [0.33925, 0, 0]

# สร้างตัวเก็บข้อมูล
position_history = []
velocity_history = []
for k in range(len(q_path)):
    Veff = p.getLinkState(robot_id, 3, computeLinkVelocity=1)
    if k<14 and ball_released == False:
        link_state = p.getLinkState(robot_id, link_index)
        ee_position = link_state[0] 
        ee_orientation = link_state[1] 
        # คำนวณตำแหน่งปลาย link สีเหลืองที่รวม Offset
        ball_position, ball_orientation = p.multiplyTransforms(
            ee_position, ee_orientation,  
            offset_to_tip, [0, 0, 0, 1] 
        )
        p.resetBasePositionAndOrientation(
            basketball_id, 
            ball_position,  
            ball_orientation  
        )
    elif k==14 :
        p.resetBaseVelocity(
            basketball_id,
            linearVelocity = [abs(Veff[6][0]),abs(Veff[6][1]),abs(Veff[6][2])] # ความเร็วในแกน x, y, z
        )
        print('ee_velocity =', [abs(Veff[6][0]),abs(Veff[6][1]),abs(Veff[6][2])])
        # อัปเดตสถานะลูกบาส
        ball_released = True
        
    # ใช้ POSITION_CONTROL สำหรับตำแหน่ง
    for i in range(4):
        p.setJointMotorControl2(
        bodyUniqueId=robot_id,
        jointIndex=i,
        controlMode=p.POSITION_CONTROL,
        targetPosition=q_path[k][i],
        force=100,       
    )
    Peff = p.getLinkState(robot_id, 3, computeLinkVelocity=1)
    rounded_tuple = tuple(round(x, 2) for x in Peff[0])
    for j in range(4):
        joint_state = p.getJointState(robot_id, j) 
        current_velocity = joint_state[1]  
        if j == 0:
            target_velocity = 0  
        else:
            target_velocity = joint_velocities[k][j-1]  
        # คำนวณ PI Error 
        error = target_velocity - current_velocity
        integral[j] += error  
        # คำนวณ PI control output
        pi_output = kp * error + ki * integral[j]
        previous_error[j] = error
        p.setJointMotorControl2(
            bodyIndex=robot_id,
            jointIndex=j,
            controlMode=p.VELOCITY_CONTROL,
            targetVelocity=pi_output,  # PI control output
            force=100
        )
    rounded_Veff= tuple(round(x, 2) for x in Veff[6])
    
    # ดึงตำแหน่งและความเร็วของ End-Effector
    link_state = p.getLinkState(robot_id, end_effector_link_index, computeLinkVelocity=1)
    ee_position = link_state[0] 
    linear_velocity = link_state[6]  # เป็น tuple (vx, vy, vz)
    vx, vy, vz = abs(linear_velocity[0]), abs(linear_velocity[1]), abs(linear_velocity[2])
    position_history.append(ee_position)
    velocity_history.append((vx, vy, vz))
    
    # link_state = p.getLinkState(robot_id, end_effector_link_index, computeLinkVelocity=1)
    # ee_position = link_state[0] 
    # ee_velocity = link_state[6]  
    # position_history.append(ee_position)
    # velocity_history.append(ee_velocity)

    p.stepSimulation()
    time.sleep(dt)


position_history = np.array(position_history)
velocity_history = np.array(velocity_history)

position_x = position_history[:, 0]
position_z = position_history[:, 2]

velocity_x = velocity_history[:, 0]
velocity_z = velocity_history[:, 2]

tracker = BallStateTracker()

for _ in range(1000):
    p.stepSimulation()
    time.sleep(1 / 240)
    
    if tracker.check_ball_through_transparent_hoop(basketball_id, hoop_position):
        print("The basketball went through the hoop!")
        break  
  
# Plot ตำแหน่งในระนาบ X-Z    
plt.figure(figsize=(8, 6))
plt.plot(position_x, position_z, label='End-Effector Trajectory', marker='o')
plt.scatter(position_x[0], position_z[0], color='green', label='Home', s=150)
plt.scatter(position_x[14], position_z[14], color='blue', label='Shoot', s=150)
plt.scatter(position_x[-1], position_z[-1], color='red', label='Rest', s=150)
plt.title('End-Effector Position in X-Z Plane')
plt.xlabel('X Position (m)')
plt.ylabel('Z Position (m)')
plt.legend()
plt.grid()
plt.tight_layout()
plt.show()

# สร้างกราฟความเร็วในระนาบ X-Z
plt.figure(figsize=(8, 6))
plt.plot(velocity_x, label='X Velocity')
plt.plot(velocity_z, label='Z Velocity')
plt.title('End-Effector Velocity in X-Z Plane')
plt.xlabel('Time Steps')
plt.ylabel('Velocity (m/s)')
plt.legend()
plt.grid()
plt.tight_layout()
plt.show()

# รอเพื่อดูผลลัพธ์
time.sleep(5)
p.disconnect()

