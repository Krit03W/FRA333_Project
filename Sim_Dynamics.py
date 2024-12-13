import pybullet as p
import pybullet_data
import time
import numpy as np
import math
from scipy.optimize import fsolve
from roboticstoolbox import DHRobot, PrismaticDH, RevoluteDH
from math import pi
from sympy import symbols, Matrix, sin, cos, diff
from spatialmath import SE3
from game_input import get_hoop_position

# รับค่าตำแหน่งห่วงบาสจากผู้ใช้โดยเรียกใช้ Pygame
x_position, z_position = get_hoop_position()

# สร้างโมเดลหุ่นยนต์ด้วย DH Parameters
robot = DHRobot([
    PrismaticDH(a=0, alpha=0, theta=0, qlim=[0, 2]),  # ข้อต่อ Prismatic (d1)
    RevoluteDH(a=0, alpha=pi/2, d=0, qlim=[np.radians(0), np.radians(10)]),  # ข้อต่อ Revolute (theta2)
    RevoluteDH(a=0.4, alpha=0, d=0, qlim=[np.radians(50), np.radians(120)]),  # ข้อต่อ Revolute (theta3)
    RevoluteDH(a=0.34, alpha=0, d=0, qlim=[np.radians(-20), np.radians(20)])  # ข้อต่อ Revolute (theta4)
], name="Basketball_Shooting_Robot")
# กำหนด Tool Transformation สำหรับ End-Effector
robot.tool = SE3.Tx(0.22)

n_joints = robot.n
print('จำนวน Joint',n_joints)

# ท่า home, shooting และ rest
home = [0.58, np.radians(10), np.radians(120), np.radians(20)]
shooting = [1.4, np.radians(5), np.radians(70), np.radians(-10)]
# shooting = [1.4, np.radians(21.25), np.radians(101.01), np.radians(-120.17)]
rest = [1.4, np.radians(0), np.radians(50), np.radians(-20)]
#------------------------------------------------------------------------------------------------

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
    
# คำนวณค่า joint angles ทุกๆ path จาก home ไป shoot
q_path = interpolate_joint_angles(home, shooting, rest, n_paths)
        
rounded_array = np.round(q_path, 2)
# print('Num_q_path :',len(q_path))
print('q_path 1 :',rounded_array[:51])
print('q_path 2 :',rounded_array[51:])

# ตั้งค่าตำแหน่งเริ่มต้นของลูกบาสและห่วงบาส
basketball_start_pos = [0, 0, 2.73]  # ตำแหน่งเริ่มต้นของลูกบาส (x, y, z)
hoop_position = [7.239, 0, 3.05]  # ตำแหน่งห่วงบาส (x, y, z)

def calculate_velocity(basketball_start_pos, hoop_position):
    g = 9.81  # ความเร่งเนื่องจากแรงโน้มถ่วง (m/s^2)
    # คำนวณระยะทางในแนว XY
    distance_xy = np.sqrt((hoop_position[0] - basketball_start_pos[0])**2 + 
                          (hoop_position[1] - basketball_start_pos[1])**2)
    # ความสูงเริ่มต้นและเป้าหมายในแนว z
    z_start = basketball_start_pos[2]
    z_hoop = hoop_position[2]
    # คำนวณเวลาในการขึ้นไปถึงห่วงบาส (สมการ z = z_0 + v_0z * t - 0.5 * g * t^2)
    time_to_reach_hoop = math.sqrt(2 * (z_hoop - z_start) / g)
    # คำนวณความเร็วในแนวระนาบ
    v0 = distance_xy / time_to_reach_hoop
    print(f'Vo : {v0}')
    return v0, time_to_reach_hoop

# print(f'Vo : {v0}')
# เรียกใช้ฟังก์ชันเพื่อคำนวณค่า v0 และเวลา
v0, time_to_reach_hoop = calculate_velocity(basketball_start_pos, hoop_position)

# Parameters
x_start = 0.18 # Start position in x-axis (m)
x_end = 0.62    # End position in x-axis (m)
t_total = 1.0  # Total time (s)
t1, t2, t3 = 0.3, 0.7, 0.3  # Key time points (s)
dt = 0.01      # Time step (s)
m_basketball = 0.625  # Mass of basketball (kg)
r = 0.22       # Distance (m) for torque calculation

# Time array
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

# Acceleration Profile (derivative of velocity)
acceleration = np.gradient(velocity, dt)

# Position Profile (integral of velocity)
position = np.cumsum(velocity) * dt
position = x_start + (x_end - x_start) * (position / position[-1])

# Calculate Force and Torque at each time step
forces = m_basketball * acceleration

# Path in x-axis
x_path = position

# ฟังก์ชันคำนวณ Jacobian Matrix
def calculate_jacobian(robot, q):
    J = robot.jacob0(q)  # Jacobian matrix ใน base frame
    return J[0:3, 1:]  # ใช้เฉพาะส่วน Linear Velocity (3x4 matrix)

def compute_torque(J, force_vec):
    J_T = J.T  # Transpose ของ Jacobian
    tau = np.dot(J_T, force_vec)  # คำนวณ Torque
    # print('Torque: ', tau)
    return tau

# print('Jacobial Return :',calculate_jacobian(robot,[1,1,1,1]))
# เก็บผลลัพธ์ Joint Velocities
joint_velocities = []
torque = []
# print(f'Velocity : {velocity}')
for i, vx in enumerate(velocity):  # velocity = Linear Velocity ของ End-Effector
    # Linear Velocity ของ End-Effector
    eff_velocity = np.array([vx, 0, 0])  # [v_x, v_y, v_z]
    
    # Jacobian Matrix ที่ตำแหน่ง q_input
    J = calculate_jacobian(robot, q_path[i])
    
    # ตรวจสอบว่า Jacobian เป็น Square Matrix
    if J.shape[0] != J.shape[1]:
        raise ValueError(f"Jacobian is not square at step {i}: shape={J.shape}")
    
    # ตรวจสอบว่า Jacobian เป็น Invertible
    if np.linalg.det(J) == 0.01:
        raise ValueError(f"Jacobian is singular at step {i}")

    # Inverse ของ Jacobian
    J_inverse = np.linalg.inv(J)
    
    # คำนวณ Joint Velocities
    q_dot = np.dot(J_inverse, eff_velocity)  # dot product ของ J^-1 กับ eff_velocity
    force_vec = np.array([forces[i], 0, 0])
    tau = compute_torque(J, force_vec)
    joint_velocities.append(q_dot)  # บันทึกค่าความเร็วของ Joint
    torque.append(tau.tolist())
    # print('qdottt', q_dot)
    # print('Jacobian', J)

print("Torque : ",torque)
# print("Joint : ",torque)
# compute_torque(J, forces)
    
# แปลงผลลัพธ์เป็น numpy array
joint_velocities = np.array(joint_velocities)
print('joint_velocities',joint_velocities)

# เริ่มต้น PyBullet
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")  # พื้น
p.setGravity(0, 0, -9.81)

# ตั้งค่ากล้อง
cameraDistance = 12.0
cameraYaw = 0
cameraPitch = -10
cameraTarget = [7, 0, 2]
p.resetDebugVisualizerCamera(cameraDistance, cameraYaw, cameraPitch, cameraTarget)

# # โหลดโมเดล URDF ของหุ่นยนต์
robot_id = p.loadURDF("basketball_robot.urdf", [0, 0, 0], useFixedBase=True)

# ท่า home, shooting และ rest
home = [1.4, np.radians(10), np.radians(120), np.radians(20)]
shooting = [1.4, np.radians(5), np.radians(70), np.radians(-10)]
# shooting = [1.4, np.radians(21.25), np.radians(101.01), np.radians(-120.17)]
rest = [1.4, np.radians(0), np.radians(50), np.radians(-20)]

class BallStateTracker:
    def __init__(self):
        self.previous_z = None  # ความสูงของลูกบาสก่อนหน้า

    def check_ball_through_transparent_hoop(self, ball_body, hoop_position, max_distance=0.3):
        # รับตำแหน่งและความเร็วลูกบาส
        ball_pos, _ = p.getBasePositionAndOrientation(ball_body)
        velocity = p.getBaseVelocity(ball_body)[0]  # ความเร็วลูกบาส (linear velocity)

        # ตรวจสอบระยะห่างในแกน X, Y
        ball_xy = np.array(ball_pos[:2])
        hoop_xy = np.array(hoop_position[:2])
        distance = np.linalg.norm(ball_xy - hoop_xy)

        # ตรวจสอบเงื่อนไข
        is_within_radius = distance <= max_distance  # ลูกบาสอยู่ในรัศมีห่วง
        is_moving_downward = velocity[2] < 0  # ลูกบาสเคลื่อนที่ลง
        passed_hoop = self.previous_z is not None and self.previous_z > hoop_position[2] and ball_pos[2] <= hoop_position[2]

        # อัปเดตตำแหน่ง z ก่อนหน้า
        self.previous_z = ball_pos[2]

        # ลูกบาสต้องผ่านระดับความสูงของห่วงลงมา
        return is_within_radius and is_moving_downward and passed_hoop
    
# สร้างลูกบาส
basketball_radius = 0.11925  # รัศมีของลูกบาส
basketball_start_pos = [0, 0, 1.5]  # ตำแหน่งเริ่มต้น
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

# เพิ่มคุณสมบัติฟิสิกส์ให้แป้นบาส
p.changeDynamics(
    backboard_body, -1,
    restitution=0.7,  # ความยืดหยุ่นของแป้นบาส
    lateralFriction=0.5  # แรงเสียดทาน
)

# เพิ่มคุณสมบัติฟิสิกส์ให้ลูกบาส
p.changeDynamics(
    basketball_id, -1,
    restitution=0.85,  # ความยืดหยุ่นของลูกบาส
    lateralFriction=0.6  # แรงเสียดทาน
)

# เพิ่มคุณสมบัติฟิสิกส์ให้พื้น
p.changeDynamics(
    0, -1,  # bodyUniqueId = 0 สำหรับพื้น
    restitution=0.5,  # ความยืดหยุ่นของพื้น
    lateralFriction=0.9  # แรงเสียดทาน
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

# สมมติว่า robot_id คือ ID ของหุ่นยนต์ และ End-Effector คือ link สุดท้ายของหุ่นยนต์
end_effector_link_index = 3  # ตัวอย่าง link ที่ 3 (ปรับตามโครงสร้างหุ่นยนต์ของคุณ)

# ตัวแปรสำหรับเก็บค่า error
previous_error = np.zeros(4)
integral = np.zeros(4)

#Dynamics __________________
def qddot(a, b, c): # q_path, q_dot, tau
    # สร้างตัวแปร
    q1, q2, q3, q4 = symbols('q1 q2 q3 q4')  # Generalized coordinates
    q1_dot, q2_dot, q3_dot, q4_dot = symbols('q1_dot q2_dot q3_dot q4_dot')  # Velocities
    g, m1, m2, m3, m4 = symbols('9.81 1 1 1 1')  # Parameters
    tau1, tau2, tau3, tau4 = symbols('tau1 tau2 tau3 tau4')  # Torques
    a2, a3, a4 = symbols('0.4 0.34 0.22')  # Link lengths

    # ตัวอย่าง Inertia Matrix (M)
    M = Matrix([
        [1 + m1 + m2 + m3 + m4, 0, 0, 0],
        [0, 1 + m2 * a2**2, 0, 0],
        [0, 0, 1 + m3 * a3**2, 0],
        [0, 0, 0, 1 + m4 * a4**2]
    ])

    # ตัวอย่าง Coriolis Matrix (C)
    C = Matrix([
        [0, 0, 0, 0],
        [0, 0.5 * m2 * a2, 0, 0],
        [0, 0, 0.5 * m3 * a3, 0],
        [0, 0, 0, 0.5 * m4 * a4]
    ])

    # ตัวอย่าง Gravitational Vector (G)
    G = Matrix([
        [0],
        [g * m2 * sin(q2)],
        [g * m3 * sin(q3)],
        [g * m4 * sin(q4)]
    ])
    q_dot = Matrix([q1_dot, q2_dot, q3_dot, q4_dot])
    # print(q_dot)
    # Torque Vector
    tau = Matrix([tau1, tau2, tau3, tau4])

    # print(tau)
    # คำนวณความเร่ง (q_ddot)
    q_ddot = M.inv() * (tau - C * q_dot - G)

    # print("ความเร่งของข้อต่อ (q_ddot):")
    # for i in range(len(q_ddot)):
    #     print(f'สมการ q{i+1} : {q_ddot[i]}\n')
    
    # แทนค่าตัวเลขใน q_ddot_solution
    numerical_q_ddot = q_ddot.subs({
    q1: a[0], q2: a[1], q3: a[2], q4: a[3],
    q1_dot: 0 , q2_dot: b[0], q3_dot:b[1], q4_dot:b[2],
    tau1: 0, tau2: c[0], tau3: c[1], tau4: c[2],
    m1: 1, m2: 1, m3: 1, m4: 1,
    g: 9.81, a2: 0.4, a3: 0.34, a4: 0.22
    })

    return numerical_q_ddot.tolist()
    # แสดงผลลัพธ์ของ q_ddot ที่แทนค่าตัวเลข
    # print("ผลลัพธ์ของ q_ddot (ค่าเป็นตัวเลข):")
    # print(numerical_q_ddot)
#_______________
# a=[1,1,1,1]
# b=[3,3,3]
# c=[5,5,5]
# qddot(a, b, c)

# ฟังก์ชัน PID controller
def pid_control(setpoint, actual_position, previous_error, kp, ki, kd, dt):
    error = setpoint - actual_position
    integral = previous_error + error * dt
    derivative = (error - previous_error) / dt
    output = kp * error + ki * integral + kd * derivative
    return output, error, integral
kp, ki, kd = 10.0, 5.0, 20.0
previous_error = 1.0
integral = 2.0

q_ddotAll = []
for i in range(len(torque)):
    q_dotdot = qddot(q_path[i], joint_velocities[i], torque[i])
    print(f'q_dotdot {i} : {q_dotdot}')
    # q_ddotAll.append(q_dotdot)

# print("q_ddotAll :" , q_ddotAll)

ball_released = False
link_index = 3
offset_to_tip = [0.33925, 0, 0]
# Control robot with velocity mode
for k in range(len(q_path)):
    Veff = p.getLinkState(robot_id, 3, computeLinkVelocity=1)
    # Peff = p.getLinkState(robot_id, link_index)
    # print(Peff)
    if k<56 and ball_released == False:
        link_state = p.getLinkState(robot_id, link_index)  # link_index คือ link สีเหลือง
        ee_position = link_state[0]  # ตำแหน่ง XYZ ของ End Effector (ศูนย์กลาง link สีเหลือง)
        ee_orientation = link_state[1]  # การหมุนของ link สีเหลือง (quaternion)

        # คำนวณตำแหน่งปลาย link สีเหลืองที่รวม Offset
        ball_position, ball_orientation = p.multiplyTransforms(
            ee_position, ee_orientation,  # ตำแหน่งและการหมุนของ link สีเหลือง
            offset_to_tip, [0, 0, 0, 1]  # Offset สำหรับปลาย link
        )
        # ย้ายลูกบาสไปยังตำแหน่งที่ปลายของ link สีเหลือง
        p.resetBasePositionAndOrientation(
            basketball_id,  # ID ของลูกบาส
            ball_position,  # ตำแหน่งที่ปลาย link สีเหลือง
            ball_orientation  # การหมุนที่เหมือนกับ link สีเหลือง
        )
        
    elif k==56 :
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
    )
    Peff = p.getLinkState(robot_id, 3, computeLinkVelocity=1)
    rounded_tuple = tuple(round(x, 2) for x in Peff[0])
    
    # for j in range(4):
    #     joint_state = p.getJointState(robot_id, j)  # ค่าของตำแหน่งและความเร็ว
    #     current_velocity = joint_state[1]  # ดึงค่าความเร็ว (index 1)
        
    #     # คำนวณข้อผิดพลาด (Error)
    #     if j == 0:
    #         target_velocity = 0  # ข้อต่อแรก (อาจเป็น prismatic joint)
            
    #     else:
    #         # target_velocity = joint_velocities[k][j-1]  # ค่าความเร็วจากข้อมูล
    #         target_velocity = joint_velocities[k][j-1]  # ค่าความเร็วจากข้อมูล

    #     # คำนวณ PI Error (ไม่คำนวณ Derivative)
    #     error = target_velocity - current_velocity  # ความผิดพลาด (target - actual)
    #     integral[j] += error  # คำนวณค่า integral

    #     # คำนวณ PI control output (P + I)
    #     pi_output = kp * error + ki * integral[j]

    #     # อัปเดตค่าของ previous error สำหรับใช้ในขั้นตอนถัดไป
    #     previous_error[j] = error
        
    #     # กำหนดค่า PI control output ไปยัง setJointMotorControl2
    #     p.setJointMotorControl2(
    #         bodyIndex=robot_id,
    #         jointIndex=j,
    #         controlMode=p.VELOCITY_CONTROL,
    #         targetVelocity=pi_output,  # PI control output
    #         force=100
    #     )
    
    rounded_Veff= tuple(round(x, 2) for x in Veff[6])
    # print(f"Peff {k}:",(rounded_tuple)) 
    print(f'Veff {k} :', rounded_Veff)



    # for i in range(4):
    #     if i == 0:
    #         Torque_target=0,  # ระบุทอร์คที่ต้องการ
    #     else:      
    #         Torque_target=torque[k][i-1]  

    #     p.setJointMotorControl2(
    #     bodyUniqueId=robot_id,
    #     jointIndex=i,
    #     controlMode=p.TORQUE_CONTROL,  # ใช้ TORQUE_CONTROL แทน POSITION_CONTROL
    #     targetTorque=Torque_target,  # ระบุทอร์คที่ต้องการ
    # )



    for i in range(4):
        if i == 0:
            p.setJointMotorControl2(
                bodyUniqueId=robot_id,
                jointIndex=i,
                controlMode=p.TORQUE_CONTROL,
                force=0  # ใช้ torque ที่คำนวณได้จาก PID
            )
        else:
            joint_state = p.getJointState(robot_id, i)
            actual_position = joint_state[0]  # joint position
            setpoint = q_path[k][i]  # ตำแหน่งที่ต้องการให้จอยท์ไปถึง

            # คำนวณค่า Torque โดยใช้ PID
            torque, previous_error, integral = pid_control(setpoint, actual_position, previous_error, kp, ki, kd, dt)

            # ควบคุม Torque
            p.setJointMotorControl2(
                bodyUniqueId=robot_id,
                jointIndex=i,
                controlMode=p.TORQUE_CONTROL,
                force=torque  # ใช้ torque ที่คำนวณได้จาก PID
            )

    p.stepSimulation()
    time.sleep(dt)

# เริ่มต้นคลาสติดตามสถานะลูกบาส
tracker = BallStateTracker()

# จำลองการเคลื่อนที่ต่อไปเพื่อดูผลลัพธ์
for _ in range(1000):
    p.stepSimulation()
    time.sleep(1 / 240)
    
    if tracker.check_ball_through_transparent_hoop(basketball_id, hoop_position):
        print("The basketball went through the hoop!")
        break  # หยุดการจำลองเมื่อลูกบาสเข้าห่วง

# รอเพื่อดูผลลัพธ์
time.sleep(5)
p.disconnect()

# while True:
#     # K_spring_id = p.addUserDebugParameter("K Constant", 0, 1000, 100)
#     # k_spring = p.readUserDebugParameter(K_spring_id)
#     # link_state = p.getLinkState(robot_id, 3, computeLinkVelocity=1)
#     # print(link_state[6])
#     p.stepSimulation()
#     time.sleep(dt)