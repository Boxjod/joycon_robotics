import math
import time
import matplotlib
matplotlib.use('TkAgg')  
import matplotlib.pyplot as plt
from glm import vec2, vec3, quat, angleAxis, eulerAngles
from pyjoycon import GyroTrackingJoyCon, get_R_id, get_L_id, ButtonEventJoyCon, JoyCon

from scipy.spatial.transform import Rotation as R
import numpy as np
import genesis as gs


class LowPassFilter:
    def __init__(self, alpha=0.1):
        self.alpha = alpha
        self.prev_value = 0.0

    def update(self, new_value):
        self.prev_value = self.alpha * new_value + (1 - self.alpha) * self.prev_value
        return self.prev_value
    
class AttitudeEstimator:
    def __init__(self):
        self.pitch = 0.0  # 倾斜角（俯仰角）
        self.roll = 0.0   # 滚转角
        self.yaw = 0.0    # 偏航角
        self.dt = 0.001    # 时间步长（秒），可以根据采样频率调整
        self.alpha = 0.5 # 滤波器常数
        
        self.direction_X = vec3(1, 0, 0)
        self.direction_Y = vec3(0, 1, 0)
        self.direction_Z = vec3(0, 0, 1)
        self.direction_Q = quat()
        
        self.lpf_roll = LowPassFilter(alpha=0.50)    # 滤波器用于滚转角，越大响应越快
        self.lpf_pitch = LowPassFilter(alpha=0.50)   # 滤波器用于俯仰角，越大响应越快
    
    def reset_yaw(self):
        self.direction_X = vec3(1, 0, 0)
        self.direction_Y = vec3(0, 1, 0)
        self.direction_Z = vec3(0, 0, 1)
        self.direction_Q = quat()
    
    def set_yaw(self,data):
        self.yaw = data
    
    def update(self, gyro_in_rad, accel_in_g):
        self.pitch = 0.0  # 倾斜角（俯仰角）
        self.roll = 0.0   # 滚转角
        
        ax, ay, az = accel_in_g
        ax = ax * math.pi
        ay = ay * math.pi
        az = az * math.pi
        
        gx, gy, gz = gyro_in_rad

        # 计算加速度计提供的俯仰角和滚转角
        roll_acc = math.atan2(ay, -az)
        pitch_acc = math.atan2(ax, math.sqrt(ay**2 + az**2))
        
        # 利用陀螺仪数据更新角度
        self.pitch += gy * self.dt
        self.roll -= gx * self.dt

        # 互补滤波器：加权融合加速度计和陀螺仪的数据
        self.pitch = self.alpha * self.pitch + (1 - self.alpha) * pitch_acc
        self.roll = self.alpha * self.roll + (1 - self.alpha) * roll_acc
        
        # 最终输出的滚转角和俯仰角再进行低通滤波
        self.pitch = self.lpf_pitch.update(self.pitch)
        self.roll = self.lpf_roll.update(self.roll)
        
        # 偏航角（通过陀螺仪更新）
        rotation = angleAxis(gx * (-1/86), self.direction_X) \
            * angleAxis(gy * (-1/86), self.direction_Y) \
            * angleAxis(gz * (-1/86), self.direction_Z)

        self.direction_X *= rotation
        self.direction_Y *= rotation
        self.direction_Z *= rotation
        self.direction_Q *= rotation        
        
        self.yaw = -self.direction_X[1] * math.pi/2  # 偏航角
        # 返回滚转角、俯仰角、偏航角（单位为弧度）
        return self.roll, -self.pitch, self.yaw

def ConnectJoycon(device):
    if device == "right":
        joycon_id = get_R_id()
    elif device == "left":
        joycon_id = get_L_id()
    else:
        print("get a wrong device name of joycon")
    print(f'{joycon_id=}')
    
    joycon_gyro = GyroTrackingJoyCon(*joycon_id)
    joycon_button = ButtonEventJoyCon(*joycon_id)
    joycon = JoyCon(*joycon_id)
    
    attitude_estimator = AttitudeEstimator()

    # 矫正和初始化
    print('正在矫正手柄姿态，请水平放置在桌面上...')
    joycon_gyro.calibrate()
    joycon_gyro.reset_orientation
    attitude_estimator.reset_yaw()
    time.sleep(2)
    joycon_gyro.calibrate()
    joycon_gyro.reset_orientation
    attitude_estimator.reset_yaw()
    time.sleep(2)
    print('手柄初始化完成！')
    return joycon_gyro, joycon_button, joycon, attitude_estimator





# class JoyconRobot:
#     def __init__(self, devices="right"):
#         # device = ["right","double","left"]
#         self.devices = devices
#         # self.device_len = len(self.devices)
      
#         if "right" in self.devices:
#           # connect to the right Joycon
#           joycon_id_r = get_R_id()
#           print(f'{joycon_id_r=}')
#           joycon_gyro_r = GyroTrackingJoyCon(*joycon_id_r)
#           joycon_button_r = ButtonEventJoyCon(*joycon_id_r)
#           joycon_r = JoyCon(*joycon_id_r)
          
#         # elif "double" in self.devices:
            
        
#         # self.if_single = if_single
        
        
#         self.pitch = 0.0  # 倾斜角（俯仰角）
#         self.roll = 0.0   # 滚转角
#         self.yaw = 0.0    # 偏航角
#         self.dt = 0.1    # 时间步长（秒），可以根据采样频率调整
#         self.alpha = 0.01 # 滤波器常数
        
#         self.direction_X = vec3(1, 0, 0)
#         self.direction_Y = vec3(0, 1, 0)
#         self.direction_Z = vec3(0, 0, 1)
#         self.direction_Q = quat()
        
#         self.lpf_roll = LowPassFilter(alpha=0.99)    # 滤波器用于滚转角，越大响应越快
#         self.lpf_pitch = LowPassFilter(alpha=0.99)   # 滤波器用于俯仰角，越大响应越快




# # 获取JoyCon ID并初始化GyroTrackingJoyCon
# joycon_id_r = get_R_id()
# print(f'{joycon_id_r=}')
# joycon_gyro_r = GyroTrackingJoyCon(*joycon_id_r)
# joycon_button_r = ButtonEventJoyCon(*joycon_id_r)
# joycon_r = JoyCon(*joycon_id_r)

# joycon_id_l = get_L_id()
# print(f'{joycon_id_l=}')
# joycon_gyro_l = GyroTrackingJoyCon(*joycon_id_l)
# joycon_button_l = ButtonEventJoyCon(*joycon_id_l)
# joycon_l = JoyCon(*joycon_id_l)

# # 初始化数据容器
# direction_data_r = [[], [], []]  # 分为三个子数据（假设direction包含三个值）
# direction_data_l = [[], [], []]  # 分为三个子数据（假设direction包含三个值）
# attitude_estimator_r = AttitudeEstimator()
# attitude_estimator_l = AttitudeEstimator()

# # 矫正和初始化
# joycon_gyro_r.calibrate()
# joycon_gyro_r.reset_orientation
# attitude_estimator_r.reset_yaw()
# joycon_gyro_l.calibrate()
# joycon_gyro_l.reset_orientation
# attitude_estimator_l.reset_yaw()
# time.sleep(2)
# joycon_gyro_r.calibrate()
# joycon_gyro_r.reset_orientation
# attitude_estimator_r.reset_yaw()
# joycon_gyro_l.calibrate()
# joycon_gyro_l.reset_orientation
# attitude_estimator_l.reset_yaw()
# time.sleep(2)

# x_r, y_r, z_r = right_zero_pos[0], right_zero_pos[1], right_zero_pos[2]
# x0_r, y0_r, z0_r = right_zero_pos[0], right_zero_pos[1], right_zero_pos[2]
# roll0_r, pitch0_r, yaw0_r = right_zero_euler[0], right_zero_euler[1], right_zero_euler[2]
# gripper_state_r = 1

# x_l, y_l, z_l = left_zero_pos[0], left_zero_pos[1], left_zero_pos[2]
# x0_l, y0_l, z0_l = left_zero_pos[0], left_zero_pos[1], left_zero_pos[2]
# roll0_l, pitch0_l, yaw0_l = left_zero_euler[0], left_zero_euler[1], left_zero_euler[2]
# gripper_state_l = 1

# while(True):
#     ############# 计算RP #############
#     attitude_estimator_value_r = attitude_estimator_r.update(joycon_gyro_r.gyro_in_rad[0], joycon_gyro_r.accel_in_g[0])
#     roll_r, pitch_r, yaw_r = attitude_estimator_value_r[0], attitude_estimator_value_r[1], attitude_estimator_value_r[2]
#     pitch_r = -pitch_r 
#     if pitch_r > 0:
#         pitch_r = pitch_r * 3.0
#     yaw_r = -yaw_r * math.pi * 10
#     yaw_rad_T = math.pi/2
#     pitch_rad_T = math.pi/2
#     pitch_r = pitch_rad_T if pitch_r > pitch_rad_T else (-pitch_rad_T if pitch_r < -pitch_rad_T else pitch_r) 
#     yaw_r = yaw_rad_T if yaw_r > yaw_rad_T else (-yaw_rad_T if yaw_r < -yaw_rad_T else yaw_r) 
#     attitude_estimator_value_l = attitude_estimator_l.update(joycon_gyro_l.gyro_in_rad[0], joycon_gyro_l.accel_in_g[0])
#     roll_l, pitch_l, yaw_l = attitude_estimator_value_l[0], attitude_estimator_value_l[1], attitude_estimator_value_l[2]
    
#     pitch_l = -pitch_l 
#     if pitch_l > 0:
#         pitch_l = pitch_l * 3.0
#     yaw_l = -yaw_l * math.pi * 10
#     pitch_l = pitch_rad_T if pitch_l > pitch_rad_T else (-pitch_rad_T if pitch_l < -pitch_rad_T else pitch_l) 
#     yaw_l = yaw_rad_T if yaw_l > yaw_rad_T else (-yaw_rad_T if yaw_l < -yaw_rad_T else yaw_l) 

#     direction_vector_r = vec3(math.cos(pitch_r) * math.cos(yaw_r), math.cos(pitch_r) * math.sin(yaw_r), math.sin(pitch_r))
#     direction_vector_l = vec3(math.cos(pitch_l) * math.cos(yaw_l), math.cos(pitch_l) * math.sin(yaw_l), math.sin(pitch_l))
    
#     ########### 复位/夹爪按键 #############
#     for event_type, status in joycon_button_r.events():
#         if event_type == 'plus' and status == 1:
#             joycon_gyro_r.calibrate()
#             joycon_gyro_r.reset_orientation
#             attitude_estimator_r.reset_yaw()
#             joycon_gyro_l.calibrate()
#             joycon_gyro_l.reset_orientation
#             attitude_estimator_l.reset_yaw()
#             time.sleep(2)
#             joycon_gyro_r.calibrate()
#             joycon_gyro_r.reset_orientation
#             attitude_estimator_r.reset_yaw()
#             joycon_gyro_l.calibrate()
#             joycon_gyro_l.reset_orientation
#             attitude_estimator_l.reset_yaw()
#             time.sleep(2)
#         if event_type == 'r':
#             if status == 1:
#                 gripper_state_r = -0.1
#             if status == 0:
#                 gripper_state_r = 1
                
#     for event_type, status in joycon_button_l.events():
#         if event_type == 'minus' and status == 1:
#             joycon_gyro_l.calibrate()
#             joycon_gyro_l.reset_orientation
#             attitude_estimator_l.reset_yaw()
#             joycon_gyro_r.calibrate()
#             joycon_gyro_r.reset_orientation
#             attitude_estimator_r.reset_yaw()
#             time.sleep(2)
#             joycon_gyro_l.calibrate()
#             joycon_gyro_l.reset_orientation
#             attitude_estimator_l.reset_yaw()
#             joycon_gyro_r.calibrate()
#             joycon_gyro_r.reset_orientation
#             attitude_estimator_r.reset_yaw()
#             time.sleep(2)
#         if event_type == 'l':
#             if status == 1:
#                 gripper_state_l = -0.1
#             if status == 0:
#                 gripper_state_l = 1
    
#     if joycon_r.get_button_home() == 1 or joycon_l.get_button_capture() == 1:
#         joycon_gyro_r.reset_orientation
#         attitude_estimator_r.reset_yaw()
#         while 1:
#             ################right
#             x_r = x_r - 0.002 if x_r > right_zero_pos[0]+0.002 else (x_r + 0.002 if x_r < right_zero_pos[0]-0.002 else x_r) 
#             y_r = y_r - 0.002 if y_r > right_zero_pos[1]+0.002 else (y_r + 0.002 if y_r < right_zero_pos[1]-0.002 else y_r)
#             z_r = z_r - 0.002 if z_r > right_zero_pos[2]+0.002 else (z_r + 0.002 if z_r < right_zero_pos[2]-0.002 else z_r)
#             right_target_pos = np.array([x_r, y_r, z_r])
#             r = R.from_euler('xyz', [pitch_r,  roll_r, yaw0_r], degrees=False).as_matrix()
#             rotation_matrix = R.from_euler('y', np.pi/2).as_matrix()
#             r = r @ rotation_matrix
#             quaternion = R.from_matrix(r).as_quat() # r.as_quat()  # 返回的是 [x, y, z, w]
#             right_target_quat = np.array([quaternion[3], quaternion[0], quaternion[1], quaternion[2]])
            
#             next_qpos = so_100_right.inverse_kinematics(
#                 link= right_end_effector,
#                 pos = right_target_pos,
#                 quat = right_target_quat
#             )
#             next_qpos[-1] = gripper_state_r
#             so_100_right.control_dofs_position(next_qpos, right_joint_idx)
#             scene.step()
#             if abs(x_r-right_zero_pos[0]) < 0.05 and abs(y_r-right_zero_pos[1]) < 0.05 and abs(z_r-right_zero_pos[2]) <0.05:
#                 break
#             ############left
#         while 1:
#             x_l = x_l - 0.002 if x_l > left_zero_pos[0]+0.002 else (x_l + 0.002 if x_l < left_zero_pos[0]-0.002 else x_l) 
#             y_l = y_l - 0.002 if y_l > left_zero_pos[1]+0.002 else (y_l + 0.002 if y_l < left_zero_pos[1]-0.002 else y_l)
#             z_l = z_l - 0.002 if z_l > left_zero_pos[2]+0.002 else (z_l + 0.002 if z_l < left_zero_pos[2]-0.002 else z_l)
#             left_target_pos = np.array([x_l, y_l, z_l])
#             r = R.from_euler('xyz', [pitch_l,  roll_l, yaw0_l], degrees=False).as_matrix()
#             rotation_matrix = R.from_euler('y', np.pi/2).as_matrix()
#             r = r @ rotation_matrix
#             quaternion = R.from_matrix(r).as_quat() # r.as_quat()  # 返回的是 [x, y, z, w]
#             left_target_quat = np.array([quaternion[3], quaternion[0], quaternion[1], quaternion[2]])

#             next_qpos = so_100_left.inverse_kinematics(
#                 link= left_end_effector,
#                 pos = left_target_pos,
#                 quat = left_target_quat
#             )
#             next_qpos[-1] = gripper_state_l
#             so_100_left.control_dofs_position(next_qpos, left_joint_idx)
#             if abs(x_l-left_zero_pos[0]) < 0.05 and abs(y_l-left_zero_pos[1]) < 0.05 and abs(z_l-left_zero_pos[2]) <0.05:
#                     break
#             scene.step()
            
#         joycon_gyro_r.calibrate()
#         joycon_gyro_r.reset_orientation
#         attitude_estimator_r.reset_yaw()
#         time.sleep(2)
    
    
#     ########### 位移 #############
#     joycon_stick_v_r = joycon_r.get_stick_right_vertical()
#     if joycon_stick_v_r > 4000: # 向前移动：朝着方向矢量的方向前进 0.1 的速度
#         x_r += 0.04 #* direction_vector_r[0]
#         z_r -= 0.01 * direction_vector_r[2]
#     elif joycon_stick_v_r < 1000: # 向后移动：朝着方向矢量的反方向移动 0.1 的速度
#         x_r -= 0.04 #* direction_vector_r[0]
#         z_r += 0.01 * direction_vector_r[2]

#     joycon_stick_v_l = joycon_l.get_stick_left_vertical()
#     if joycon_stick_v_l > 4000:# 向前移动：朝着方向矢量的方向前进 0.1 的速度
#         x_l -= 0.04 #* direction_vector_l[0]
#         z_l -= 0.01 * direction_vector_l[2]

#     elif joycon_stick_v_l < 1000: # 向后移动：朝着方向矢量的反方向移动 0.1 的速度
#         x_l += 0.04 #* direction_vector_l[0]
#         z_l -= 0.01 * direction_vector_l[2]
        
#     rotation_matrix_h = np.array([[0, -1, 0], [1, 0, 0],[0, 0, 1]])
#     joycon_stick_v_r = joycon_r.get_stick_right_horizontal()
#     direction_vector_hr = np.dot(rotation_matrix_h, direction_vector_r)
#     if joycon_stick_v_r > 4000: # 向前移动：朝着方向矢量的方向前进 0.1 的速度
#         # x_r += 0.01 * direction_vector_hr[0]
#         y_r += 0.01 * direction_vector_hr[1]
#     elif joycon_stick_v_r < 1000: # 向后移动：朝着方向矢量的反方向移动 0.1 的速度
#         # x_r -= 0.01 * direction_vector_hr[0]
#         y_r -= 0.01 * direction_vector_hr[1]

#     joycon_stick_v_l = joycon_l.get_stick_left_horizontal()
#     direction_vector_hl = np.dot(rotation_matrix_h, direction_vector_r)
#     if joycon_stick_v_l > 4000:# 向前移动：朝着方向矢量的方向前进 0.1 的速度
#         # x_l += 0.01 * direction_vector_hl[0]
#         y_l += 0.01 * direction_vector_hl[1]
#     elif joycon_stick_v_l < 1000: # 向后移动：朝着方向矢量的反方向移动 0.1 的速度
#         # x_l -= 0.01 * direction_vector_hl[0]
#         y_l -= 0.01 * direction_vector_hl[1]

#     joycon_button_b = joycon_r.get_button_b()
#     joycon_button_x = joycon_r.get_button_x()
#     if joycon_button_x == 1:
#         z_r += 0.01
#     elif joycon_button_b == 1:
#         z_r -= 0.01 
        
#     joycon_button_up = joycon_l.get_button_up()
#     joycon_button_down = joycon_l.get_button_down()
#     if joycon_button_up == 1:
#         z_l -= 0.01
#     elif joycon_button_down == 1:
#         z_l += 0.01 
        
#     # 暂停一段时间，模拟实时数据获取
#     print(f'{x_r:.3f}, {y_r:.3f}, {z_r:.3f}, {roll_r:.3f}, {pitch_r:.3f}, {yaw_r:.3f}')
#     print(f'{x_l:.3f}, {y_l:.3f}, {z_l:.3f}, {roll_l:.3f}, {pitch_l:.3f}, {yaw_l:.3f}')
    
#     z_r = 0.86 if z_r < 0.86 else (1.0 if z_r > 1.0 else z_r )
#     z_l = 0.86 if z_l < 0.86 else (1.0 if z_l > 1.0 else z_l )
    
#     # x_r = x0_r if x_r<x0_r else x_r
#     # x_l = x0_l if x_l<x0_l else x_l
    
#     right_target_pos = np.array([x_r, y_r, z_r])
#     r = R.from_euler('yxz', [roll_r, pitch_r, yaw_r], degrees=False).as_matrix()
#     rotation_matrix = R.from_euler('y', np.pi/2).as_matrix()
#     r = r @ rotation_matrix
#     # rotation_matrix = R.from_euler('x', -np.pi/3).as_matrix()
#     # r = r @ rotation_matrix
#     quaternion = R.from_matrix(r).as_quat()  # 返回的是 [x, y, z, w]
#     right_target_quat = np.array([quaternion[3], quaternion[0], quaternion[1], quaternion[2]])
#     next_qpos = so_100_right.inverse_kinematics(link= right_end_effector,pos = right_target_pos,quat = right_target_quat)
#     next_qpos[-1] = gripper_state_r
#     # next_qpos[0] = torch.tensor(yaw_r)
#     so_100_right.control_dofs_position(next_qpos, right_joint_idx)
    
    
#     left_target_pos = np.array([x_l, y_l, z_l])
#     r = R.from_euler('yxz', [roll_l, pitch_l, yaw_l], degrees=False).as_matrix()
#     rotation_matrix = R.from_euler('y', np.pi/2).as_matrix()
#     r = r @ rotation_matrix
#     # rotation_matrix = R.from_euler('x', -np.pi/3).as_matrix()
#     # r = r @ rotation_matrix
#     quaternion = R.from_matrix(r).as_quat()  # 返回的是 [x, y, z, w]
#     left_target_quat = np.array([quaternion[3], quaternion[0], quaternion[1], quaternion[2]])
#     next_qpos = so_100_left.inverse_kinematics(link=left_end_effector, pos=left_target_pos, quat=left_target_quat)
#     next_qpos[-1] = gripper_state_l
#     # next_qpos[0] = torch.tensor(yaw_l)
#     so_100_left.control_dofs_position(next_qpos, left_joint_idx)
    
#     for i in range(5):
#         scene.step()
        
#     left_new_pos = np.array(left_end_effector.get_pos().cpu())
#     left_new_quat = np.array(left_end_effector.get_quat().cpu())
#     yaw0_l = left_new_quat[2]
#     x_l, y_l, _ = left_new_pos[0], left_new_pos[1], left_new_pos[2]
    
#     right_new_pos = np.array(right_end_effector.get_pos().cpu())
#     right_new_quat = np.array(right_end_effector.get_quat().cpu())
#     yaw0_r = right_new_quat[2]
#     x_r, y_r, _ = right_new_pos[0], right_new_pos[1], right_new_pos[2]
    
# follower_arm.disconnect()

