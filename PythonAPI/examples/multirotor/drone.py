# ==============================================================================
# -- DroneMode -----------------------------------------------------------------
# ==============================================================================

import numpy as np
import math
import pygame

# from examples.generate_traffic_air import Trajectory

def minmax(val, range):
    if val>range[1]:
        val = range[1]
    if val<range[0]:
        val = range[0]
    return val

class Drone():
    #------------初始化机体位置和旋翼位置-----------
    com = np.array([0.0,0.0,0.0])
    prop1 = np.array([0.0,0.0,0.0])
    prop2 = np.array([0.0,0.0,0.0])
    prop3 = np.array([0.0,0.0,0.0])
    prop4 = np.array([0.0,0.0,0.0])
    #-----------初始化旋翼速度-----------------------
    r1 = 0.0
    r2 = 0.0
    r3 = 0.0
    r4 = 0.0
    #-----------初始化旋翼角---------------------
    theta = 0.0
    phi = 0.0
    psi = 0.0
    #-----------初始化 STATE VECTOR-----------------------
    state = np.zeros((12,))
    state_dot = np.zeros((12,))
    #-----------初始化参数---------------
    id = None
    mass = 1.2
    arm_length = 0.3
    r = 0.1
    moi_x = ((2*(3*mass/5)*r**2)/5)+(2*(mass/10)*arm_length**2)
    moi_y = moi_x
    moi_z = ((2*(3*mass/5)*r**2)/5)+(4*(mass/10)*arm_length**2)
    prop_pitch = 4.5
    prop_dia = 10.0
    thrust_fac = 4.392e-8 * (pow(prop_dia,3.5)/np.sqrt(prop_pitch)) * 4.23e-4 * prop_pitch
    drag_fac = thrust_fac*0.0245
    delta_t = 0.001
    g = 9.80665
    mode = "waypoint"
    last_v = 0
    trajectory = [] # 轨迹点
    start_point = None # 初始化点
    #----------PID 参数---------------------------------
    int_linear_err = {'X':0.0,'Y':0.0,'Z':0.0}
    int_angular_err = {'th':0.0,'ph':0.0,'ps':0.0}
    linear_pid = {'X':{'P':300.0,'I':0.04,'D':450.0},'Y':{'P':300.0,'I':0.04,'D':450.0},'Z':{'P':7000.0,'I':4.5,'D':5000.0}}
    angular_pid = {'th':{'P':22000.0,'I':0.0,'D':12000.0},'ph':{'P':22000.0,'I':0.0,'D':12000.0},'ps':{'P':1500.0,'I':1.2,'D':0.0}}
    tilt_limits = [-np.pi/3, np.pi/3]
    yaw_limits = [-800,800] 
    motor_speed_limits = [4000,10000]
    yaw_error_rate_scaler = 0.18
    
    def initialize(self):
        self.set_initial_state_vector()
        self.set_other_coordinates()
    
    def set_initial_state_vector(self):
        self.state[0] = self.com[0]
        self.state[1] = self.com[1]
        self.state[2] = self.com[2]
        self.state[6] = minmax(self.theta, self.tilt_limits)
        self.state[7] = minmax(self.phi, self.tilt_limits)
        self.state[8] = self.set_cyclic_angle(self.psi)

    def set_cyclic_angle(self,vec):
         return (vec+np.pi)%(2*np.pi) - np.pi

    def set_com(self):
        self.com = self.state[0:3]

    def get_orientation_vector(self):
        v1 = self.prop2 - self.com
        v2 = self.prop1 - self.com
        dir_vec = np.cross(v2,v1)
        return self.com,dir_vec

    def set_angles(self):
        self.theta = minmax(self.state[6],self.tilt_limits)
        self.phi =  minmax(self.state[7],self.tilt_limits)
        self.psi = self.state[8]

    def set_other_coordinates(self):
        rot_x = np.array([[1.0,0.0,0.0],[0.0,math.cos(self.theta),-1*math.sin(self.theta)],[0.0,math.sin(self.theta),math.cos(self.theta)]])
        rot_y = np.array([[math.cos(self.phi),0.0,math.sin(self.phi)],[0.0,1.0,0.0],[-1*math.sin(self.phi),0.0,math.cos(self.phi)]])
        rot_z = np.array([[math.cos(self.psi),-1*math.sin(self.psi),0.0],[math.sin(self.psi),math.cos(self.psi),0.0],[0.0,0.0,1.0]])
        rot_net = np.dot(rot_z,np.dot(rot_y,rot_x))
        self.prop2 = self.com-np.dot(rot_net, np.array([self.arm_length,0,0]))
        self.prop1 = self.com+np.dot(rot_net, np.array([0,self.arm_length,0]))
        self.prop4 = self.com+np.dot(rot_net, np.array([self.arm_length,0,0]))
        self.prop3 = self.com-np.dot(rot_net, np.array([0,self.arm_length,0]))

    def set_state_dots(self):
        self.set_angles()
        moi = np.array([[self.moi_x, 0.0, 0.0],[0.0, self.moi_y, 0.0],[0.0, 0.0, self.moi_z]])
        moi_inv = np.array([[1/self.moi_x, 0.0, 0.0],[0.0, 1/self.moi_y, 0.0],[0.0, 0.0, 1/self.moi_z]])
        rot_x = np.array([[1.0,0.0,0.0],[0.0,math.cos(self.theta),-1*math.sin(self.theta)],[0.0,math.sin(self.theta),math.cos(self.theta)]])
        rot_y = np.array([[math.cos(self.phi),0.0,math.sin(self.phi)],[0.0,1.0,0.0],[-1*math.sin(self.phi),0.0,math.cos(self.phi)]])
        rot_z = np.array([[math.cos(self.psi),-1*math.sin(self.psi),0.0],[math.sin(self.psi),math.cos(self.psi),0.0],[0.0,0.0,1.0]])
        rot_net = np.dot(rot_z,np.dot(rot_y,rot_x))
        self.state_dot[0] = self.state[3]
        self.state_dot[1] = self.state[4]
        self.state_dot[2] = self.state[5]
        X_double_dot = np.array([0.0,0.0,-1*self.g]) + (1/self.mass)*np.dot(rot_net,np.array([0.0,0.0,self.thrust_fac*(self.r1**2 + self.r2**2 + self.r3**2 + self.r4**2)]))
        self.state_dot[3] = X_double_dot[0]
        self.state_dot[4] = X_double_dot[1]
        self.state_dot[5] = X_double_dot[2]
        self.state_dot[6] = self.state[9]
        self.state_dot[7] = self.state[10]
        self.state_dot[8] = self.state[11]
        omega = self.state[9:12]

        tau = np.array([self.arm_length*self.thrust_fac*(self.r1**2-self.r3**2), self.arm_length*self.thrust_fac*(self.r2**2-self.r4**2), self.drag_fac*(self.r1**2-self.r2**2+self.r3**2-self.r4**2)])

        omega_dot = np.dot(moi_inv, (tau - np.cross(omega, np.dot(moi,omega))))
        self.state_dot[9] = omega_dot[0]
        self.state_dot[10] = omega_dot[1]
        self.state_dot[11] = omega_dot[2]

    def dynamics_euler_angles(self, delta_t):
        self.set_state_dots()
        self.state += delta_t*self.state_dot
        # self.state += self.delta_t*self.state_dot
        self.state[6:9] = self.set_cyclic_angle(self.state[6:9])
        self.state[2] = max(0,self.state[2])
        self.set_angles()
        self.set_com()
        self.set_other_coordinates()
        result = []
        result.extend(list(self.com))
        result.extend(list(self.prop1))
        result.extend(list(self.prop2))
        result.extend(list(self.prop3))
        result.extend(list(self.prop4))
        return result
    
    def controller_update(self,x_des,y_des,z_des,yaw_des):
        x_error = x_des - self.state[0]
        y_error = y_des - self.state[1]
        z_error = z_des - self.state[2]
        self.int_linear_err['X'] += x_error
        self.int_linear_err['Y'] += y_error
        self.int_linear_err['Z'] += z_error
        change_wrt_x = self.linear_pid['X']['P']*x_error - self.linear_pid['X']['D']*(self.state[3]) + self.linear_pid['X']['I'] * self.int_linear_err['X']
        change_wrt_y = self.linear_pid['Y']['P']*y_error - self.linear_pid['Y']['D']*(self.state[4]) + self.linear_pid['Y']['I'] * self.int_linear_err['Y']
        change_wrt_z = self.linear_pid['Z']['P']*z_error - self.linear_pid['Z']['D']*(self.state[5]) + self.linear_pid['Z']['I'] * self.int_linear_err['Z']
        dest_theta = change_wrt_x*math.sin(self.state[8]) - change_wrt_y*math.cos(self.state[8])
        dest_phi = change_wrt_x*math.cos(self.state[8]) + change_wrt_y*math.sin(self.state[8])
        dest_theta = minmax(dest_theta, self.tilt_limits)
        dest_phi = minmax(dest_phi, self.tilt_limits)
        theta_error = dest_theta - self.state[6]
        phi_error = dest_phi - self.state[7]
        psi_error = yaw_des - self.state[8]
        psi_dot_error = self.yaw_error_rate_scaler*self.set_cyclic_angle(psi_error) - self.state[11]
        self.int_angular_err['th'] += theta_error
        self.int_angular_err['ph'] += phi_error
        self.int_angular_err['ps'] += psi_dot_error
        change_wrt_th = self.angular_pid['th']['P']*theta_error - self.angular_pid['th']['D']*self.state[9] + self.angular_pid['th']['I'] * self.int_angular_err['th']
        change_wrt_ph = self.angular_pid['ph']['P']*phi_error - self.angular_pid['ph']['D']*self.state[10] + self.angular_pid['ph']['I'] * self.int_angular_err['ph']
        change_wrt_ps = self.angular_pid['ps']['P']*psi_dot_error + self.angular_pid['ps']['I'] * self.int_angular_err['ps']
        change_wrt_ps = minmax(change_wrt_ps, self.yaw_limits)
        self.r1 += change_wrt_z + change_wrt_ps + change_wrt_th
        self.r2 += change_wrt_z - change_wrt_ps + change_wrt_ph
        self.r3 += change_wrt_z + change_wrt_ps - change_wrt_th
        self.r4 += change_wrt_z - change_wrt_ps - change_wrt_ph
        self.r1 = minmax(self.r1, self.motor_speed_limits)
        self.r2 = minmax(self.r2, self.motor_speed_limits)
        self.r3 = minmax(self.r3, self.motor_speed_limits)
        self.r4 = minmax(self.r4, self.motor_speed_limits)

    def debug(self):
        print("{} {} {} {}".format(self.r1, self.r2, self.r3, self.r4))

    def trajectory_motion(self):
        if self.trajectory[0] is None: # TODO：可能存在隐患
            print("飞行器{}路径点为空！".format(self.id))

    def manual_update(self, thrust_input, roll_input, pitch_input, yaw_input, delta_t, hz):
        # 计算推力
        thrust = thrust_input

        # state[0:9] x y z roll_speed pitch_speed yaw_speed roll pitch yaw
        #            0 1 2  3            4            5       6    7    8


        x = self.state[0]
        y = self.state[1]
        z = self.state[2]
        pitch_speed = self.state[4]
        pitch = self.state[7]
        roll = self.state[6]
        roll_speed = self.state[3]
        yaw = self.state[8]
        yaw_speed = self.state[5]
        Kp_pitch = self.angular_pid['ph']['P']
        Kd_pitch = self.angular_pid['ph']['D']
        Kp_roll = self.angular_pid['th']['P']
        Kd_roll = self.angular_pid['th']['D']
        Kp_yaw = self.angular_pid['ps']['P']
        Kd_yaw = self.angular_pid['ps']['D']
        

        # 计算俯仰角
        pitch_error = pitch_input - self.state[7]
        pitch_speed += (Kp_pitch * pitch_error - Kd_pitch * pitch_speed) * delta_t
        pitch += pitch_speed * delta_t

        # 计算横滚角
        roll_error = roll_input - roll
        roll_speed += (Kp_roll * roll_error - Kd_roll * roll_speed) * delta_t
        roll += roll_speed * delta_t

        # 计算偏航角
        yaw_error = yaw_input - yaw
        yaw_speed += (Kp_yaw * yaw_error - Kd_yaw * yaw_speed) * delta_t
        yaw += yaw_speed * delta_t

        # 计算位置和速度
        gravity = 9.81
        acceleration = (thrust / self.mass)  - gravity
        cur_v = acceleration*delta_t       

        # 悬停模式
        if thrust == 11.772:
            self.last_v = 0

        # 考虑空气阻力导致的最大速度限制
        if (cur_v +self.last_v)*hz > 10:   # 最大爬升速度 10m/s
            self.last_v = (10/hz)
            z += (10/hz)
        elif (cur_v +self.last_v)*hz < -15: # 最大跌落速度 15m/s
            z -= (15/hz)
            self.last_v = (-15/hz)
        else:
            z += (cur_v +self.last_v)
            self.last_v += cur_v


        y += math.sin(roll) * cur_v
        x += math.sin(pitch) * cur_v

        # 位于地面不累计向下加速度
        if z <= 0:
            self.last_v = 0

        print(self.last_v*hz)

        self.state[0] = x 
        self.state[1] = y
        self.state[2] = z
        self.state[4] = pitch_speed
        self.state[7] = pitch
        self.state[6] = roll
        self.state[3] = roll_speed
        self.state[8] = yaw
        self.state[5] = yaw_speed

        self.state[6:9] = self.set_cyclic_angle(self.state[6:9])
        self.state[2] = max(0,self.state[2])
        self.set_angles()





