import time
import numpy as np
import matplotlib.pyplot as plt

class PIDController:
    def __init__(self, Kp, Ki, Kd, target):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.target = target

        self.errors = []
        self.last_error = 0
        self.integral = 0

    def update(self, current_value, dt):
        error = self.target - current_value
        self.errors.append(error)

        self.integral += error * dt
        derivative = (error - self.last_error) / dt
        self.last_error = error

        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

        return output

# 初始化PID控制器
target = 50
controller = PIDController(1.6, 0.9, 0.2, target)

# 初始化变量
current_value = 0
dt = 0.1
total_time = 10
num_steps = int(total_time / dt)
times = np.linspace(0, total_time, num_steps)
values = np.zeros(num_steps)

# 进行PID控制并记录输出值
for i in range(num_steps):
    output = controller.update(current_value, dt)
    current_value += output * dt
    values[i] = current_value

# 绘制PID控制器输出值和目标值的图表
plt.plot(times, values, label='Output')
plt.plot(times, [target] * num_steps, label='Target')
plt.legend()
plt.xlabel('Time (s)')
plt.ylabel('Value')
plt.title('PID Controller Output')
plt.show()

# 绘制误差随时间的变化图表
plt.plot(times, controller.errors)
plt.xlabel('Time (s)')
plt.ylabel('Error')
plt.title('PID Controller Error')
plt.show()
