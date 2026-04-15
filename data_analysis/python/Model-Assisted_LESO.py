import numpy as np
import matplotlib.pyplot as plt

# --- 1. 参数与带宽设置 ---
g = 9.81     
l = 1.0      
b0 = -1.0/l  

# 【修改1】根据 25Hz 视觉采样率，重设观测器带宽
w_o = 30.0   # 157 rad/s 的 1/10 左右，保证离散化系统的稳定性

beta1 = 3 * w_o
beta2 = 3 * w_o**2 - (g / l)
beta3 = w_o**3

# --- 2. 仿真与多速率环境设置 ---
dt_physics = 0.001   # 物理引擎步长 (1000Hz)
dt_imu = 0.005       # IMU和观测器运行步长 (200Hz) -> 对应 5 个物理步
dt_vision = 0.040    # 视觉信号刷新步长 (25Hz)  -> 对应 40 个物理步

T = 10.0     
t = np.arange(0, T, dt_physics)
n = len(t)

# --- 3. 状态变量初始化 ---
x = np.zeros((2, n))
x[0, 0] = 0.2  # 真实初始摆角

z = np.zeros((3, n)) # LESO 记录历史数据

# 外部输入与扰动
u = 2.0 * np.sin(1.5 * t)  
d = 0.5 * np.cos(3 * t) + 0.1 * np.random.randn(n)
f_true = np.zeros(n)

# 【修改2】定义传感器的零阶保持器 (ZOH) 变量
y_meas = x[0, 0]             # 视觉测量的当前值
u_meas = u[0]                # IMU测量的当前值
z_current = np.zeros(3)      # 飞控中实际维护的观测器当前状态

# --- 4. 仿真主循环 ---
for i in range(n-1):
    
    # 【A. 物理世界更新 - 1000Hz】
    theta_ddot = -(g/l) * np.sin(x[0, i]) - (u[i]/l) * np.cos(x[0, i]) + d[i]
    x[0, i+1] = x[0, i] + x[1, i] * dt_physics
    x[1, i+1] = x[1, i] + theta_ddot * dt_physics
    f_true[i] = -(g/l) * (np.sin(x[0, i]) - x[0, i]) - (u[i]/l) * (np.cos(x[0, i]) - 1.0) + d[i]
    
    # 【B. 传感器采样环节】
    # 视觉相机采样 - 25Hz (每 40 步刷新一次)
    if i % int(dt_vision / dt_physics) == 0:
        y_meas = x[0, i]
        
    # IMU采样及观测器运行 - 200Hz (每 5 步运行一次)
    if i % int(dt_imu / dt_physics) == 0:
        u_meas = u[i]
        
        # 计算观测误差 (利用滞后的视觉测量 y_meas)
        e = y_meas - z_current[0]  
        
        # LESO 连续方程的离散化执行
        z1_dot = z_current[1] + beta1 * e
        z2_dot = -(g/l) * z_current[0] + b0 * u_meas + z_current[2] + beta2 * e
        z3_dot = beta3 * e
        
        # 使用飞控循环时间 (0.005s) 更新状态
        z_current[0] = z_current[0] + z1_dot * dt_imu
        z_current[1] = z_current[1] + z2_dot * dt_imu
        z_current[2] = z_current[2] + z3_dot * dt_imu

    # 记录当前飞控内存中的 z 到数组中（模拟 1000Hz 日志记录系统）
    z[:, i+1] = z_current

f_true[-1] = -(g/l) * (np.sin(x[0, -1]) - x[0, -1]) - (u[-1]/l) * (np.cos(x[0, -1]) - 1.0) + d[-1]

# --- 5. 计算均方根误差 (RMSE) ---
# 为了客观评价“跟踪”性能，我们排除前 1.0 秒的初始收敛瞬态误差
eval_start_idx = int(1.0 / dt_physics)

rmse_theta = np.sqrt(np.mean((x[0, eval_start_idx:] - z[0, eval_start_idx:])**2))
rmse_omega = np.sqrt(np.mean((x[1, eval_start_idx:] - z[1, eval_start_idx:])**2))
rmse_f     = np.sqrt(np.mean((f_true[eval_start_idx:] - z[2, eval_start_idx:])**2))

print("=== LESO Tracking RMSE (t > 1.0s) ===")
print(f"Angle (theta) RMSE      : {rmse_theta:.4f} rad")
print(f"Angular Vel (omega) RMSE: {rmse_omega:.4f} rad/s")
print(f"Disturbance (f) RMSE    : {rmse_f:.4f}")

# --- 6. 绘图验证 ---
plt.figure(figsize=(10, 8), dpi=100)

plt.subplot(3, 1, 1)
plt.plot(t, x[0, :], label='True Angle $\\theta$ (1000Hz)', color='black')
plt.plot(t, z[0, :], '--', label='LESO Angle $\\hat{x}_1$ (200Hz Update, 25Hz Vision)', color='red', alpha=0.8)
plt.ylabel('Angle (rad)')
plt.title('Multi-Rate LESO State Estimation (Vision: 25Hz, IMU/Ctrl: 200Hz)')
plt.legend(loc='upper right')
plt.grid(True)

plt.subplot(3, 1, 2)
plt.plot(t, x[1, :], label='True Angular Vel $\\dot{\\theta}$', color='black')
plt.plot(t, z[1, :], '--', label='LESO Vel $\\hat{x}_2$', color='blue', alpha=0.8)
plt.ylabel('Angular Vel (rad/s)')
plt.legend(loc='upper right')
plt.grid(True)

plt.subplot(3, 1, 3)
plt.plot(t, f_true, label='True Lumped Disturbance $f(t)$', color='black', alpha=0.6)
plt.plot(t, z[2, :], '--', label='LESO Disturbance $\\hat{x}_3$', color='green', alpha=0.8)
plt.ylabel('Disturbance $f(t)$')
plt.xlabel('Time (s)')
plt.legend(loc='upper right')
plt.grid(True)

plt.tight_layout()
plt.show()