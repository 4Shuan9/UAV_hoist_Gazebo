import pandas as pd
import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import os

# ==========================================
# 配置区：在此修改文件名和路径
# ==========================================
CSV_FILE_PATH = '../csv/20260409_test1_Data.csv'  # 替换为你的文件名
START_TIME_OFFSET = 0  # 绘图起始时间偏移（秒），默认为0

# ==========================================
# 数据处理逻辑
# ==========================================
def process_drone_data(file_path):
    if not os.path.exists(file_path):
        print(f"Error: 文件 {file_path} 不存在。")
        return

    # 读取数据
    print(f"正在读取文件: {file_path} ...")
    df = pd.read_csv(file_path)

    # 定义列名（根据你的CSV格式）
    col_time = '__time'
    col_cv_x = '/uav/cv/swing_angle/cartesian/vector/x'
    col_cv_y = '/uav/cv/swing_angle/cartesian/vector/y'
    col_truth_x = '/uav/truth/swing_angle/cartesian/vector/x'
    col_truth_y = '/uav/truth/swing_angle/cartesian/vector/y'

    # 1. 提取视觉数据 (CV - 25Hz) 并去除空值
    cv_data = df[[col_time, col_cv_x, col_cv_y]].dropna().copy()
    
    # 2. 提取真实数据 (Truth - 200Hz) 并去除空值
    truth_data = df[[col_time, col_truth_x, col_truth_y]].dropna().copy()

    # 时间对齐：将时间归零化（从0开始）
    t0 = min(cv_data[col_time].min(), truth_data[col_time].min())
    cv_data['t'] = cv_data[col_time] - t0
    truth_data['t'] = truth_data[col_time] - t0

    # 3. 计算误差：由于频率不同，需将 Truth 插值到 CV 的时间戳上
    # 这样我们可以计算在每一个 CV 采样点处，它与真实值的偏差
    interp_truth_x = np.interp(cv_data['t'], truth_data['t'], truth_data[col_truth_x])
    interp_truth_y = np.interp(cv_data['t'], truth_data['t'], truth_data[col_truth_y])

    error_x = cv_data[col_cv_x].values - interp_truth_x
    error_y = cv_data[col_cv_y].values - interp_truth_y

    # 4. 计算 RMSE (均方根误差)
    rmse_x = np.sqrt(np.mean(error_x**2))
    rmse_y = np.sqrt(np.mean(error_y**2))

    print(f"--- 分析结果 ---")
    print(f"X方向 RMSE: {rmse_x:.4f}")
    print(f"Y方向 RMSE: {rmse_y:.4f}")

    # ==========================================
    # 可视化绘图 (Plotly)
    # ==========================================
    fig = make_subplots(
        rows=3, cols=1,
        shared_xaxes=True,
        vertical_spacing=0.08,
        subplot_titles=(
            f"X-Axis Comparison (RMSE: {rmse_x:.4f})", 
            f"Y-Axis Comparison (RMSE: {rmse_y:.4f})", 
            "Estimation Error (CV - Truth)"
        )
    )

    # Subplot 1: X Comparison
    fig.add_trace(go.Scatter(x=truth_data['t'], y=truth_data[col_truth_x], name="Truth X (200Hz)", line=dict(color='blue', width=1)), row=1, col=1)
    fig.add_trace(go.Scatter(x=cv_data['t'], y=cv_data[col_cv_x], name="Visual X (25Hz)", mode='lines+markers', marker=dict(size=4), line=dict(color='red', dash='dot')), row=1, col=1)

    # Subplot 2: Y Comparison
    fig.add_trace(go.Scatter(x=truth_data['t'], y=truth_data[col_truth_y], name="Truth Y (200Hz)", line=dict(color='green', width=1)), row=2, col=1)
    fig.add_trace(go.Scatter(x=cv_data['t'], y=cv_data[col_cv_y], name="Visual Y (25Hz)", mode='lines+markers', marker=dict(size=4), line=dict(color='orange', dash='dot')), row=2, col=1)

    # Subplot 3: Errors
    fig.add_trace(go.Scatter(x=cv_data['t'], y=error_x, name="Error X", line=dict(color='purple')), row=3, col=1)
    fig.add_trace(go.Scatter(x=cv_data['t'], y=error_y, name="Error Y", line=dict(color='brown')), row=3, col=1)

    # 更新布局
    fig.update_layout(
        height=900, 
        title_text=f"UAV Slung Load Swing Angle Analysis: {file_path}",
        showlegend=True,
        template="plotly_white",
        hovermode="x unified"
    )

    fig.update_xaxes(title_text="Time (s)", row=3, col=1)
    fig.update_yaxes(title_text="Angle (deg)", row=1, col=1)
    fig.update_yaxes(title_text="Angle (deg)", row=2, col=1)
    fig.update_yaxes(title_text="Error (deg)", row=3, col=1)

    # 显示图表（在浏览器打开，支持自由缩放）
    fig.show()

    # 如果需要保存为 HTML 文件
    # fig.write_html("swing_angle_analysis.html")

if __name__ == "__main__":
    process_drone_data(CSV_FILE_PATH)