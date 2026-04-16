import pandas as pd
import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import os

# ==========================================
# 配置区：在此修改文件名和路径
# ==========================================
CSV_FILE_PATH = '/home/ashuang/4Shuan9_ws/UAV_hoist_Gazebo/data_analysis/csv/20260416_test1_Data.csv'  # 替换为你的目标CSV文件
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

    # 定义目标列名（只关注 Y 轴相关的列和 truth）
    col_time = '__time'
    col_truth_y = '/uav/truth/swing_angle/world_frame/cartesian/vector/y'
    col_eso_y = '/uav/eso/swing_angle/world_frame/cartesian/vector/y'
    col_eso_fusion_y = '/uav/eso_fusion/swing_angle/world_frame/cartesian/vector/y'
    col_cv_y = '/uav/cv/swing_angle/world_frame/cartesian/vector/y'

    # 1. 提取各个观测源数据并去除空值（独立处理以适应多采样率）
    truth_data = df[[col_time, col_truth_y]].dropna().copy()
    eso_data = df[[col_time, col_eso_y]].dropna().copy()
    eso_fusion_data = df[[col_time, col_eso_fusion_y]].dropna().copy()
    cv_data = df[[col_time, col_cv_y]].dropna().copy()

    # 2. 时间对齐：将时间归零化（从0开始）
    t0 = min(truth_data[col_time].min(), eso_data[col_time].min(), 
             eso_fusion_data[col_time].min(), cv_data[col_time].min())
    
    truth_data['t'] = truth_data[col_time] - t0
    eso_data['t'] = eso_data[col_time] - t0
    eso_fusion_data['t'] = eso_fusion_data[col_time] - t0
    cv_data['t'] = cv_data[col_time] - t0

    # 3. 计算误差：由于频率不同，我们将 Truth 数据分别插值到各个观测源的时间戳上
    interp_truth_for_eso = np.interp(eso_data['t'], truth_data['t'], truth_data[col_truth_y])
    interp_truth_for_eso_fusion = np.interp(eso_fusion_data['t'], truth_data['t'], truth_data[col_truth_y])
    interp_truth_for_cv = np.interp(cv_data['t'], truth_data['t'], truth_data[col_truth_y])

    # 偏差 = 估计值 - 真实值
    error_eso = eso_data[col_eso_y].values - interp_truth_for_eso
    error_eso_fusion = eso_fusion_data[col_eso_fusion_y].values - interp_truth_for_eso_fusion
    error_cv = cv_data[col_cv_y].values - interp_truth_for_cv

    # 4. 计算指标: RMSE (均方根误差) & 均值
    rmse_eso = np.sqrt(np.mean(error_eso**2))
    mean_eso = np.mean(error_eso)
    
    rmse_eso_fusion = np.sqrt(np.mean(error_eso_fusion**2))
    mean_eso_fusion = np.mean(error_eso_fusion)
    
    rmse_cv = np.sqrt(np.mean(error_cv**2))
    mean_cv = np.mean(error_cv)

    # 终端输出结果 (参考 Model-Assisted LESO)
    print(f"--- 稳态跟踪 Y轴方向误差 ---")
    print(f"ESO:        RMSE = {rmse_eso:.4f}, Mean Error = {mean_eso:.4f}")
    print(f"ESO Fusion: RMSE = {rmse_eso_fusion:.4f}, Mean Error = {mean_eso_fusion:.4f}")
    print(f"CV:         RMSE = {rmse_cv:.4f}, Mean Error = {mean_cv:.4f}")

    # ==========================================
    # 可视化绘图 (Plotly)
    # ==========================================
    # 生成包含全局 RMSE 信息的标题
    title_str = (f"UAV Swing Angle Y-Axis Comparison<br>"
                 f"ESO: RMSE={rmse_eso:.4f} | "
                 f"ESO_Fusion: RMSE={rmse_eso_fusion:.4f} | "
                 f"CV: RMSE={rmse_cv:.4f}")

    fig = make_subplots(
        rows=2, cols=1,
        shared_xaxes=True,
        vertical_spacing=0.08,
        subplot_titles=(
            "Y-Axis Estimation vs Truth", 
            "Estimation Errors (Estimate - Truth)"
        )
    )

    # Subplot 1: Y 数据对比
    fig.add_trace(go.Scatter(x=truth_data['t'], y=truth_data[col_truth_y], name="Truth Y", line=dict(color='black', width=2)), row=1, col=1)
    fig.add_trace(go.Scatter(x=eso_data['t'], y=eso_data[col_eso_y], name="ESO Y", line=dict(color='blue', width=1, dash='dash')), row=1, col=1)
    fig.add_trace(go.Scatter(x=eso_fusion_data['t'], y=eso_fusion_data[col_eso_fusion_y], name="ESO Fusion Y", line=dict(color='red', width=1.5, dash='dashdot')), row=1, col=1)
    fig.add_trace(go.Scatter(x=cv_data['t'], y=cv_data[col_cv_y], name="CV Y", mode='lines+markers', marker=dict(size=4), line=dict(color='orange', dash='dot')), row=1, col=1)

    # Subplot 2: 误差对比
    fig.add_trace(go.Scatter(x=eso_data['t'], y=error_eso, name="Error ESO", line=dict(color='blue', width=1)), row=2, col=1)
    fig.add_trace(go.Scatter(x=eso_fusion_data['t'], y=error_eso_fusion, name="Error ESO Fusion", line=dict(color='red', width=1.5)), row=2, col=1)
    fig.add_trace(go.Scatter(x=cv_data['t'], y=error_cv, name="Error CV", line=dict(color='orange', width=1)), row=2, col=1)

    # 更新布局
    fig.update_layout(
        height=800, 
        title_text=title_str,
        showlegend=True,
        template="plotly_white",
        hovermode="x unified"
    )

    fig.update_xaxes(title_text="Time (s)", row=2, col=1)
    fig.update_yaxes(title_text="Angle Y (deg / rad)", row=1, col=1)
    fig.update_yaxes(title_text="Error Y", row=2, col=1)

    # 显示图表（在浏览器打开，支持自由缩放）
    fig.show()

    # 如果需要保存为 HTML 文件可以取消以下注释
    # fig.write_html("swing_angle_analysis_Y.html")

if __name__ == "__main__":
    process_drone_data(CSV_FILE_PATH)