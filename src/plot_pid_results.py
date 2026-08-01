"""
Part 2: PID 导航结果可视化脚本
================================
读取 pid_controller 记录的 CSV，生成两张图：
1. 轨迹图：实际 (x,y) 轨迹 + 起点/终点标记
2. 误差曲线：距离 ρ 与合速度 v 随时间收敛

使用方法：
    python plot_pid_results.py pid_log_t2.csv
    python plot_pid_results.py pid_log_t2.csv --title "测试2：纯横移"
"""

import sys
import os
import argparse
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
import numpy as np


# ---------- 中文字体配置 ----------
def setup_cjk_font():
    """配置 matplotlib 支持中文显示"""
    font_names = ['Microsoft YaHei', 'SimHei', 'SimSun',
                  'Noto Sans CJK SC', 'WenQuanYi Micro Hei']
    available = [f.name for f in matplotlib.font_manager.fontManager.ttflist]
    for name in font_names:
        if name in available:
            plt.rcParams['font.sans-serif'] = [name] + plt.rcParams['font.sans-serif']
            plt.rcParams['axes.unicode_minus'] = False
            return name
    print("警告：未找到中文字体，中文可能显示为方框")
    return None


def plot_trajectory(df, target_x, target_y, title, save_path):
    """绘制轨迹图"""
    fig, ax = plt.subplots(figsize=(8, 6))

    # 实际轨迹
    ax.plot(df['x'], df['y'], 'b-', linewidth=2, label='实际轨迹', zorder=2)

    # 起点和终点
    start_x, start_y = df['x'].iloc[0], df['y'].iloc[0]
    ax.scatter(start_x, start_y, c='green', s=150, marker='o',
               label=f'起点 ({start_x:.2f}, {start_y:.2f})', zorder=3, edgecolors='black')
    ax.scatter(target_x, target_y, c='red', s=150, marker='*',
               label=f'目标点 ({target_x:.2f}, {target_y:.2f})', zorder=3, edgecolors='black')

    # 终点位置
    end_x, end_y = df['x'].iloc[-1], df['y'].iloc[-1]
    ax.scatter(end_x, end_y, c='orange', s=100, marker='s',
               label=f'终点 ({end_x:.2f}, {end_y:.2f})', zorder=3, edgecolors='black')

    # 理想直线（起点到目标的直线）
    ax.plot([start_x, target_x], [start_y, target_y],
            'r--', linewidth=1.5, alpha=0.5, label='理想路径', zorder=1)

    ax.set_xlabel('X (m)', fontsize=12)
    ax.set_ylabel('Y (m)', fontsize=12)
    ax.set_title(f'{title} - 轨迹图', fontsize=14, fontweight='bold')
    ax.legend(loc='best', fontsize=10)
    ax.set_aspect('equal', adjustable='box')
    ax.grid(True, alpha=0.3)

    # 添加箭头表示运动方向（取轨迹中段）
    mid = len(df) // 2
    if mid > 0 and mid < len(df) - 1:
        dx = df['x'].iloc[mid + 5] - df['x'].iloc[mid]
        dy = df['y'].iloc[mid + 5] - df['y'].iloc[mid]
        if dx != 0 or dy != 0:
            ax.annotate('', xy=(df['x'].iloc[mid] + dx, df['y'].iloc[mid] + dy),
                        xytext=(df['x'].iloc[mid], df['y'].iloc[mid]),
                        arrowprops=dict(arrowstyle='->', color='blue', lw=2))

    plt.tight_layout()
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    print(f'轨迹图已保存: {save_path}')
    plt.close()


def plot_error_curve(df, title, save_path):
    """绘制误差曲线图（距离 + 速度）"""
    fig, ax1 = plt.subplots(figsize=(10, 6))

    # 左轴：距离误差
    color1 = '#1a365d'
    ax1.set_xlabel('时间 (s)', fontsize=12)
    ax1.set_ylabel('距离误差 ρ (m)', color=color1, fontsize=12)
    ax1.plot(df['t'], df['rho'], color=color1, linewidth=2, label='距离误差 ρ')
    ax1.tick_params(axis='y', labelcolor=color1)
    ax1.axhline(y=0.05, color='gray', linestyle=':', alpha=0.7, label='到达容差 (0.05m)')
    ax1.grid(True, alpha=0.3)

    # 右轴：合速度
    ax2 = ax1.twinx()
    color2 = '#2b6cb0'
    ax2.set_ylabel('合速度 v (m/s)', color=color2, fontsize=12)
    v_total = np.sqrt(df['vx'] ** 2 + df['vy'] ** 2)
    ax2.plot(df['t'], v_total, color=color2, linewidth=1.5, alpha=0.7, label='合速度 v')
    ax2.tick_params(axis='y', labelcolor=color2)

    # 合并图例
    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, labels1 + labels2, loc='upper right', fontsize=10)

    ax1.set_title(f'{title} - 误差与速度曲线', fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    print(f'误差曲线已保存: {save_path}')
    plt.close()


def plot_velocity_components(df, title, save_path):
    """绘制分速度曲线（vx, vy, w）"""
    fig, ax = plt.subplots(figsize=(10, 5))

    ax.plot(df['t'], df['vx'], 'r-', linewidth=1.5, label='vx (前向)')
    ax.plot(df['t'], df['vy'], 'g-', linewidth=1.5, label='vy (横向)')
    ax.plot(df['t'], df['w'], 'b-', linewidth=1.5, label='ω (角速度)')

    ax.set_xlabel('时间 (s)', fontsize=12)
    ax.set_ylabel('速度', fontsize=12)
    ax.set_title(f'{title} - 分速度曲线', fontsize=14, fontweight='bold')
    ax.legend(loc='best', fontsize=10)
    ax.grid(True, alpha=0.3)
    ax.axhline(y=0, color='k', linewidth=0.5)

    plt.tight_layout()
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    print(f'分速度曲线已保存: {save_path}')
    plt.close()


def main():
    parser = argparse.ArgumentParser(description='PID 导航结果可视化')
    parser.add_argument('csv_file', help='CSV 数据文件路径')
    parser.add_argument('--target_x', type=float, default=None,
                        help='目标点 X（默认用 CSV 末尾推断）')
    parser.add_argument('--target_y', type=float, default=None,
                        help='目标点 Y（默认用 CSV 末尾推断）')
    parser.add_argument('--title', default='PID 点到点导航',
                        help='图表标题前缀')
    args = parser.parse_args()

    # 配置中文字体
    setup_cjk_font()

    # 读取数据
    df = pd.read_csv(args.csv_file)
    print(f'读取 {len(df)} 条数据，时长 {df["t"].iloc[-1]:.2f}s')

    # 推断目标点（如果未指定，用到达时刻的位置近似）
    target_x = args.target_x if args.target_x is not None else df['x'].iloc[-1]
    target_y = args.target_y if args.target_y is not None else df['y'].iloc[-1]

    # 生成保存路径（与 CSV 同目录）
    base_dir = os.path.dirname(os.path.abspath(args.csv_file))
    base_name = os.path.splitext(os.path.basename(args.csv_file))[0]

    # 绘制三张图
    plot_trajectory(df, target_x, target_y, args.title,
                    os.path.join(base_dir, f'{base_name}_trajectory.png'))
    plot_error_curve(df, args.title,
                     os.path.join(base_dir, f'{base_name}_error.png'))
    plot_velocity_components(df, args.title,
                             os.path.join(base_dir, f'{base_name}_velocity.png'))

    # 打印统计信息
    print('\n========== 统计信息 ==========')
    print(f'起点: ({df["x"].iloc[0]:.3f}, {df["y"].iloc[0]:.3f})')
    print(f'终点: ({df["x"].iloc[-1]:.3f}, {df["y"].iloc[-1]:.3f})')
    print(f'目标: ({target_x:.3f}, {target_y:.3f})')
    print(f'初始距离: {df["rho"].iloc[0]:.3f} m')
    print(f'最终误差: {df["rho"].iloc[-1]:.3f} m')
    print(f'总时长: {df["t"].iloc[-1]:.2f} s')
    print(f'最大速度: {np.sqrt(df["vx"]**2 + df["vy"]**2).max():.3f} m/s')
    print(f'平均速度: {np.sqrt(df["vx"]**2 + df["vy"]**2).mean():.3f} m/s')


if __name__ == '__main__':
    main()
