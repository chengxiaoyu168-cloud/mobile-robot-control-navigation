# 提交文件清单

这是我用于提交移动机器人控制与导航作业的 GitHub 仓库文件夹。仓库根目录保留 `README.md`，便于老师打开仓库后直接查看环境配置、运行命令、核心代码说明、实验结果和已知限制。

## 已包含内容

- `README.md`：GitHub 版实验报告，包含 Part 1 至 Part 5 的任务说明、方案、核心代码、运行结果和结论。
- `docs/README.docx`：排版后的 Word 版最终报告。
- `docs/Mobile_Robot_Assignment.md`：作业要求的文字整理版，便于对照检查提交内容。
- `src/`：各部分核心 Python 源码，包括 PID 点到点导航、轨迹跟踪、视觉目标跟踪、ROS MCP 服务和演示客户端等。
- `ros_package/`：从 ROS2 工作区整理出的包结构文件，包括 `setup.py`、`package.xml`、launch、URDF、world 文件、消息定义和节点源码。
- `results/`：实验过程中记录的 CSV 数据和轨迹/误差图。
- `videos/`：按 Part 重新命名后的演示视频文件。

## 视频命名说明

- `Part1_Webots_keyboard_control_demo.mp4`：Webots 仿真启动与键盘控制演示。
- `Part2_or_Part3_navigation_test_extra_demo.mp4`：点到点导航或轨迹跟踪补充演示。
- `Part4_ArUco_visual_following_demo.mp4`：ArUco 视觉目标跟踪演示。
- `Part5_follower_point_and_trajectory_demo.mp4`：Part 5 中跟随者点到点导航与轨迹跟踪演示。
- `Part5_target_following_and_chinese_command_demo.mp4`：Part 5 中动态目标跟踪与中文自然语言控制演示。

## 未放入内容

- 汇报 PPT：该文件仅用于课堂/导师汇报，不作为代码仓库提交内容。
- 旧版 README 草稿和临时 Word 文件：最终仓库只保留正式报告。
- 与本机器人作业无关的个人介绍材料。
- ROS2 本地 `build/`、`install/`、`log/` 目录以及调试过程中的原始日志：这些文件应在本地重新生成或通过 README 中的截图/视频复现。

## 使用建议

我将该文件夹作为 GitHub 仓库根目录使用，确保 `README.md` 位于仓库首页位置。

