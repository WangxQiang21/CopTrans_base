# 论文与项目链接索引

本地 PDF 均放在 [`papers/`](papers/)。论文页面与 PDF 文件分开列出；页面打不开或需要订阅时，可使用本地副本。

## 与单车多模式规划直接相关

| 论文 | 本地 PDF | 正文 / DOI 页面 | 代码、数据 | 获取/核验状态 |
|---|---|---|---|---|
| Bao et al., *Hybrid A* Path Planning with Multi-Modal Motion Extension for Four-Wheel Steering Mobile Robots* (2025, arXiv v3) | `06_Wang_4WIS_HybridAstar_2025.pdf` | [arXiv v3 HTML](https://arxiv.org/html/2509.06115v3) · [PDF](https://arxiv.org/pdf/2509.06115v3) | 本轮未找到公开实现 | 本地 PDF 已有；以 v3 为准 |
| Bao et al., *From Multi-Modal Paths to Executable Trajectories: A Trajectory Planning Framework for 4WIS Robots* (2026) | `From_Multi-Modal_Paths_to_Executable_Trajectories_4WIS.pdf` | [arXiv 摘要](https://arxiv.org/abs/2608.29108) · [HTML](https://arxiv.org/html/2608.29108v1) · [PDF](https://arxiv.org/pdf/2608.29108) | 本轮未找到公开实现 | 本地 PDF 已有 |
| Zhu et al., *Adaptive Multi-Mode Path Planning for Four-Wheel Independent Steering Vehicles* (2026) | `Zhu_Adaptive_Multi-Mode_Path_Planning_4WIS_2026.pdf` | [MDPI 正文](https://www.mdpi.com/2032-6653/17/7/335) · [Brunel 机构记录](https://bura.brunel.ac.uk/handle/2438/33757) | 未找到公开实现 | 全文 PDF 已归档 |
| Yang et al., *A 4WIS Vehicle Trajectory Planning Approach Incorporating Multiple Motion Mode Sequence Decision-Making* (Automotive Innovation, online 2026-01-13) | `A 4WIS Vehicle Trajectory Planning Approach Incorporating Multiple Motion Mode Sequence Decision-Making.pdf` | [Springer](https://link.springer.com/article/10.1007/s42154-025-00369-5) · [DOI](https://doi.org/10.1007/s42154-025-00369-5) | 未找到公开实现 | 本地 PDF 已有；详情可直接核正文 |
| Chang et al., *Generic motion planning for multi-locomotion autonomous vehicles: A spatio-temporal decoupling framework to accommodate multiple maneuvering modes* (Information Sciences 754, 123675; 2026) | `Generic motion planning for multi-locomotion autonomous vehicles_ A spatio-temporal decoupling framework to accommodate multiple maneuvering modes.pdf` | [ScienceDirect](https://www.sciencedirect.com/science/article/pii/S0020025526006067) · [DOI](https://doi.org/10.1016/j.ins.2026.123675) | 未找到公开实现 | **全文 PDF 已归档并分析**；报告接收 2026-05-21，online 2026-05-22 |
| Chang et al., *An Optimal Parking Path Planning Method Integrating Motion Mode Decision-Making for 4WIS Vehicles* (ITSC 2024) | `An_Optimal_Parking_Path_Planning_Method_Integrating_Motion_Mode_Decision-Making_for_4WIS_Vehicles.pdf` | [IEEE DOI](https://doi.org/10.1109/ITSC58415.2024.10920196) | 未找到公开实现 | 本地 PDF 已有 |
| Nguyen et al., *Motion planning for 4WS vehicle with autonomous selection of steering modes via an MIQP-MPC controller* (ICRA 2024) | `Motion_planning_for_4WS_vehicle_with_autonomous_selection_of_steering_modes_via_an_MIQP-MPC_controller.pdf` | [IEEE DOI](https://doi.org/10.1109/ICRA57147.2024.10610461) | 论文称有 Python 实现；未定位公开仓库 | 本地 PDF 已有；研究对象是参考轨迹跟踪，不是全局泊车搜索 |
| Zhang et al., *4WIDS-MAPF: Multiagent Pathfinding for Four-Wheel Independent Drive/Steering Robots* (2025) | `4WIDS-MAPF_Multiagent_Pathfinding_for_Four-Wheel_Independent_Drive_Steering_Robots.pdf` | [IEEE Xplore](https://ieeexplore.ieee.org/document/11077696/) · [DOI](https://doi.org/10.1109/JIOT.2025.3587737) | 未找到公开实现 | 本地 PDF 已有；后续多车扩展参考 |
| Xin et al., *A Generic Trajectory Planning Method for Constrained All-Wheel-Steering Robots* (IROS 2024) | 未见独立 PDF（与 AWSPlanning 对应） | [arXiv v3](https://arxiv.org/html/2404.09677v3) | [AWSPlanning](https://github.com/Rex-sys-hk/AWSPlanning) | 项目已克隆到 `/tmp/4wis_AWSPlanning`；独立 toy optimizer 曾运行成功，非完整规划仿真 |

## 相关但问题设定不同的本地论文

| 论文/主题 | 本地 PDF | 链接 | 对本项目的用途边界 |
|---|---|---|---|
| ParkFormer: Transformer-based parking policy | `01_ParkFormer_2025.pdf` | [arXiv 2506.16856](https://arxiv.org/abs/2506.16856) · [代码页](https://github.com/little-snail-f/ParkFormer) | 学习型感知/决策基线；不是可解释的离散模式规划器 |
| E2E Parking Dataset | `02_E2E_Parking_Dataset_2025.pdf` | [arXiv 2504.10812](https://arxiv.org/abs/2504.10812) · [数据集仓库](https://github.com/KejiaGao/e2e-parking-carla-dataset) | 仿真数据、闭环评估参考 |
| MERL Parking in dynamic environments | `04_MERL_Parking_Dynamic_2024.pdf` | [MERL 技术报告 TR2024-034](https://www.merl.com/publications/TR2024-034) | 动态障碍下静态路径与时空冲突调度分层参考 |
| Deep RL for Adverse Garage Scenario Generation | `03_DRL_Garage_2024.pdf` | [arXiv 2407.01333](https://arxiv.org/abs/2407.01333) | 场景生成和压力测试，不是规划器 |
| HOPE: RL-based Hybrid Policy Path Planner | `05_HOPE_TITS_2025.pdf` | [arXiv 2405.20579](https://arxiv.org/abs/2405.20579) | RL 引导搜索和 RS action；与可解释模式优化不同 |
| MultiPark: Multimodal Parking Transformer | `07_MultiPark_Transformer_2025.pdf` | [arXiv 2508.11537](https://arxiv.org/abs/2508.11537) | 多模态下一段预测学习基线 |
| Automatic parking planning control method based on improved A* | `09_Improved_Astar_2024.pdf` | [arXiv 2406.15429](https://arxiv.org/abs/2406.15429) | A*、Bezier 优化和 MPC，非 4WIS 模式规划 |
| Motion Planning and Control of an Overactuated 4-Wheel Drive with Constrained Independent Steering | `10_Liu_4WD_Steering_2025.pdf` | [arXiv 2510.19054](https://arxiv.org/abs/2510.19054) | 轮级转向可行域、速度不连续区与停车重构参考 |

## 项目与可复用代码

| 项目 | 链接 | 架构/已核验状态 | 适用边界 |
|---|---|---|---|
| AWSPlanning / C-AWS | [GitHub](https://github.com/Rex-sys-hk/AWSPlanning) | 连续轮级动作采样的 Hybrid A* + CasADi/IPOPT 后端；toy example 本地运行成功 | 动作空间不是本项目四个离散模式；许可证需按子目录核对 |
| MPPI Swerve Drive ROS | [GitHub](https://github.com/MizuhoAOKI/mppi_swerve_drive_ros) | MPPI-3D/4D 采样状态选择器 + ROS/Gazebo | mode 是采样维度切换，不是 Ackermann/Lateral/Parallel/Spin 规划序列 |
| OTC-Planner | [GitHub](https://github.com/TeleYuhao/OTC-Planner) | Hybrid A* + Oriented Travel Corridor + CasADi 几何平滑 | 可参考 corridor 和 smoother 工程；要改成全车 footprint 和多模式动力学 |
| Apollo | [GitHub](https://github.com/ApolloAuto/apollo) | Open Space 搜索/迭代平滑/速度优化的工程管线 | 参考分层软件结构，不直接复用车辆模型或全套系统 |
| Safe Corridor 4WIS 仓库 | [GitHub](https://github.com/balmung08/4WIS-Hybrid-Astar-Safe-Corridor-Optimization) | README 指向双向 Ackermann `(x,y,yaw)`；本轮仅网页核验 | 走廊模块参考；名称不代表它实现了多模式规划 |
| Kinodynamic 4WISD Planning System | [GitHub](https://github.com/githubGzj/Kinodynamic-Planning-System-for-4WISD-Robots) | 页面说明论文接收后开放源码；本轮无可核源码 | 暂列观察名单，不计作可复现实现 |
| balmung08 多模式 4WIS 规划 | [GitHub](https://github.com/balmung08/4WIS-Robot-Multimodal-Trajectory-Planning) | 旧笔记记录为 README/图片占位，非可运行实现 | 不以仓库名或论文关联推断其实现完整度 |

**校正提示：**旧笔记有一些将论文作者/年份、仓库、论文版本混写的记录。引用和实验报告请以本索引中的题名、DOI、arXiv 版本、本地 PDF 首页为准。MERL 链接使用报告编号入口；引用前建议检查该页是否重定向到报告 PDF。
