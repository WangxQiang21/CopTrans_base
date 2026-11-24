# 源分支和目标后端

| ROS2 后端 | ROS1 分支 | 来源提交 | 原配置默认值 |
|---|---|---|---|
| `single_multimodal_hybrid_astar` | `multimodal-hybrid-astar` | `c245556b29581ab9390b01e5db18d3a14bf472e8` | `legacy` |
| `single_constant_primitive` | `mode-path-constant-primitive` | `19f87a9c6d8616cdd712939d90b0776ddc57b0c8` | `mode_path_v1` |
| `single_continuous_mode_path` | `continuous-mode-path` | `956bc64ead93f38e8f5632738ceb0f5e6cb7f4ee` | `mode_path_v2` |
| `multi_4wis` | `4wis_mutil_mode_plan01` | `bb02ed7c59eeffeffb40c12016b14ba55707bc42` | `virtual_rigid_a` |

原工作空间入口为 `catkin_ws/src/fwis_v3_demo/launch/{demo,formation_demo}.launch`。源实现从 `fwis_v3_planner`、`fwis_v3_optimizer`、`fwis_v3_playback`、`fwis_v3_demo` 提取，目标节点放到 `ros2_ws/src/fwis_backends/<后端>/`，算法 Python 模块位于相同包内的 `fwis_backends/<后端>/`。公共运动学和消息分别在 `ros2_ws/src/fwis_common` 与 `ros2_ws/src/fwis_v3_msgs`。
