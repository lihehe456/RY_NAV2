# nav2_route_polyline_planner

这是一个 Nav2 全局规划器插件包，主要用于路线跟随、稀疏路点曲线连接，以及面向 `NavigateToPose / ComputePathToPose` 的安全距离规划。

当前提供三个 `nav2_core::GlobalPlanner` 插件：

- `nav2_route_polyline_planner/RoutePolylinePlanner`
- `nav2_route_polyline_planner/RouteSparseCurvePlanner`
- `nav2_route_polyline_planner/RouteClearancePlanner`

## 插件说明

`RoutePolylinePlanner` 是原始直线段规划器。它会优先用直线连接输入 route 点；如果直线段碰撞，则在局部走廊内使用 NavFn fallback 绕行。

`RouteSparseCurvePlanner` 用于稀疏 through-poses 路线。它根据相邻点朝向判断直线或 Hermite 曲线，并在曲线不可通行时进行走廊 fallback。

`RouteClearancePlanner` 用于 `NavigateToPose / ComputePathToPose` 场景，不要求人工提供中间稀疏路点。它会先生成内部参考路径，再围绕参考路径自动构建走廊，并结合最终 `global_costmap` 的 lethal cell、膨胀代价和 clearance map 生成尽量远离墙体/障碍、同时允许窄通道通过的路径。

## Planner ID 配置

在 `planner_server` 中注册插件，例如：

```yaml
planner_server:
  ros__parameters:
    planner_plugins:
      - GridBased
      - RoutePolyline
      - RouteSparseCurve
      - RouteClearance

    RoutePolyline:
      plugin: "nav2_route_polyline_planner/RoutePolylinePlanner"

    RouteSparseCurve:
      plugin: "nav2_route_polyline_planner/RouteSparseCurvePlanner"

    RouteClearance:
      plugin: "nav2_route_polyline_planner/RouteClearancePlanner"
```

使用安全距离规划器时，将 planner id 指定为 `RouteClearance`。

## RouteClearance 推荐参数

```yaml
RouteClearance:
  plugin: "nav2_route_polyline_planner/RouteClearancePlanner"
  allow_unknown: false
  hard_min_clearance: 0.4
  soft_target_clearance: 1.0
  clearance_weight: 8.0
  centerline_weight: 0.0
  cost_weight: 2.5
  turn_weight: 0.45
  goal_search_radius: 1.0
  reference_corridor_half_width: 2.2
  reference_use_astar: true
  reference_allow_unknown: false
  start_goal_keepout_radius: 0.35
  max_goal_candidates: 80
  path_interpolation_resolution: 0.0
  use_final_goal_orientation: true
```

常用调参方向：

- `hard_min_clearance`：硬安全距离。低于该距离的栅格不可通行。
- `soft_target_clearance`：期望安全距离。低于该距离会增加代价，但在窄通道中仍允许通过。
- `clearance_weight`：远离障碍物和虚拟墙的权重。
- `cost_weight`：避开高代价膨胀区的权重。
- `turn_weight`：转弯惩罚，增大后路径会更倾向少转弯、少折线。
- `reference_corridor_half_width`：围绕内部参考路径生成自动走廊的半宽。
- `goal_search_radius`：当目标点不可通行或过近障碍时，在该半径内搜索安全替代终点。
- `path_interpolation_resolution`：输出路径插值分辨率，`0.0` 表示使用 global costmap resolution。
- `use_final_goal_orientation`：末端是否保留用户给定的 goal orientation。

如果路径离墙仍然太近，优先增大 `soft_target_clearance`、`clearance_weight` 或 `cost_weight`。

如果路径已经离墙足够远但不够平滑，可以适当增大 `turn_weight`，并略微减小 `clearance_weight` 或 `reference_corridor_half_width`。

## 虚拟墙接入方式

`RouteClearancePlanner` 不直接订阅虚拟墙话题，只读取最终的 Nav2 `global_costmap`。因此虚拟墙应通过 costmap layer 写入全局代价地图。

推荐 global costmap 插件顺序：

```yaml
plugins:
  - static_layer
  - obstacle_layer
  - virtual_wall_layer
  - inflation_layer
```

这样虚拟墙会先成为 lethal cost cell，再由 inflation layer 膨胀，planner 可以统一基于最终 costmap 做安全距离规划。

## 编译

在 ROS 2 workspace 根目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select nav2_route_polyline_planner
source install/setup.bash
```

## 测试

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test --packages-select nav2_route_polyline_planner --event-handlers console_direct+ --ctest-args --output-on-failure
```

## 可视化验证示例

可以使用已有的 planner debugger 验证 `RouteClearance`：

```bash
ros2 launch fishbot_navigation2 planner_debugger.launch.py \
  params_file:=/home/xingchen/workspace/robot_ws/nav2_ws/src/fishbot_navigation2/param/fishbot_rpp_smachyb.yaml \
  route_json:=/home/xingchen/workspace/robot_ws/nav2_ws/src/11_0_sparse-twopoints.json \
  output_json:=./11_0_route_clearance_global_path.json \
  map:=/path/to/map.yaml \
  planner_id:=RouteClearance \
  use_sim_time:=false \
  rviz:=true
```

对于 `RouteClearance`，双点 JSON 即可模拟 NavToPose 规划：起点加终点，不需要继续手工提供中间稀疏点。
