# Hainan 16 线 FAST-LIO-SAM 动态点去除实验

## 结论摘要与证据优先级

结合实验输出 PCD 的人工三维检查，当前结论修订为：

1. **FreeDOM F0 是当前推荐 baseline**。它在实际 PCD 中对动态点和车辆拖影的清理效果最好，
   同时整体静态结构保留优于 ERASOR；F0 不依赖未经标定的垂直 FOV，且运行速度最快。
2. **ERASOR E1 能去除动态点，但存在不可接受的结构化误删**，尤其是细杆、栅栏等稀疏却对
   定位有价值的目标。E2 更激进，不推荐。
3. **HMM-MOS H0 效果不佳，不推荐用于当前输入**。最可能原因是输入为间隔约 0.45 m 的 FAST-LIO
   keyframe，且没有 timestamp，HMM 的帧窗口无法表达稳定的真实时间；此外 50 m 量程裁剪也混入
   了输出差异。

评价证据按以下优先级解释：人工三维 PCD 检查 > 统一尺度 ROI/俯视图 > 0.2 m 最近邻 approximate
removed 数量。最近邻差分只能说明 raw 与输出的几何差异，无法判定差异点是动态物、重复观测、
体素表示变化还是被误删的静态结构，因此不能单独用于算法排名。报告中的定量数据保持不变，
算法优劣和推荐则以人工 PCD 检查后的结论为准。

## 1. 数据说明

实验分支为 `experiment/dynamic-removal`，基线为干净的 `feat/ros1`。原始数据位于
`test_qc/session{1,2}`，转换数据位于 `out/uni_mapper/session{1,2}`。转换脚本只读取
`VERTEX_SE3:QUAT`/`VERTEX_SE3:QUATERNION`，归一化 `(qx,qy,qz,qw)` 后输出 KITTI 3×4
矩阵；`VERTEX_SCALE:DOUBLE`、`EDGE_DIS:VEC3`、`EDGE_SE3:QUAT` 和 `FIX` 均不会被误当作
pose。`frame_map.csv` 记录原始 vertex/PCD 到连续输出索引的映射。

| session | scan | pose | 原始点数 | scan 索引 | PCD 物化方式 |
|---|---:|---:|---:|---|---|
| session1 | 2,388 | 2,388 | 11,514,934 | 0–2387 连续、一一对应 | hardlink |
| session2 | 1,899 | 1,899 | 9,708,142 | 0–1898 连续、一一对应 | hardlink |

PCD 中的点以 LiDAR/body 原点为中心，而 pose 平移约在地图坐标数百米处，确认 scan 是局部
keyframe，不是世界坐标整图。session1 相邻 keyframe 平移间隔 mean/median/P90/max 为
0.454/0.483/0.508/0.569 m，旋转间隔为 0.223/0.057/0.742/2.761°。数据没有可用时间戳，
因此 time interval 为 `UNKNOWN`，不能把这些 keyframe 当成固定 10/20 Hz 原始帧。
session2 对应平移为 0.461/0.480/0.519/0.580 m，旋转为 0.258/0.056/0.877/2.188°，
与 session1 的关键帧密度一致。

session1 的局部 Z 精确 P1/P5/P50/P95/P99 为 −0.944/−0.260/1.660/14.840/20.036 m；
每帧最低点中位数为 −1.259 m。仅凭回波分布无法可靠反推出 LiDAR 离地外参，所以 ERASOR
高度参数保持原值。回波 elevation 的观测包络约 −5.5°～31.5°，它受场景遮挡和安装姿态
影响，只作为 FreeDOM F1/F2 的“数据估计 FOV”，不是传感器标定值。

## 2. OpenLMM Dynamic Remover 架构

实际调用链为：

```text
MapServer
  -> MapAligner(DataLoader -> LoopDetector -> BackendOptimizer)
  -> MapUpdater(reload raw local scans + db_optimized_poses)
  -> DynamicRemoverBase
       -> DynamicRemoverOffline -> ERASOR / FreeDOM
       -> DynamicRemoverOnline  -> HMM-MOS / OTD / DUFOMap
  -> 0.2 m downsample
  -> global_map_A.pcd
```

`MapServer::process()` 先完成所有 agent 的 `MapAligner`，随后才逐 agent 运行 `MapUpdater`。
因此当前动态去除发生在该轮 intra/inter-agent 优化之后；它不会清理送入 Scan Context、KISS
Matcher 或 backend 的 keyframe，也不会改变 loop/PGO。此次实验没有修改该顺序。

Offline wrapper 先用优化 pose 把所有 raw scan 拼成世界系 raw map，再向插件传入每帧 scan 和
pose。FreeDOM 在 `build_scan_map()` 内部完成局部到世界变换；ERASOR core 明确要求 query 已在
世界系，但原适配没有执行变换，本实验补上该适配。ERASOR 原先使用 `pcl::VoxelGrid` 时，
Hainan 大地图在 0.2 m 下发生 int32 稠密体素索引乘积溢出、过滤被跳过；改为项目已有的稀疏
哈希体素器后，配置的 leaf size 才实际生效。两处均为输入/容器适配，不改变 R-POD 判定逻辑。

## 3. ERASOR 原理

ERASOR 从完整拼接图截取当前 pose 周围的 Volume of Interest，并按半径 ring 和方位 sector
构成 R-POD。每个 bin 记录 map/query 的最低、最高高度和点集。`scan_ratio` 是两者高度跨度之比
的对称形式；低于阈值时，若 map 高而 query 低，表示先前占据物已消失，候选为动态。算法再做
地面提取，把地面放回，避免整柱删除。阈值越大越容易触发，因而 E2 理论上比 E1 更 aggressive。

需要特别注意：upstream E0 的 `replace_intensity=true` 会把动态候选设置 intensity 后重新加入
输出，它是 label baseline，不是纯静态地图。E1/E2 使用 `false` 才得到实际移除后的 static map。

## 4. FreeDOM 原理

FreeDOM 把当前 scan 和全局地图表示为 block/voxel/sub-voxel 的多分辨率稀疏结构。每条 LiDAR
射线经过的单元累积 free-space evidence；已占据单元持续被射线穿过并达到 `counts_to_free` 后被
判为动态，`counts_to_revert` 用于稳定结构恢复。随后按 conservative/aggressive connectivity
扩展标签并把当前静态观测积分回地图。

F0 关闭 depth-image raycast enhancement，只检验 voxel/free-space 主路径；F1 用 16 行深度图和
数据估计 FOV；F2 仅把 `counts_to_free` 从 6 改为 3，检验灵敏度。配置中的 `replace_intensity`
在当前 FreeDOM `getStaticMap()` 路径中不影响输出：只导出 `dynamic_level <= STATIC` 的 sub-voxel。

## 5. HMM-MOS 为什么可能不适合 FAST-LIO keyframe

HMM-MOS 先把每帧变换到世界系并体素化，用 Bresenham raycasting 更新 occupied/free/unobserved
三状态 HMM，再对局部时间窗的 3-D 卷积分数做 Otsu 阈值和区域生长。其窗口单位是“输入帧数”而
不是米或秒。默认 `local_window_size=3` 在本数据约覆盖 1.36 m，`global_window_size=300` 对应
约 136 m 的行程，但帧间时间完全未知。关键帧选择造成的非均匀时序和 0.45 m 级位移，会弱化连续
原始帧方法依赖的遮挡、free/occupied 状态转移证据。

此外当前实现解析了 `free_sigma` 但没有在 HMM 更新中使用；H0 的 `replace_intensity=true` 同样是
标记并放回模式。H1 只做最小密度调整：local 3→5（约 2.27 m）、global 300→220（约 100 m），
并关闭放回以观察静态输出。若 H0 已显示运行或输入不适配，H1 不应被当作效果调参。

## 6. 实验参数

所有统计统一以 0.2 m voxel 和 0.20 m raw-to-static 最近邻阈值生成 approximate removed cloud。
该点云只用于对齐后的可视化，不是算法内部 dynamic label。

| config | 关键差异 |
|---|---|
| ERASOR E0 | upstream 全默认；80 m，20×108，min pts 6，ratio 0.20，interval 5，label/reinsert |
| ERASOR E1 | 60 m，20×72，min pts 4，ratio 0.20，query/map 0.1/0.2 m，interval 1，实际删除 |
| ERASOR E2 | E1，仅 ratio 0.20→0.30 |
| FreeDOM F0 | upstream 其余默认，raycast enhancement=false，counts 6 |
| FreeDOM F1 | enhancement=true，16 lines，data-estimated FOV −5.5°～31.5°，counts 6 |
| FreeDOM F2 | F1，仅 counts_to_free 6→3 |
| HMM H0 | upstream 全默认，label/reinsert |
| HMM H1 | 预置 local 5、global 220、replace_intensity=false；H0 暴露输入不适配后未运行 |

ERASOR `min_h=-1.7, max_h=3.1, tf_z=0.7` 未改。DUFOMap 在 ROS1/Ubuntu 20.04 的 GCC 9
下因 C++20 concepts 不可用而由构建系统关闭；不为单一插件升级系统。OTD 是可选项，本轮不把它
混入 ERASOR/FreeDOM 主结论。LAMM/M-Detector 对连续时序、depth map、occlusion 及速度/加速度
假设更敏感，也不纳入本轮。

## 7. 实验结果

完整机器可读数据见 `experiment_results/hainan_dynamic_removal/session1/metrics.csv`。表中 raw/static
为输出 PCD 的 voxel 前点数；removed/ratio 是 raw 与 static 都先做 0.2 m voxel 后，最近邻距离
大于 0.20 m 的近似值。

### session1 主实验

| method | config | raw points | static/output points | removed approx | removed ratio | runtime | peak RSS | 结论 |
|---|---|---:|---:|---:|---:|---:|---:|---|
| raw | baseline | 11,514,934 | 11,514,934 | 0 | 0% | 6.94 s | 1,115.57 MB | 同一优化轨迹的精确拼接；0.2 m 后 3,346,308 点 |
| ERASOR | E0 | 11,514,934 | 3,346,291 | 4 | 0.00012% | 117.57 s | 1,716.96 MB | label/reinsert，不是 clean static map |
| ERASOR | E1 | 11,514,934 | 2,772,714 | 501,394 | 14.98% | 534.21 s | 1,712.06 MB | 有动态清理能力，但人工检查确认细杆/栅栏误删 |
| ERASOR | E2 | 11,514,934 | 2,762,868 | 510,534 | 15.26% | 535.42 s | 1,711.12 MB | 比 E1 多删 9,140 voxel，静态误删风险更高 |
| FreeDOM | F0 | 11,514,934 | 2,709,930 | 471,354 | 14.09% | 33.83 s | 2,599.69 MB | 人工 PCD 检查效果最好；当前推荐 baseline |
| FreeDOM | F1 | 11,514,934 | 2,709,930 | 471,354 | 14.09% | 36.35 s | 2,601.58 MB | 与 F0 的最终 0.2 m 几何判定相同，增强无收益 |
| FreeDOM | F2 | 11,514,934 | 2,664,503 | 498,836 | 14.91% | 36.56 s | 2,601.78 MB | counts 6→3 后更 aggressive |
| HMM-MOS | H0 | 11,514,934 | 2,292,234 | 876,484 | 26.19% | 681.90 s | 2,029.13 MB | **不可横向当作动态删除率**；含 50 m range crop |

`static/output points` 是各输出 PCD 在 MapServer 保存时的点数。removed ratio 的分母是统一
0.2 m 后的 3,346,308 个 raw voxel。E0 输出有约 458,727 个 voxel 的平均 intensity 大于 0.5，
印证其主要用途是标记；H0 最终只有 772 个 voxel 保持 intensity≈1，却因 50 m 量程裁剪出现
876,484 个差分点，说明其 26.19% 绝大部分不能解释为动态检测。

### ROI 差分统计（0.2 m approximate removed 点数）

| ROI | ERASOR E1 | FreeDOM F0 | HMM H0 |
|---|---:|---:|---:|
| 疑似车辆拖影 | 11,252 | 1,537 | 243 |
| 集装箱边缘 | 34,726 | 37,142 | 97,759 |
| 路缘/道路边缘 | 14,345 | 8,466 | 11,430 |
| 墙/建筑边 | 32,170 | 14,226 | 4,065 |
| 远距稀疏回波 | 2,730 | 28,188 | 61,620 |
| 细杆/栅栏 | 1,795 | 9,036 | 16,931 |

这些数字是 raw voxel 到 static map 最近邻距离大于 0.20 m 的数量，不是算法内部 dynamic label，
也没有语义真值。点数较大可能来自动态点被清除，也可能来自降采样、稀疏重建、量程裁剪或表面
采样位置变化；点数较小同样不能证明细杆/栅栏的拓扑被完整保留。人工三维 PCD 检查显示：
FreeDOM 的动态点去除和整体结构完整性最好；ERASOR 虽然某些 ROI 的差分数量较小，实际仍会削掉
细杆和栅栏；HMM-MOS 则保留较多动态残留。该结果说明 approximate removed 数量不能替代三维
结构检查，更不能直接用作算法排名。

### session2 泛化

session2 只复跑 session1 选出的 E1/F0，没有重新搜索参数。

| method | config | raw points | static/output points | removed approx | removed ratio | runtime | peak RSS |
|---|---|---:|---:|---:|---:|---:|---:|
| raw | baseline | 9,708,142 | 9,708,142 | 0 | 0% | 5.58 s | 907.14 MB |
| ERASOR | E1 | 9,708,142 | 2,412,077 | 383,919 | 13.49% | 353.79 s | 1,460.35 MB |
| FreeDOM | F0 | 9,708,142 | 2,321,115 | 368,799 | 12.96% | 27.94 s | 2,232.79 MB |

session2 raw 在 0.2 m 后为 2,844,951 点。E1/F0 的比例相对 session1 分别变化 −1.49/−1.12
个百分点，没有数值失控，说明两组参数的数值尺度均可跨 session。removed 俯视图沿道路、集装箱
和建筑轮廓分布，只能说明这些区域存在 raw-to-output 几何差异，不能直接判为静态误删。结合 PCD
人工检查，FreeDOM 仍作为优先方案；ERASOR 的细杆/栅栏误删是阻止其成为 baseline 的主要问题。

## 8. 可视化

所有 XY 图固定为同一范围并保持 equal aspect ratio：

- `session1/comparison/raw_top.png`
- `session1/comparison/erasor_E0_comparison.png`
- `session1/comparison/erasor_E1_comparison.png`
- `session1/comparison/erasor_E2_comparison.png`
- `session1/comparison/freedom_F0_comparison.png`
- `session1/comparison/freedom_F1_comparison.png`
- `session1/comparison/freedom_F2_comparison.png`
- `session1/comparison/hmm_H0_comparison.png`
- `session1/comparison/comparison_top_view.png`
- `session2/comparison/{raw,erasor_E1,freedom_F0}_top.png`
- `session2/comparison/comparison_top_view.png`

手工 ROI 配置在 `open_lmm/config/experiments/hainan/rois_session1.json`。六类区域覆盖疑似车辆拖影、
集装箱边缘、路沿/道路边缘、墙/建筑边、远距稀疏回波、细杆/栅栏；每个算法的 ROI 目录同时保存
`raw_roi.pcd`、`static_roi.pcd`、`removed_roi.pcd` 和三联图。

## 9. 失败案例

- **ERASOR E0 语义陷阱**：默认 `replace_intensity=true` 把候选动态点标记后重新加入，因此输出
  几何几乎等于 raw。若只比较点数会误判为算法完全无效。
- **ERASOR 静态误删**：E1 有动态清理能力，但人工三维检查确认细杆、栅栏等稀疏静态结构会被
  削掉，部分集装箱边缘和路缘也受到影响。E2 只增加删除量，没有足够收益抵消更高的误删风险。
- **FreeDOM 剩余风险**：人工 PCD 检查显示 F0 的综合效果最好，先前仅根据 approximate removed
  轮廓将其判断为远距/细结构不安全并不充分。F1 的数据估计 FOV 没有改变最终 0.2 m 几何结果，
  且真实 FOV 仍为 `UNKNOWN`；因此推荐不依赖 FOV 的 F0，而不是把估计 FOV 当作标定值。F2 降低
  `counts_to_free` 后更激进，也不作为默认配置。
- **HMM-MOS 输入不适配**：H0 可运行且无 ROS1/GCC 报错，但 11.4 分钟后动态残留仍明显。当前
  输入是约 0.45 m 间隔的关键帧而非固定频率原始扫描，并且没有 timestamp；`local_window_size`、
  `global_window_size` 只能表示关键帧个数，无法对应稳定的真实时间窗，这很可能是效果不佳的主要
  原因。与此同时 `max_range=50 m` 丢弃大量远距观测，使 26.19% NN 差分不能解释为动态删除率。
  H1 只改窗口无法修复缺失时间语义，因此停止调参。
- **DUFOMap 构建限制**：Ubuntu 20.04/GCC 9 缺少该实现要求的 C++20 concepts，构建系统已禁用；
  修复需要工具链升级，超出此次 ROS1 实验范围。OTD 为可选项，未挤占主实验。

判定遵循“删得多不等于好”：车辆消失同时必须检查墙、路沿、杆、围栏和集装箱边缘。全图 removed
ratio 和二维 ROI 差分只作为辅助指标，最终选择以人工三维 PCD 中的动态清理效果与静态结构
完整性为主。

## 10. 最终推荐

当前推荐 **FreeDOM F0** 作为 Offline Map Builder 的动态点去除 baseline：关闭
`enable_raycast_enhancement`，保持 `counts_to_free=6` 和其余 upstream 参数。选择依据不是其
14.09% removed ratio，而是输出 PCD 的人工三维检查确认它对动态点/车辆拖影的清理效果最好，
同时没有 ERASOR 那样明显地削掉细杆和栅栏。F0 在 session1/session2 分别为 14.09%/12.96%，
参数尺度跨 session 稳定；运行时间约 33.83/27.94 s，也显著短于 ERASOR E1 的 534.21/353.79 s。

FreeDOM F1 与 F0 的最终 0.2 m 几何结果相同，但使用了未经标定的数据估计 FOV，且略慢，因此没有
理由优先于 F0。F2 把 `counts_to_free` 从 6 降到 3 后更激进，也不作为默认方案。FreeDOM 峰值
内存约 2.60 GB，高于 ERASOR 的约 1.71 GB，这是部署时需要接受或继续优化的主要代价。

ERASOR E1 保留为对照配置，不推荐作为当前 baseline。它能够去除部分动态点，但细杆、栅栏和部分
边缘结构的误删会损害后续定位/配准所需的几何约束。HMM-MOS 不建议继续用于当前 keyframe 数据；
只有获得带 timestamp 的连续原始 LiDAR stream，并把 50 m range crop 与动态标签分开评估后，
才值得重新测试。

即使 FreeDOM F0 当前表现最好，也建议先以可回滚的旁路 static map 接入，不直接替代送入 loop/PGO
的 raw keyframe。缺少语义真值时，人工 PCD 检查可以纠正二维差分指标的误导，但仍不能给出严格的
precision/recall。

## 11. 下一步

1. 对六个 ROI 建立可复核的点级或体素级静/动态真值，补充 precision/recall，而不再仅依赖 NN 差分。
2. 优先给 FreeDOM F0 增加“只评估不接管”的 per-scan static mask 导出，验证车辆清理以及墙、
   路缘、细杆、栅栏的保留率；不直接覆盖 raw keyframe。ERASOR E1 保留为误删对照。
3. 将 raw 与 static keyframe 作为两条并行支路，对比 Scan Context recall、错误闭环率、KISS inlier、
   PGO 残差和最终地图一致性；只有静态支路持续获益后才接入 multi-session registration。
4. 若继续 FreeDOM 的 raycast enhancement，再从雷达型号/标定文件确认真实 16 线垂直角表；当前
   −5.5°～31.5° 仅是场景回波统计。F0 本身不依赖这个估计 FOV，也不以降低 `counts_to_free`
   作为默认方向。
5. HMM-MOS 改用连续原始帧并保留 timestamp 后重新定义时间窗；否则停止在 keyframe 上调参。

只有在 session2 泛化通过后，才建议增加额外支路实验：保留当前 raw keyframe/PGO 基线，同时把
最佳动态去除器输出的静态 keyframe 用于 multi-session registration。不能直接用“清过的最终整图”
替代逐帧描述子输入；需要先定义可追溯的 per-scan static mask，并分别比较 loop recall、错误闭环率、
配准 inlier 和最终地图一致性。
