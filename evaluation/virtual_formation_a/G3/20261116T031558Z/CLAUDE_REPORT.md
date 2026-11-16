# G3 返工报告：第三空间导数符号修复与全域动态证书复核

状态：`PASS_PENDING_CODEX_RECHECK`。Codex 接受状态：
`NOT_RUN_NOT_CLAIMED`。本报告针对首轮 G3 审核 `NOT ACCEPTED` 的返工要求，
不自行声明 `ACCEPTED`，不进入 G4。

运行目录：`evaluation/virtual_formation_a/G3/20261116T031558Z`。基线
HEAD：`567bdecba9676313da3bc9a97b82f57b5b569a2b`；分支：
`feature/virtual-rigid-formation-a`。未提交、未推送、未建分支、未操作其它
worktree。首轮 G3 证据 `evaluation/virtual_formation_a/G3/20261116T023528Z/`
原样保留。

## 1. 结论摘要

- 已修复 `timing.py:_vehicle_bounds` 与 `audit.py:_audit_interval_vehicle`
  中连续区间 `p_sss` 的 x 分量符号：`+3*a*k*eta` 改为
  `-3*a*k*eta`；点态代码和 G1 几何原本就使用正确符号。
- Codex 退化区间反例现在两个区间实现都给出
  `jerk = 0.10677546534667996`，不小于真实
  `||p_sss|| = hypot(-0.04, 0.099) = 0.10677546534667971`；
  修复前两个实现都给出 `0.10100000000000023`，确实低估。
- 24 个窄区间用例覆盖全部三个偏移、`k`/`eta` 四种符号、`p_sss` 与 jerk
  每个分量及范数，0 失败。
- 两条真实合法 C2 曲率路径（互为镜像）重新做全路径五次时间参数化：
  `T=10.654562988281448 s`，每条 4 个 Hermite 段、32 个闭区间叶节点、
  20 个内部点（每段 q=0.1/0.3/0.5/0.7/0.9）和 3 个内部节点左右极限，
  三辆车全部包含；前车 `a=1` 且 `k*eta` 非零，四条
  `(sign k, sign eta)` 组合全覆盖。
- 合法负例：同一 C2 路径将 `jerk_max` 收紧到 `0.2`，再把时间缩短为
  `0.8`；审核返回 `DYNAMIC_LIMIT`（车辆 3，jerk
  `0.20980131467237287 > 0.2`），独立 20,001 点稠密扫描在
  远离端点处找到 `0.3055394669537801` 的见证。
- 粗区间过宽时审核返回 `BOUND_UNCONFIRMED`，细分后 13 个叶节点确认；
  没有把“区间无法确认”误报为动态可行。
- 八条 G2 冻结几何路径全部重新过 G3：T 与首轮 G3 **逐位相等**，九个
  模型摘要全部未变；151 项 benchmark 全过；测试 18/23/51 全过。
- Legacy 记录保持真实：整体 `FAIL`，4 个通过、`timing_continuous`
  仍因基线已有的 `KeyError: 'dynamic_peaks'` 失败；未修改或放宽旧测试。

## 2. 缺陷、正确推导与符号修复

### 2.1 正确的刚体偏移局部导数

令偏移 `(a,b)`、局部曲率 `k`、曲率速率 `eta=dk/ds`、二阶曲率速率
`u=deta/ds`，局部切向/法向坐标为 `R(-psi)p`。G1 几何和点态轨迹使用：

```text
R(-psi) p_s   = (1 - b*k, a*k)
R(-psi) p_ss  = (-b*eta - a*k², k - b*k² + a*eta)
R(-psi) p_sss = (-k² + b*k³ - 3*a*k*eta - b*u,
                  eta - 3*b*k*eta + a*u - a*k³)
```

推导要点：对 `p_ss` 的 x 分量 `-b*eta - a*k²` 求弧长导数得
`-b*u - 2*a*k*eta`；在曲率 `k` 的局部标架中对向量 `(vx,vy)` 求导还要加
`(-k*vy, +k*vx)`，其中 `-k*vy` 对 `vy=k-b*k²+a*eta` 贡献 `-a*k*eta`。
两项合并即为 `-3*a*k*eta`。因此 `+3*a*k*eta` 不是等价变形，而是符号
错误。

### 2.2 缺陷位置与影响

首轮 `timing.py:_vehicle_bounds` 和 `audit.py:_audit_interval_vehicle`
的 z3.x 区间都使用：

```text
-k² + b*k³ + 3*a*k*eta - b*u      # 错误
```

点态 `spatial_derivatives` 和 G1 几何使用 `-3*a*k*eta`。前车偏移为
`(1,0)`，`a=1,b=0`；因此只要 `k*eta != 0`，错误项会使区间 x 分量偏移，
并可能低估 `||p_sss||` 与 jerk 上界。侧车偏移 `(0,1)`、`(0,-1)` 的
`a=0`，该缺陷项为零；但侧车仍通过 `-3*b*k*eta`（z3.y）和完整 jerk
公式接受检查。

符号修复本身只把两处区间实现的 z3.x 改为 `-3.0 * aoff`。本次返工同时
增加了区间分量诊断字段、生产 audit 的叶中点对照和节点左右极限对照；
这些新增只做包含性检查和证据记录，不改变时间律、限值、容差、预算、
外扩规则、时长选择或失败关闭行为。

### 2.3 Codex 退化反例

输入：车辆 1 偏移 `(a,b)=(1,0)`，`k=0.1`，`eta=0.1`，`u=0`，
`g=0`，`w=1`，`a_time=0`，`j_time=0`。

```text
p_sss = (-0.01 - 0.03, 0.1 - 0.001) = (-0.04, 0.099)
```

区间实现还产生 y 区间 `[0.09899999999999996, 0.09900000000000005]`，
实际点 `0.1` 位于其中。修复后：

| 实现 | 修复前 jerk | 修复后 jerk | 真实 `||p_sss||` |
|---|---:|---:|---:|
| timing 区间 | 0.10100000000000023 | 0.10677546534667996 | 0.10677546534667971 |
| audit 区间 | 0.10100000000000023 | 0.10677546534667996 | 0.10677546534667971 |

两个实现的 `p_sss_norm_upper` 分别为 `0.10677546534667981` 和
`0.10677546534667981`，`jerk_component_norm_upper` 均为
`0.10677546534667991`，均不小于真实值。该反例在
`tests/test_virtual_a_timing.py` 与 `tests/test_virtual_a_audit.py` 的
独立测试中都直接调用区间实现并独立计算闭式真实值。

### 2.4 窄区间覆盖

24 个用例：三个偏移 × 四种
`(sign k, sign eta)` 符号样本 × 区间半宽 `1e-5`；timing/audit 两个区间
实现各 12 例。每个用例检查：

```text
p_sss_x, p_sss_y          实际值落在闭区间内
p_sss_norm               实际范数 <= 上界
jerk_x, jerk_y           实际分量落在闭区间内
jerk_component_norm      实际范数 <= 分量范数上界
jerk_norm                实际范数 <= 标量 jerk 上界
```

24/24 `all_contained=true`，`failure_count=0`。测试使用独立闭式/数值
微分，而不是比较两个复制的区间实现。

## 3. 区间证书覆盖范围与时间阶数

### 3.1 覆盖范围

`timing.py` 的时长选择和 `audit.py` 的独立动态审核都把每个 Hermite 段
划分为 8 个闭区间单元
（`interval_subdivisions_per_segment=8`）。闭单元包含端点：一个内部节点
同时属于左段最后一个单元和右段第一个单元；因此节点左、右极限都在证书
覆盖内。

对每个单元、每个车辆、每个轮位，区间实现计算：

```text
p_s   = (1-b*k, a*k)
p_ss  = (-b*eta-a*k², k-b*k²+a*eta)
p_sss = (-k²+b*k³-3*a*k*eta-b*u, eta-3*b*k*eta+a*u-a*k³)

jerk = p_sss*w³ + 3*p_ss*w*a_time + p_s*j_time
```

其中 `w,a_time,j_time` 是五次时间律在对应闭 τ 单元上的区间；乘法和加法
全部使用区间运算并对上界 `_next_up`、下界 `_next_down` 外扩。`jerk` 的
标量返回值是 `||p_sss||*|w|³ + 3*||p_ss||*|w|*|a| + ||p_s||*|j|` 的外扩
上界；`p_sss_local_interval`、`jerk_local_interval`、
`p_sss_norm_upper`、`jerk_component_upper` 等字段是新增的覆盖诊断，不
改变时长选择或审核接受规则。审核对每个轮位取最小值。

### 3.2 时间阶数未变

```text
S      = 路径总弧长
tau    = t/T
h      = 10*tau³ - 15*tau⁴ + 6*tau⁵
w      = S*h1/T
a_time = S*h2/T²
j_time = S*h3/T³
h1     = 30*tau²*(1-tau)²
h2     = 60*tau - 180*tau² + 120*tau³
h3     = 60 - 360*tau + 360*tau²
```

所有轮速、转向率、加速度、jerk、yaw 量、横向加速度的限值和外扩方式
均未修改；600 s 工程预算、`T0=1 s`、1.01 安全因子、最多 8 次 1.05
重缩放、积分误差容差和时间反解容差均未放宽。

## 4. 真实 C2 路径的点态数值对照

### 4.1 路径与时间参数化

正例路径以 G2 夹具车辆模型和默认限值为基础，构造合法 C2 曲率剖面：

```text
k(s)   = sign * (0.08*s² - 0.025*s³)
eta(s) = sign * (0.16*s  - 0.075*s²)
knots (s,k,eta):
  (0, 0, 0)
  (1, sign*0.055, sign*0.085)
  (2, sign*0.12,  sign*0.02)
  (3, sign*0.045, -sign*0.195)
  (4, -sign*0.32, -sign*0.56)
```

`sign=+1` 为 `positive_curvature_cubic`，`sign=-1` 为
`negative_curvature_cubic`；两条路径互为镜像。路径从 `(0,0,0)` 开始，
满足 G1/G2 的初始曲率和曲率速率约束，`C2CurvatureProfile.from_knots`
生成 4 个 Hermite 段。两者都使用默认请求重新选择公共 T；结果均为：

```text
status          = DYNAMIC_FEASIBLE
required_duration = 10.549072265625195 s
selected T      = 10.654562988281448 s
rescalings      = 0
audit check_count = 513
certificate leaves = 32 (4 segments × 8)

positive model sha256 = 6e26472608ef60db697105dc8c57cab4472ae4a797d385add3bc4d91db40b803
positive request sha256 = efdb61b9fc8bb222a4474f4e22750fa1a6c100132a5b2f1b0697da71322b5381
negative model sha256 = 70955164fd6bf5e4717c2f37a3edcb604b324a1d7089e78d662d8d640bbb6caf
negative request sha256 = 92c7083a6f7853a6e69ebdca72babd131cccfb4b08ae2063caa7f54add44e580
```

### 4.2 采样覆盖

每条路径：

- 4 个 segment，每段 5 个内部点 q=0.1/0.3/0.5/0.7/0.9，共 20 点；
- 3 个内部 Hermite 节点，每个节点分别计算左极限和右极限；
- 每个点/极限检查三辆车的 `p_sss_x/y`、`p_sss` 范数、jerk x/y 分量、
  jerk 分量范数和 jerk 标量范数。

`comparison.contained` 在全部 20 个内部点和 3 个节点左右极限上为真；
三辆车共有 20×3 + 3×2×3 = 78 个 `(点/极限, 车辆)` 组合，每个组合 7 项
检查，全部包含。前车 `a=1` 的 20 个内部采样点均有非零 `k*eta`。

四条符号组合的并集：

```text
positive_curvature_cubic: (-1,-1), (1,-1), (1,1)
negative_curvature_cubic: (-1,-1), (-1,1), (1,1)
union:                    (-1,-1), (-1,1), (1,-1), (1,1)
```

### 4.3 代表性点：段 1、q=0.5

`segment_index=1`（第二段），`q=0.5`，正路径的局部量：

```text
tau=0.4325183551362153, t=4.608294058386691 s
k=0.095625, eta=0.07125, w=0.6785132422138116
a_time=0.07003449097856976, j_time=-0.09379271866720691
```

| 路径 | 车辆 | 实际 jerk 范数 | 局部区间 jerk 上界 | 实际 `p_sss` 范数 | 区间 `p_sss` 上界 |
|---|---:|---:|---:|---:|---:|
| positive | 1 | 0.1056341042856931 | 0.13585059214142767 | 0.030068407274780506 | 0.04087264286213266 |
| positive | 2 | 0.08224574406904031 | 0.13512802462780013 | 0.07615769997403676 | 0.0931360390428864 |
| positive | 3 | 0.12395098368368843 | 0.17030317215224877 | 0.1184686040763853 | 0.13434855256943715 |
| negative | 1 | 0.1056341042856931 | 0.13585059214142767 | 0.030068407274780506 | 0.04087264286213266 |
| negative | 2 | 0.12395098368368843 | 0.17030317215224877 | 0.1184686040763853 | 0.13434855256943715 |
| negative | 3 | 0.08224574406904031 | 0.13512802462780013 | 0.07615769997403676 | 0.0931360390428864 |

负路径在同一 q 点交换了车辆 2/3 的数值，符合镜像关系。每个分量
`jerk_x`、`jerk_y`、`p_sss_x`、`p_sss_y` 的闭区间包含性在
`analysis/g3_rework_numerics.json` 的 `samples[*].comparison.vehicles[*].checks`
中逐项记录，不能只看范数。

### 4.4 代表性节点：内部节点 2 的左/右极限

节点 2 是正路径第二段与第三段之间的内部节点，路径位置 `s=2`。
`left_comparison` 使用左段最后单元，`right_comparison` 使用右段第一单元。

| 路径 | 车辆 | 左极限 jerk 范数 / 上界 | 右极限 jerk 范数 / 上界 | 左 `p_sss` 范数 / 上界 | 右 `p_sss` 范数 / 上界 |
|---|---:|---:|---:|---:|---:|
| positive | 1 | 0.1197944798404484 / 0.149780428117015 | 0.11979447984044851 / 0.16137968982150425 | 0.12362955141874446 / 0.12544173831386765 | 0.12362955141874507 / 0.16082162263191008 |
| positive | 2 | 0.04312826046605571 / 0.1388373600386996 | 0.04312826046605549 / 0.14284038485286135 | 0.12796976042800107 / 0.13279269581471978 | 0.1279697604280017 / 0.14748597428217794 |
| positive | 3 | 0.16584910804506658 / 0.17473620937452375 | 0.1658491080450668 / 0.17852185052600003 | 0.15847962766235912 / 0.16537985000000888 | 0.15847962766235973 / 0.17771172279166322 |
| negative | 1 | 0.1197944798404484 / 0.149780428117015 | 0.11979447984044851 / 0.16137968982150425 | 0.12362955141874446 / 0.12544173831386765 | 0.12362955141874507 / 0.16082162263191008 |
| negative | 2 | 0.16584910804506658 / 0.17473620937452375 | 0.1658491080450668 / 0.17852185052600003 | 0.15847962766235912 / 0.16537985000000888 | 0.15847962766235973 / 0.17771172279166322 |
| negative | 3 | 0.04312826046605571 / 0.1388373600386996 | 0.04312826046605549 / 0.14284038485286135 | 0.12796976042800107 / 0.13279269581471978 | 0.1279697604280017 / 0.14748597428217794 |

节点极限存在有界的 `g`/jerk 跳变（G2 几何允许），但左右两侧各自的局部
区间证书都包含对应单侧值；不是用节点中心值替代左/右极限。

### 4.5 正例汇总

| 路径 | T (s) | 段/叶 | audit checks | 20 点最大前车 jerk 范数比 | 最大 `p_sss` 范数比 | 全部包含 |
|---|---:|---:|---:|---:|---:|---|
| positive_curvature_cubic | 10.654562988281448 | 4/32 | 513 | 0.9563717974527609 | 0.97897151641978 | true |
| negative_curvature_cubic | 10.654562988281448 | 4/32 | 513 | 0.9563717974527609 | 0.97897151641978 | true |

生产 audit 的内部叶中点对照记录分别为 `max_jerk_ratio=0.9470976570447946`、
`max_p_sss_ratio=0.983316155135251`，与上面的独立 20 点见证不同但都小于
1；两者是不同采样位置，不互相替代。

## 5. 负例与失败关闭

### 5.1 真实 jerk 超限

使用合法 C2 路径：

```text
knots (s,k,eta):
  (0, 0, 0)
  (1, 0.020, 0.050)
  (2, 0.105, 0.115)
  (3, 0.25166666666666666, 0.185)
```

其余限值提高到 `1e6`，只把 `jerk_max` 收紧到 `0.2`。完整 T 的审核通过：

```text
T = 11.064619675640401 s
audit_passed = true
check_count = 414
scanned_maximum.jerk = 0.16632277418088717 (车辆 3, tau=1 的单侧终点)
```

把同一路径的时间缩短为 `0.8` 后，`T=8.851695740512321 s`，审核返回：

```text
reason = DYNAMIC_LIMIT
first_failure = {quantity: jerk, vehicle_id: 3,
                 value: 0.20980131467237287, limit: 0.2}
```

独立稠密扫描使用 20,001 个等间隔 τ 点（不是生产证书，也不是全域证明）：

```text
全区间最大 jerk = 0.3248491683220452 (t=T 的单侧终点)
开区间最大      = 0.32475171844248624 (tau=0.99995)
远离端点最大    = 0.3055394669537801
                  (t=8.763178783107199, tau=0.99, s=2.999970448199999,
                   车辆 3, k=0.2516611996229652, eta=0.184997340355466,
                   w=0.0009965209219323524)
远离端点超限样本数 = 3611
```

远离端点的见证排除了“只在终点单侧 jerk 超限”的歧义。扫描只声明
“存在这些采样见证”，不是全域证明；全域结论来自区间证书和审核的
`DYNAMIC_LIMIT` 拒绝。

### 5.2 粗区间过宽时失败关闭

同一 G_JUMP 路径、默认限值，`T=7.628074976415586 s`。粗配置
`interval_subdivisions_per_segment=1, max_depth=0`：

```text
passed = false
reason = BOUND_UNCONFIRMED
first_failure = {quantity: speed, vehicle_id: 3,
                 value: 0.8185223689206615, limit: 0.8,
                 segment_index: 1}
```

这不是实际速度超限，而是粗区间上界过宽，无法确认。审核没有返回
`DYNAMIC_FEASIBLE`，而是失败关闭。细分到 `max_depth=20` 后使用 13 个叶
节点确认，`passed=true`，最大 speed 利用率 `0.9942039542263723`。这说明
审核在“证书无法确认”时细分或拒绝，而不是把过宽上界当成可行。

## 6. 八条 G2 冻结几何路径的 G3 重规划

### 6.1 T、时长选择和约束

| 实例 | S (m) | T (s) | required (s) | worst 约束 | 车辆/段/阶 | 最大利用率 |
|---|---:|---:|---:|---|---:|---:|
| E1 | 6.000496658791536 | 14.627622484021819 | 14.482794538635464 | speed | 3 / 4 / 1 | 0.9900990099009901 |
| E2 | 6.000496658791535 | 14.627622484021813 | 14.482794538635458 | speed | 2 / 4 / 1 | 0.9900990099009901 |
| E3 | 6.0020064978265975 | 15.10059882623887 | 14.951087946771159 | speed | 3 / 4 / 1 | 0.9900990099009902 |
| E4 | 6.002006497826603 | 15.10059882623911 | 14.951087946771397 | speed | 2 / 4 / 1 | 0.9900990099009902 |
| E5 | 8.011137789911 | 20.842263324804446 | 20.6359042819846 | speed | 3 / 4 / 1 | 0.9900990099009895 |
| E6 | 8.011137789911029 | 20.842263324805067 | 20.635904281985216 | speed | 2 / 4 / 1 | 0.9900990099009893 |
| NO_MOTION_HOLD | 0.0 | 0.0 | 0 | exact hold | - | - |
| STRAIGHT_LINE | 5.999999999999998 | 14.203125000005778 | 14.062500000005722 | speed | 2 / 4 / 1 | 0.9900990099009902 |
| STRAIGHT_POLICY | 5.992304773484085 | 15.606499280191949 | 15.451979485338564 | speed | 2 / 4 / 1 | 0.9900990099009895 |

所有运动实例 `rescalings=0`，最大 T 为 `20.842263324805067 s`，远低于
600 s 工程预算。T 与首轮 G3 逐位相等；纠正后的 z3 没有使任何一条路径
的 T 改变，也没有放宽任何限值或容差。

### 6.2 审核计数与点态覆盖

每个运动实例：64 个闭区间叶节点、909 次审核检查（首轮 845 次 + 64 个
点态叶中点检查），三辆车共 192 个叶-车组合。九条模型合计 512 个运动叶
点态证书、1536 个叶-车包含性检查。`NO_MOTION_HOLD` 为显式零时长 hold，
16 次检查，无运动证书和点态叶检查。

| 实例 | 最大点态 jerk 比 | 最大分量比 | 最大 `p_sss` 比 | 最坏记录 |
|---|---:|---:|---:|---|
| E1 | 0.9519608293506188 | 0.9986857877358029 | 0.9951789008465938 | jerk_x, 车辆 1, segment.4[0,0.125] |
| E2 | 0.9519608293506182 | 0.9986857877358027 | 0.9951789008465944 | jerk_x, 车辆 1, segment.4[0,0.125] |
| E3 | 0.9375904586411317 | 0.9956079921953904 | 0.993147391472375 | jerk_x, 车辆 1, segment.4[0,0.125] |
| E4 | 0.937590458641162 | 0.9956079921953883 | 0.9931473914723405 | jerk_x, 车辆 1, segment.4[0,0.125] |
| E5 | 0.9140728162817655 | 0.9877201288197411 | 0.9921069856825946 | p_sss_y, 车辆 3, segment.7[0.875,1] |
| E6 | 0.9140728162818278 | 0.9877201288197354 | 0.9921069856825567 | p_sss_y, 车辆 2, segment.7[0.875,1] |
| STRAIGHT_LINE | 0.999791589461371 | 0.9997915894625576 | 0.9879828837352729 | jerk_x, 车辆 1, segment.3[0.875,1] |
| STRAIGHT_POLICY | 0.8150816356239055 | 0.9718054244580129 | 0.98220514376589 | p_sss_x, 车辆 3, segment.2[0,0.125] |

所有比均不大于 1。节点左右极限也逐侧记录了 `jerk_ratio`、
`component_ratio`、`p_sss_ratio`；例如 E1 节点 6 右侧最坏为
`p_sss_y=0.9999999999994765`，仍包含。节点的近 1 比值来自闭区间端点
处实际值接近外扩上界，不是超限。

### 6.3 模型摘要

九条最终模型摘要（`benchmark/results.json` 中的 canonical model JSON）：

```text
E1              19d676e6e8b9ac6a9a3740e45d9b802b6b9715b98b27b655aa3559347b7922b4
E2              b53273d2b6aab168bb3bd53351edb664df94f6a6084b45aa208d5fc712a69f2b
E3              0bcec22ba51f0a3eb9e07c1fd0fde343102c9f16b07fc8496ee3ae354c3817cc
E4              abe3bfdfcec9ff0bdfae59b6ea20eb2de28b3ecb3f877df43e254729199ea3fc
E5              67401452fb51b3933e202a8f3ac671192d440f6f68de2c28821c4ce33e9ee5dc
E6              35d7e03c5ae2cfeebb68c62ec6de0a422e6550f59f63062912456ac4e61c0469
NO_MOTION_HOLD  39802144c887e534b1897a56836b389739fcfd90215d10161f0fede3a924660a
STRAIGHT_LINE   c6c0456a11872077d47c62fde1a8fcb6077397e39da476e38c355107b65d4277
STRAIGHT_POLICY b8a2d53c8d0f30adbb77d951e54e809660637e1213d3eaaf59cafcacefdfe0ea
```

九个模型摘要与首轮 G3 全部逐位相等；`S/T` 也逐位相等。原因不是忽略
jerk 修复，而是这八条路径的绑定时长约束均为 speed，且修复后的 z3 区间
只收紧、不放大，未改变任何 binding metric。

## 7. 测试、benchmark 与 legacy 真相

### 7.1 测试

| 套件 | 首轮 | 本次 | 结果 | 墙钟 |
|---|---:|---:|---|---:|
| timing | 16 | 18 | OK | 0.371 s |
| trajectory | 23 | 23 | OK | 0.447 s |
| audit | 44 | 51 | OK | 3.449 s |

新增 timing 测试：Codex 退化反例、24 个窄区间分量/范数覆盖。新增 audit
测试：退化反例、窄区间、512 个叶点态 vs 局部区间、节点左右极限证书、
侧车 jerk 超限拒绝、粗区间失败关闭/细分。原有测试未改宽或删除，所有
G1/G2 测试源码保持不变。为验证保留性，重新运行未修改的 G1/G2 套件：

| 保留套件 | 测试数 | 结果 | 墙钟 |
|---|---:|---|---:|
| `test_virtual_a_geometry.py`（G1） | 20 | OK | 1.142 s |
| `test_virtual_a_optimizer.py`（G2） | 26 | OK | 50.735 s |
| `test_virtual_a_profile.py`（G2） | 22 | OK | 1.305 s |

三组退出码均为 `0`，日志在 `logs/g1_g2_regression.log`。它们没有调用
G3 retimer 的修改路径之外的接口，证明 G1/G2 源码和测试未被改造。

### 7.2 Benchmark

`tools/virtual_a_benchmark.py --stage G3` 退出 `0`，151/151 checks
均通过，0 failed，9 个 timed model（8 motion + 1 exact hold）。
`benchmark/summary.json` 的 `dynamic_feasible=9` 包含 exact hold；
`results.json` 的 `aggregate.dynamic_feasible_cases=8` 只统计运动实例。
两者不是矛盾，报告中按此区分。

### 7.3 Legacy

未修改的 G0 gate 退出 `1`，BLOCKED 于
`UNKNOWN_WORKSPACE_MODIFICATIONS`（G1/G2/G3 返工任务书和 G2 夹具）。
仅把这些路径加入 G0 **内存 allowlist** 的披露 wrapper 退出 `1`，整体
`FAIL`：4 个 case 通过，`timing_continuous` 失败。失败是 HEAD 既有问题：

```text
tests/test_continuous_retiming.py:271
KeyError: 'dynamic_peaks'
```

pristine HEAD 的 `git archive` 探针独立复现退出 `1`，失败测试和源码哈希
绑定在 `results.json`。旧测试和旧源码未修改、未放宽；这里如实记录 FAIL，
没有把它写成 PASS。

## 8. 保护差异与源码哈希

### 8.1 相对 Codex 已接受的 G2

```text
G2 protected tree sha256 = fa1b0596f7b9ecef616512785ab65ef2b93884dfde3a9c2a9794a9f7ac222f67
G3 protected tree sha256 = 9bee7d7a483418877277fe63c83986363f7ffc114f03db6ff265927092d89fba
新增 4 个受保护文件，修改 5 个受保护文件，删除 0
only_authorized_changes = true
```

新增：

```text
timing.py              3ec16bfe6fd0d3f6860f46a2ec1d0939d521edd88b69c860f891b8bfb0c42ab8
trajectory.py          3f5c770ab7f1bcbf3e94aed7c96b07f11d839d7e8cc70ab0283f1a2c08aed491
test_virtual_a_timing.py   3969ccf4ab0f573caf9583d1024765e1b3e12cd89818a8da02949b2dd318f88c
test_virtual_a_trajectory.py d3c595a3c7a6fa2097332757a4e7c5016a701fe25b505f336a89dd47fe3e63e0
```

修改（G2→G3）：

```text
audit.py       2c7da5165498a4fd5273dea336bf51c4b652f4c1cd5b31615f78b34367a7a58d
            -> 819a44e85a578ab91271b2ad5da1d332dcf921e483dfc946296834308415097b
bounds.py      3c12cca5e062c6c35423feb0437eadf7700f72f4666822923a471b3644e8c650
            -> 0f1c47c7a1dc8614b213a5d4828e7f6d012e7a39f42077a5df5f26b2d323d6d2
contracts.py   27b101f4a8143f549cab67794999a5fb7a9d9a66e253e3db0518bd0745a1ec69
            -> 0fb50cf78bfdbe5546438e28530b7cc6c7344e3c3e810ee77874cace2ca18497
test_virtual_a_audit.py 979b5d69df56bb28751c6491971716e4d89b2e69d6476d0d18407c4f909c8503
            -> 7708c3b51404d810d6c3c055ee5979cd5401def04a892021f97c1b0267eb582f
virtual_a_benchmark.py 92368b1714fdee494fd31a2f57270148904ef87893b5b95cb5961647715a9cad
            -> 019430a0e9212734c6ab9795178bf036677bb1d750e09d608053310a738a26f2
```

其中 `bounds.py`、`contracts.py`、`virtual_a_benchmark.py` 的 G2→G3 变化
来自首轮 G3 的已授权改动，不是本次返工额外修改；本次返工相对首轮 G3
只改了 `audit.py`、`timing.py` 和两个测试文件。

### 8.2 相对首轮 G3

```text
首轮 G3 source file count = 30
本次 source file count   = 31（增加用户返工任务书 CLAUDE_G3_REWORK_01.md）
相对首轮 G3 修改的文件   = 4
新增/删除                = 0/0（受保护树）
```

逐文件旧→新：

```text
audit.py
  49543d758a771f22dfee0d96b3908c68c9e68b48dfcde3dabb301440af4425d9
->819a44e85a578ab91271b2ad5da1d332dcf921e483dfc946296834308415097b
timing.py
  6e87a867c498839f958dad09f23c566445c78a54bee5c81859d28d4e450a8749
->3ec16bfe6fd0d3f6860f46a2ec1d0939d521edd88b69c860f891b8bfb0c42ab8
tests/test_virtual_a_audit.py
  eb66f34e887634c73b0fba536e95f5347945536a4a0623ef59f13b138e7ba614
->7708c3b51404d810d6c3c055ee5979cd5401def04a892021f97c1b0267eb582f
tests/test_virtual_a_timing.py
  3cb5fe13d272b0938f918e4c4271830713b7987c54f4b6e6bb478bcea4a13004
->3969ccf4ab0f573caf9583d1024765e1b3e12cd89818a8da02949b2dd318f88c
```

其余 26 个首轮 G3 源码文件逐位未变。31 个源码文件、全部运行证据（除
自引用的 manifest 与两份 manifest 日志）以及 G2 接受摘要由
`source_manifest.json` 绑定；独立 verifier 重算。

## 9. 数值证明边界

必须区分三层：

1. **连续区间证书**：代码对每个闭单元做区间乘加，并在选定的上/下界处
   调用 `_next_up`/`_next_down` 外扩。若把实现中的 binary64 运算视为
   精确实数、把 `_next_up/_next_down` 视为正确 directed rounding，则
   证书覆盖整个连续单元；它不是仅采样。实现并未使用完整的 IEEE-754
   directed-rounding 库，因此这是“按代码的外扩规则得到的浮点区间证书”，
   不是机器可验证的精确实数证明。
2. **独立点态数值**：`analysis/g3_rework_numerics.py` 用独立手写公式、
   独立 200 次二分反解（残差 <= 1e-14）计算实际 `p_sss`/jerk；这些值
   在 512 个生产叶中点、正例的 20+3 点/路径和负例的 20,001 点扫描上与
   区间证书对照。它们是数值见证，不是全域证明。
3. **代数推导**：`p_sss` 的符号由上一节的手工微分确定；区间实现与点态
   公式的一致性由独立测试和窄区间覆盖验证。

因此本报告不把“采样最大值未超限”当成全域证明，也不把“两个实现一致”
当成正确性证明。全域结论依赖区间证书；数值扫描用于发现低估、构造负例
和验证失败关闭路径。

## 10. 非声明与停止状态

- `collision_checked=false`，`executable=false`；没有地图、碰撞、ROS/
  Gazebo、硬件、实时或时间最优声明。
- `DYNAMIC_FEASIBLE` 只是独立动态审核结果，不是 Codex `ACCEPTED`，也
  不是可执行轨迹。
- 未进入 G4、地图、ROS、时间最优；未提交 Git，未推送，未创建分支。
- 本报告和证据等待 Codex 复核；不自行写 `ACCEPTED`。

## 11. 命令与退出码

精确命令见同目录 `COMMANDS.md`，退出码见
`logs/exit_codes.log`。关键结果：

```text
timing 18 tests            exit 0
trajectory 23 tests        exit 0
audit 51 tests             exit 0
g3_rework_numerics         exit 0
benchmark 151/151 PASS     exit 0
legacy unmodified gate     exit 1 (BLOCKED, expected)
legacy disclosed wrapper   exit 1 (overall FAIL, pre-existing case)
pristine-HEAD probe        exit 1 (KeyError: 'dynamic_peaks')
protection diff            exit 0
environment                exit 0
assemble evidence          exit 0
```

证据文件：

```text
results.json
environment.json
analysis/g3_rework_numerics.json
analysis/protection_diff_vs_g2_accepted.json
benchmark/results.json
benchmark/summary.json
legacy/regression_results.json
logs/*.log
source_manifest.json（生成后由独立 verifier 重算）
```