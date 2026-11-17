# Codex增量验收矩阵

基线23fd162；审增量diff/数学/原始证据，再针对性独立重现一正一反。稳定未改部分不重复全套G0–G5；新阶段的NOT_RUN不能算通过。阶段独立结果写新CODEX_ACCEPTANCE.json，包含源码hash/证据路径/状态/限制；不重写旧记录。

| 门 | 必查P0/P1 | 最小独立验证 | 可量化标准 |
|---|---|---|---|
| C0-A | 三处前缀积分与段末，solver/失败语义 | 手算直线q、曲线对照；timeout反例 | 直线1e-12m，曲线1e-7m，目标相对1e-9 |
| C0-B | 同候选余项+guard，未偷改5e-8 | 复算一组panel预算 | 分项闭合，未证实仍拒绝；变合同另批准 |
| C1 | 全原cell/unknown/边界，portal非退化 | 单盒平面+穿角反例+独立SAT | seed违约≤1e-9m，r≥min(.005,.1res)，area>1e-10m² |
| C2-A | 不等距QP/U保留/8段任意M | M=12分配+坏圆角 | primal≤1e-8m，σ全覆盖/正确双约束 |
| C2-B | 世界坐标、双solver、连续包含/context、三层 | 一自主弯道+配点间越界/中心安全OBB碰撞 | 目标.005m/.2°，刚性1e-6，反例0误认证 |
| C3 | routes/initials/恢复/共同预算 | 调用计数+一次实际失败 | ≤3路线/6初值/1恢复；预算耗尽不启动新solver |
| C4 | 准入证据/不同车区域/有限采样局限 | 支持式对四角、跨C接缝反例 | 1e-12m公式一致，同预算一个病例收益；否则SKIPPED |
| V0 | 真实插件/尺寸/身份/标签 | 成功→失败错digest+标准窗口 | 不残留PASS；标签位置1e-9m/速度.005m/s |
| V1 | 显示就是实际C、topic原子清理 | vertices/hash/三角面积 | 位置1e-9m，面积相对1e-9，无旧id |
| V2 | A*/guide/final来源不同、标签正确 | guide实际消费和未审反例 | 不把guide当认证轨迹；frame仍同state_at |
| V3 | admit/busy/reject/map/generation/hold/clear | 真ROS新请求+失败/身份负例 | 0误PASS/残影/误播，收到事件2tick内本地更新 |
| C5 | 公平分母/配置/预算/机器/全部失败 | 抽一对A/B和失败复算 | 72记录对应、反例0误发布，不要求提升百分比 |

## 只读核对命令

```bash
git rev-parse HEAD
git status --short
git diff --stat
git diff --check
git diff -- catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a
```
已提交门用交接base SHA到current SHA完整diff，不能只看最后一commit。sha256sum抽核变化文件/原始输入/模型与accepted manifest；目录保护对上次门，不运行会覆盖历史的旧runner。定向测试环境和命令见各TASK，独立结果明确与Claude保存记录区分。

## 回归和A/B

legacy_route默认路径在C2后仍通过原冻结六场景及模型JSON往返/摘要回归；新corridor代表任务至少一条非平凡通过，不保证全部窄道成功，必须完整报告失败。新旧backend ROS按各自合同，同master不双发布。

A=C0后旧强引导，B=C3新走廊，C仅C4实施后；同地图/物理/预算/审计/硬件，允许显式算法配置不同但全部记录。离线三层通过/executable与真实ROS发布分别统计，不能互相替代。旧单车dynamic_peaks失败仍FAIL，不冒充新退化或全绿。

P0：碰撞漏检/假证书/错身份/失败误播/降低物理或审核，立即不通过；P1：配点错、伪连续包含、旧模式退化、无限恢复/预算不共享，返修当前门；P2：文本/颜色/非必要统计可登记，不反复触发全部实验。
