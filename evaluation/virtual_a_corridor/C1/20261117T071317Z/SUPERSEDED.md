# SUPERSEDED：本 run 不是最终 C1 证据

本目录是 C1 开发过程中的中间证据 run，生成于 `20261117T071317Z`，已被
`../20261117T073007Z/` 取代。

原因：本 run 之后，`corridor.py` 的独立审核又收紧了三处，本目录的日志、
`sample_chain.json`、`measurements.json`、`results.json`、`source_hashes.txt`
和 `SUMMARY.md` 均对应收紧前代码：

1. 审核新增“每条多边形边必须落在某条存储半空间边界上”，并与顶点满足全部
   存储半空间一起，证明存储 `A,b` 与审核多边形是同一区域；防止 C2 消费的
   `A,b` 比 SAT 审核的多边形更松而放过障碍。
2. 地图包含改为闭地图精确比较，不再允许 `1e-9 m` 容差；顶点越界
   `5e-10 m` 也会被拒绝。
3. Portal Chebyshev LP 使用字面 `||A_row||` 范数列，并从 LP 中心的实际
   slack 重新计算认证半径，不再直接信任 HiGHS 目标值。

本目录文件保持原样，作为开发历史保留，不得作为 C1 验收证据引用。最终证据
见 `../20261117T073007Z/SUMMARY.md`。