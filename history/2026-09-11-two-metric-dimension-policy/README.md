# 双指标降维策略快照（2026-09-11）

本目录记录基于用户本地最新包 `主被动融合系统-异构(1).zip` 的本次策略修正。

## 基线与产物校验

- 输入包 SHA256: `bcd93dc6d9d72ca6d68d5072725f5622c7faad264a0c7ae304e4a1c9fe420ca8`
- 修改后完整包 SHA256: `319769ab079649ee1243f5f18524a0deeff2072432cc72c64b2b1eb19e5f30fe`

## 核心变化

3D→2D 的降维请求恢复为两个物理指标：

1. 径向距离 1-sigma 不确定度 `radial_sigma_m`；
2. 最近一次可靠主动三维距离更新的龄期 `range_age_s`。

默认阈值：

- `joint_radial_sigma_warn_m = 5000`
- `joint_radial_sigma_drop_m = 10000`
- `joint_range_age_warn_s = 3`
- `joint_range_age_drop_s = 10`

预警采用 OR：任一指标超过 warn 即进入预警；正式降维申请采用 AND：两个指标同时超过 drop 才提出降维请求。真正切换仍遵守现有 transactional pending/commit：二维分支必须 ready 且 2D/3D 一致，pending 不隐藏原 owner，也不提前改写历史 ID。

`position95`、`radial95`、相对距离不确定度和 NIS 继续计算和记录，但只作为诊断，不直接参与 3D→2D 决策。

`last_valid_range_t` 只在真正成功完成主动三维距离/空间更新时刷新；被动 AE 或主动 AE-only 不刷新该时间。

## 文件

- `two_metric_dimension_policy.patch`：相对于该输入包的完整统一 diff，可直接复现本次修改。
- `二维三维升降维策略说明.md`：策略与状态语义说明。

说明：当前连接器不能把本地 ZIP/目录直接作为 GitHub 文件参数批量上传，因此 GitHub 保存的是可重现的完整修改补丁和策略说明；修改后的完整源码 ZIP 由会话附件提供。未在此环境运行 MATLAB/Octave 数值回归，仅完成源码级静态检查。
