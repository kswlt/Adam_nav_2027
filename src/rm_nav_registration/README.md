# rm_nav_registration

Unified small_gicp and KISS registration adapters.

`SmallGicpBackend` 已调用真实上游 helper 库，并通过已知变换与异常点云测试。
接口接收 target/source 点云及初值；输出变换、Hessian、欧氏 RMSE、内点率和耗时。
算法与度量说明见 [后端验收](../../docs/small_gicp_acceptance.md)。
`KissGicpBackend` 已调用真实 KISS 粗配准和 GICP 精化；调用者必须提供受限候选区域。
较大修正保持 CANDIDATE，不自动发布 TF，见 [KISS 验收](../../docs/kiss_gicp_acceptance.md)。

`registration_types.hpp` is the shared result contract for local GICP, KISS coarse registration, AMCL/GICP and loop closure. It intentionally does not implement a registration algorithm or accept a result based only on a library `converged` flag.

`localization_validator.hpp` 提供有限值、SE(3)、内点、残差、退化、幅度与置信度门限。
大幅修正返回 `CANDIDATE`；独立确认、停止确认与重规划提交事务仍待实现。

`registration_backend.hpp` defines the common backend boundary. The unconfigured backend intentionally returns an invalid result so the caller must use a fallback instead of accepting a fabricated pose.
