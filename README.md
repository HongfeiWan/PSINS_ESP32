# PSINS_ESP32 · INSmini 硬件与固件

当前工程：**INSmini S3 R2B + navigation 3.0.4 外场试用固件**。硬件和固件于 2026-10-29 整理上传。INSmini 使用 `firmware/insmini-nav` 下的 PlatformIO 工程。

| 内容 | 入口 |
|---|---|
| 全部硬件版本：原理图、PCB、工程库、Gerber、BOM、CPL | [hardware](hardware/)（R1、R2、R2B） |
| 当前已加工硬件与 RF/制造检查 | [hardware/R2B](hardware/R2B/) |
| 当前固件完整源码与网页 | [firmware/insmini-nav](firmware/insmini-nav/) |
| 当前 3.0.4 二进制、分区、校验与受保护刷机脚本 | [prebuilt/insmini-nav-v3.0.4](prebuilt/insmini-nav-v3.0.4/) |
| 使用说明、实测进展与已知限制 | [docs/insmini](docs/insmini/) · [验证状态](docs/insmini/VALIDATION.md) |
| J4 全部引脚和扩展能力 | [J4.md](docs/insmini/J4.md) |
| PSINS/MATLAB 导入与回放适配 | [integrations/psins](integrations/psins/) |
| 数据取回、严格解码和外场分析 | [tools](tools/) |
| 离线测试 | [tests](tests/) |
| 一次性 TF 格式化维护源码 | [maintenance/tf-format](maintenance/tf-format/) |

当前板为 ESP32-S3-WROOM-1-N8 + LSM6DS3TR-C 六轴 + ATGM336H-5N31 + TF，无磁力计。已完成上电自主记录、网页姿态、GNSS/PPS 软件同步、已知初始方向下的 PSINS 松组合和严格导入；真实 GNSS 曾定位并融合，但导航精度、稳定外场轨迹和长期可靠性仍待验证。5 片中只验证了 1 片，不能据此认定全部成品或全部 PSINS 示例功能已通过。

公开原生工程仅清空了账户密码字段并核对其余设计数据一致；Gerber/BOM/CPL 等原始制造数据不变。个人 Wi-Fi 密码、NVS、定位轨迹、私有服务器资料和构建缓存不在本次提交中。第三方代码的来源和许可见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。
