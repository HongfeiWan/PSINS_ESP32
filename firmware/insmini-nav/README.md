# INSmini navigation 3.0.4

当前 R2B 板上运行的 PlatformIO/Arduino 固件源码。发布二进制在 `../../prebuilt/insmini-nav-v3.0.4/`，应用 SHA256：
`1af701f4f21df6d8db2bc9d11b5ca936050d16ede9b65a5e5fc6ac47ff5fee7a`。

从仓库根目录构建：
```sh
pio run -d firmware/insmini-nav -e r2b
```
固定平台 `espressif32@6.12.0`、Arduino 2.0.17，ESP32-S3 8 MB、DIO/80 MHz、原生 USB CDC。项目本地 `lib/SD` 含已验证的块传输后让出 CPU 修复，保留看门狗；不要替换为未修复的全局库。

Wi-Fi 启动读取 ESP32 NVS 的已保存配置，源码和发布镜像不包含个人无线密码。新板首次连接 USB，打开串口 115200，发送 `c` + SSID + 一个真实 Tab + 密码 + 换行；程序保存配置并申请连接。不要把密码写入公开源码、命令日志或 Git。

上电完成约 5 秒静止校准后自主记录 TF，联网等待也有超时；建议放稳约 15 秒并以红灯慢闪确认。长按 BOOT 两秒请求停录，等待常亮表示缓存排空且结束记录成功。TF 采用 INL3/120 字节记录、CRC、约 64 MiB 分段；卡满不覆盖旧文件，没有提前容量预警和预留收尾空间机制。

PSINS 为已知初始真北方向下的 SINS/GNSS 松组合，或 +Y 与前进方向一致时的 GNSS 航迹初始化。六轴没有磁北观测；单天线静止不能自动确定机身方向。模拟模式明确标记并与 REAL 数据隔离。

详见 `../../docs/insmini/VALIDATION.md`、使用说明和 J4 引脚表。当前是工程外场试用固件，尚未完成导航精度、满卡和长期掉电可靠性验收。
