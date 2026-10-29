# 3.0.4 外场试用固件

`flash-images/firmware.bin` 与当前已验证板上镜像相同，964032 字节；所有文件见 `MANIFEST.json`。源码位于 `../../firmware/insmini-nav`。

`flash_current_board.py` 是原交付的受保护更新脚本，只允许已验证的 MAC `80:45:6b:44:07:68`，校验现有分区后仅更新应用 `0x10000`，不擦 NVS。它不会自动适用于其他四片板；其他板须先确认型号、MAC、分区和连接，再采用对应刷机流程。不要为跳过保护而盲改 MAC。

安装 `requirements-flash.txt` 后，在本目录运行 `python flash_current_board.py`。`--rollback` 使用随附 3.0.3 回退应用。此脚本是旧板应用更新流程，不是首次空白 Flash 初始化流程；发布了原有引导/分区文件供核对，但本次未执行任何设备刷写。

3.0.4 已做实物冷上电自动记录、真实传感器 CRC 记录和模拟导航测试。历史冻结报告中的物理步骤 pending 不改写，后续补充结论见 `../../docs/insmini/VALIDATION.md`。真实性能与限制按该文档区分。
