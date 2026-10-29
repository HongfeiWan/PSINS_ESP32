# INSmini S3 R2B

当前已加工的硬件版本，板框 39×43 mm。

- `.eprj2`：嘉立创EDA原生工程，含原理图、PCB 和工程库。
- `Schematic.pdf`：原理图阅读版；`Gerber.zip`、`BOM.csv`、`CPL.csv`：配套生产文件。
- Gerber ZIP、BOM、CPL、PDF 保持本地交付文件的原始字节；坐标原点、负 Y、角度和层序均未修改。
- 原生工程公开副本仅清空 `users.password` 并执行 SQLite VACUUM；其余全部表的数据指纹一致，详见上级目录 `NATIVE_PROJECT_PUBLICATION.json`。
- `ORIGINAL_SHA256SUMS.txt` 是历史交付校验表。公开副本以本目录新生成的 `SHA256SUMS.txt` 为准。
- 历史说明和检测报告保留当时状态；最新实测边界见 `../../docs/insmini/VALIDATION.md`。
