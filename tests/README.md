# 可移植的离线检查

从仓库根目录运行：
```sh
python tests/offline/test_decoder.py
python tests/offline/test_field_analyzer.py
node tests/offline/test_web_math.cjs
```

这些测试自动生成虚拟数据，不连接或改写实物板子，不包含个人测量记录。

`native/` 保留已用 GCC/AddressSanitizer/UndefinedBehaviorSanitizer 验证过的 C++ 测试源。纯头文件测试可用宿主机 g++，例如：
```sh
g++ -std=c++17 -O1 -g -fsanitize=address,undefined -Ifirmware/insmini-nav/src tests/native/field_button_test.cpp -o field_button_test
./field_button_test
```
其余测试按所需源加入 `navigation.cpp`、`lib/PSINS/src/PSINS.cpp` 和对应 include 路径。离线通过不代替真实 RF、PPS、卡满或导航精度实测。
