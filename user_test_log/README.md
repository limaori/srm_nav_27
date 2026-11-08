# user_test_log —— 现场测试产物

放**人工测试**产生的数据与图片，不参与 `colcon build`，不进版本库。

| 子类 | 说明 |
| --- | --- |
| `*.png` | 波形图（如 `lio_yaw_*.png`：LIO 航向 vs Gazebo 真值） |
| `*.csv` | 采样数据（与同名 PNG 配套，每行一个采样点） |
| `*.log` | 终端输出 / 节点日志片段（排查时留的证据） |

## 命名约定

`<工具>_<主题>_<日期>_<时间>.<后缀>`，例如

```
lio_yaw_20261105_0231.png
lio_yaw_20261105_0231.csv
```

`debug_tools/plot_lio_gt_yaw.py` 默认就写到这里（文件名带时间戳，不会互相覆盖）：

```bash
./debug_tools/plot_lio_gt_yaw.py --headless          # 默认落到 user_test_log/
./debug_tools/plot_lio_gt_yaw.py --save /tmp/xx.png  # 也可以指定别处
```

## 注意

- 这里的东西**随时可以删**，不承载任何代码或配置。
- 需要长期保留的结论（而不是原始数据）请写进 `docs/`。
