# Hardware Test Suite

云台硬件测试代码汇总。每个测试项包含功能说明、使用方法和输出说明。

---

## 1. 阶跃响应测试 (Step Response Test)

**Package:** `gimbal_response_test`

**功能:** 对两轴云台（PAN/TILT）发送阶跃角度指令，记录命令和反馈数据，离线分析瞬态响应指标。

**测试流程:**
1. 云台归位到 HOME 位置
2. PAN 轴按预设角度序列依次阶跃（默认: 0° → 30° → -30° → 0°）
3. TILT 轴按预设角度序列依次阶跃（默认: 0° → 20° → -20° → 0°）
4. 每个阶跃保持 3s 等待稳定
5. 测试完成后归位并发送 STOP

**运行方法:**

```bash
# 启动测试（含 pelco_control 节点和 rosbag 录制）
roslaunch gimbal_response_test step_response_test.launch

# 测试自动完成后 Ctrl+C，分析 bag 数据
python3 gimbal_response_test/scripts/analyze_step_response.py \
  /home/loc/loc_ws/test_results/step_response_<timestamp>.bag
```

**输出指标:**
- 上升时间 (10%-90%)
- 建立时间 (±2%)
- 超调量 (%)
- 响应延迟
- 稳态误差
- PAN/TILT 阶跃时域图

**配置:** 编辑 [gimbal_response_test/configs/test_config.yaml](gimbal_response_test/configs/test_config.yaml)

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `hold_time_s` | 3.0 | 每步保持时间 |
| `pan_angles_deg` | [0, 30, -30, 0] | PAN 角度序列 |
| `tilt_angles_deg` | [0, 20, -20, 0] | TILT 角度序列 |
| `loop_rate_hz` | 50 | 指令发布频率 |
| `post_test_home` | true | 测试后是否归位 |

---

*更多测试项待补充...*
