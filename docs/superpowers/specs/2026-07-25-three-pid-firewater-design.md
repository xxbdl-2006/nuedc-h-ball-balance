# 三 PID 角度控制与 FireWater 波形设计

日期：2026-07-25

## 目标

将当前“公共速度 PID + 角度差速修正”重构为与参考代码一致的三级控制结构：一个角度 PID 产生左右轮差速修正，公共速度目标负责整车前进或后退，合成后的左右轮目标再分别通过独立速度 PID 输出电机 PWM。控制周期固定为 50 ms。

PID 核心只保留参考代码中的位置式和增量式计算公式，去除当前实现中不需要的死区、独立积分累计变量、积分限幅、内部时间节流和速度 IIR 滤波。代码使用简洁、与实际行为一致的中文注释。

UART0 蓝牙链路按 FireWater 文本格式输出左轮有符号目标速度和左轮有符号实际速度。串口发送在主循环执行，不在 50 ms 定时器中断中阻塞。

## 已确认的工程事实

- SysConfig 中 `TIMER_TICK` 的实际周期为 50 ms。
- `encoder_update()` 每个定时周期快照并清零临时计数，`get_encoder_count_L/R()` 已返回该周期的有符号编码器增量，不是长期累计值。
- `set_left_PWM()` 和 `set_right_PWM()` 接受有符号整数；符号控制方向，绝对值控制 PWM，公开范围为 0..1000。
- UART0 已由蓝牙模块使用，`Bluetooth_Poll()` 在主循环执行协议解析和状态发送。
- UART0 当前为 9600 baud。每 50 ms 发送两个短整数能够满足带宽要求。
- JY61P/WIT 使用另一串口，本设计不改变其串口或功能。
- 本设计不修改 `.syscfg`，也不手工修改 `Debug` 下的生成文件。

## 方案选择

采用“50 ms 中断计算、主循环发送”的方案。

- 定时器中断只完成编码器快照和三路 PID 计算，保证控制周期稳定。
- `Bluetooth_Poll()` 读取最近一次完成的控制状态并发送 FireWater 行。
- 不在中断中调用阻塞式 UART 发送。
- 不引入 UART DMA、遥测队列或新的硬件外设，避免超出本次需求。

## 模块职责

### `pid.c/.h`：通用 PID 计算核心

`pid_t` 保留以下字段：

- `target`、`now`：目标值和反馈值。
- `error[3]`：本次、上次、上上次误差。
- `p`、`i`、`d`：控制参数。
- `pout`、`iout`、`dout`：三个分量输出。
- `out`：PID 总输出。
- `pid_mode`：`POSITION_PID` 或 `DELTA_PID`。

除 `pid_mode` 使用 `uint32_t` 外，目标、反馈、误差、参数和全部输出字段均使用 `float`。所有公式按单精度浮点运算，`pid_cal()` 返回 `void`，计算结果从结构体字段读取；在角度目标和 PWM 最终输出之前不做整数运算。

公开接口：

- `pid_init(pid, mode, p, i, d)`：清零全部运行状态并设置模式和参数。
- `pid_reset(pid)`：清除误差历史和输出，保留目标、模式及参数。需要停车时由上层先显式清零目标。
- `pid_cal(pid)`：按照模式执行一次 PID 计算。
- `pidout_limit(pid, min_out, max_out)`：独立限制输出，不改变 PID 计算公式。

位置式公式与参考代码一致：

```text
error = target - now
pout = p * error
iout += i * error
dout = d * (error - last_error)
out = pout + iout + dout
```

增量式公式与参考代码一致：

```text
error = target - now
pout = p * (error - last_error)
iout = i * error
dout = d * (error - 2 * last_error + last_last_error)
out += pout + iout + dout
```

每次计算结束后按 `error[2] <- error[1] <- error[0]` 更新历史误差。

`pidout_limit()` 只把 `pid->out` 钳位到调用方给出的闭区间，不修改 `iout`、误差历史或其他分量。位置式 `iout` 继续像参考代码一样直接累积，不增加抗积分饱和；这是有意保留的参考算法行为。限幅顺序固定为“先 `pid_cal()` 完成全部分量和总输出，再调用 `pidout_limit()`”。

`pid.c/.h` 不包含电机、编码器、UART、时间或蓝牙依赖。

### `pid_control.c/.h`：三 PID 控制桥接层

模块拥有三个文件内静态 PID 实例，不允许其他模块直接改写：

- `angle_pid`：位置式 PID，输入角度目标和 BNO08X yaw，输出可正可负的左右轮差速修正。
- `motor_left_pid`：增量式 PID，控制左轮速度。
- `motor_right_pid`：增量式 PID，控制右轮速度。

初始化行为：

- 三路 PID 参数均保持当前安全策略，上电默认 `P=I=D=0`，不擅自写入未经实车验证的参数。
- 角度目标默认为 0°，三路输出和两路 PWM 均清零。
- 公共有符号速度目标默认为 0，允许范围为 `-200..200` 个编码器脉冲/50 ms。
- 角度环输出限制为 `-200..200` 个编码器脉冲/50 ms，沿用工程现有速度目标范围。
- 两个速度 PID 输出限制为 `0..1000`，与电机驱动公开 PWM 范围一致。

`pid_control_update(current_angle, enc_left, enc_right)` 每次调用只执行一轮控制，不再检查 `tick_ms`，也不再计算编码器二次差分或 IIR 滤波。调用流程如下：

1. 如果角度 PID 的 `P/I/D` 不全为 0，则设置 `angle_pid.now = current_angle`，计算角度 PID，并把角度输出限制在 `-200..200`。如果三项参数全为 0，则跳过角度计算、复位角度 PID 的积分和误差历史，并强制本轮角度修正为 0，避免位置式 PID 保留旧的 `iout/out`。
2. 使用用户确认的叠加方向计算两个浮点轮速目标：

   ```text
   left_target  = common_speed_target - angle_pid.out
   right_target = common_speed_target + angle_pid.out
   ```

3. 分别把合成后的左右轮目标限制在 `-200..200`，再按“正数加 0.5、负数减 0.5”对称四舍五入为整数，然后调用 `motor_target_set(left_target, right_target)`。限幅作用于每个车轮的最终目标，不反向修改公共速度目标或角度 PID 输出。
4. `motor_target_set()` 保存每个车轮的方向，并把速度 PID 的目标转换为非负幅值。
5. 编码器反馈已经是有符号的“脉冲/50 ms”；`get_encoder_count_R()` 已完成右轮符号归一化，不再次取反。非零目标把原始有符号反馈投影到目标方向坐标：`projected_now = raw_now * sign(target)`，同向运动为正、反向运动为负，再分别写入两个速度 PID 的 `now`。速度 PID 的目标仍是非负目标幅值，因此车轮反向运动时 PID 会看到更大的正误差。
6. 分别计算左右速度 PID，并将输出限制在 `0..1000`。
7. 速度 PID 的负输出在第 6 步被钳位为 0，不通过反向 PWM 主动制动。非负输出按“加 0.5 后转为 `int`”四舍五入为 0..1000 的 PWM 幅值，再用已保存方向恢复符号并调用 `set_left_PWM()`、`set_right_PWM()`。
8. 在所有计算完成后发布一次状态快照，供 OLED 和蓝牙读取。

零目标采用明确的安全规则：该轮方向记为 0，速度 PID 目标设为 0，跳过本轮 PID 计算、复位该轮 PID 运行历史并立即输出 PWM 0。状态快照中的实际速度仍记录编码器返回的原始有符号值，以便 FireWater 显示停车后的惯性转动。零目标不保留上一次方向。

三种典型组合的语义固定如下：

- 先调速度环：把角度 `P1/I1/D1` 全部设为 0，再用 `ST/T2/Ot/T` 设置公共速度目标，此时左右轮目标相同。
- 原地角度控制：设置 `ST=0!`，此时左右轮目标分别为 `-angle_pid.out` 和 `+angle_pid.out`。
- 两环同时启用：公共速度负责前进/后退，角度 PID 输出负责在公共速度上产生左右差速。

`pid_control_stop()` 清零公共速度目标和角度目标，复位三个 PID，并立即将两路 PWM 置零。它是明确的安全停止入口。

### 状态接口与并发

`pid_control_status_t` 的字段和类型固定为：

```c
float angle_p, angle_i, angle_d;
float motor_p, motor_i, motor_d;
float angle_target, angle_now, angle_out, speed_target;
int32_t left_target, left_now, left_output;
int32_t right_target, right_now, right_output;
uint32_t update_count;
```

`speed_target` 是公共有符号速度目标；`left/right_target` 是公共速度与角度修正合成并限幅后的最终有符号轮速目标；`left/right_now` 是编码器接口返回的原始有符号周期增量；`left/right_output` 是最终传给电机驱动的有符号 PWM。初始化时整个快照清零。每完成一轮三 PID 控制，`update_count` 自增一次；它只用于诊断，自然按无符号 32 位回绕，任何控制逻辑都不依赖它。

50 ms 中断在一轮控制结束后更新一个 `volatile pid_control_status_t` 状态快照。主循环调用 `pid_control_get_status()` 时使用工程现有的临界区模式：保存 `__get_PRIMASK()`、执行 `__disable_irq()`、复制完整快照，并且只在进入前中断已开启时调用 `__enable_irq()`。复制结束后立即恢复中断；格式化和 UART 发送不在临界区内执行。

蓝牙同时修改两个速度 PID 参数时也使用相同的短临界区，在一次临界区内更新左右两侧参数，避免某个控制周期只更新一侧。`pid_control_set_angle_target()`、`pid_control_set_speed_target()`、`pid_control_set_gain()` 和 `pid_control_stop()` 等所有可能在主循环修改 ISR 所用状态的公开函数都必须使用同样的 PRIMASK 临界区；初始化发生在控制定时器开始工作前，不需要额外临界区。角度参数更新后如果三项参数已经全为 0，必须在同一临界区内立即复位角度 PID 运行历史并把发布的角度输出清零。

### `timer.c`：50 ms 调度

`TIMER_TICK_INST_IRQHandler()` 在每次 `DL_TIMER_IIDX_ZERO` 中断中执行：

```c
encoder_update();
pid_control_update(bno08x_data.yaw,
                   get_encoder_count_L(),
                   get_encoder_count_R());
```

删除误导性的 20 ms 注释并启用控制调用。定时器中断不发送 UART 数据。

中断先完成 `encoder_update()`，因此随后读取的左右值在本轮内保持稳定。`bno08x_data.yaw` 是对齐的 32 位浮点量，中断入口只读取一次并作为函数参数传入；主循环更新 yaw 时不会出现半字写入。与旧竞赛任务有关的其他全局量和代码不在本次清理范围内。

### `bsp_hc05.c`：FireWater 输出与调参

保留现有主循环解析方式和 `CSV` 开关：

- `CSV=1!`：开启波形输出。
- `CSV=0!`：关闭波形输出。
- 上电默认关闭，避免蓝牙连接后未经请求持续刷屏。
- 开启后每 50 ms 最多发送一行，发送最近一次完整控制快照。
- 将当前 `BLUETOOTH_CSV_PERIOD_MS` 从 200 改为 50。

收到 `CSV=1!` 时记录当前 `tick_ms`，第一条周期波形必须等待至少 50 ms；随后使用回绕安全的无符号差值 `(uint32_t)(now - last) >= 50U` 判断。每次 `Bluetooth_Poll()` 最多发送一行并把 `last` 更新为当前时间，不补发积压帧。`Q!` 不受此周期限制，收到后立即发送一行。初始化完成但首轮 PID 尚未执行时，快照为全零，因此 `Q!` 返回 `pid:0,0\n`。

FireWater 行格式固定为：

```text
pid:<left_target>,<left_actual>\n
```

示例：

```text
pid:-25,-23
```

两个数均为有符号整数，单位均为“编码器脉冲/50 ms”：

- 通道 0：左轮有符号目标速度。
- 通道 1：左轮有符号实际速度。

两个字段直接取状态快照中的 `int32_t left_target` 和 `int32_t left_now`，不在发送阶段再次缩放或舍入。

使用整数格式化，避免引入浮点 `printf` 的代码体积和工具链配置问题。UART 发送继续使用现有 `Send_Bluetooth_Data()`，且仅从主循环调用。

调参命令保持最小兼容：

- `P1/I1/D1`：角度 PID 参数。
- `P2/I2/D2`：同时设置左右两个速度 PID 的相同参数。
- `AT`、`T1`、`It`：设置角度目标。
- `ST`、`T2`、`Ot`、`T`：设置公共有符号速度目标，接收范围为 `-200..200` 个编码器脉冲/50 ms；超出范围时返回现有范围错误，不改变原目标。
- `STOP!`：停止并清零控制器。
- `Q!` 返回与 FireWater 相同的最近一组左轮目标/实际数据。

实现时拆分并替换当前六字段状态发送逻辑：`Q!` 和周期 CSV 都调用新的 FireWater 两字段发送函数，固定使用单个换行符 `\n`，不再输出旧的 `\r\n` 六字段行。恢复 `ST/T2/Ot/T` 的成功分支，并调用 `pid_control_set_speed_target()`；这些命令不再进入 `ERR,CMD!` 未知命令路径。

### `oled_display.c`：状态名称同步

蓝牙/PID 页面继续显示角度 PID 参数与角度目标/反馈，并显示当前公共速度目标；原“公共速度 PID”标签改为“左右轮共同速度 PID 参数”。不增加新页面，不改按键操作，也不重构其他 OLED 页面。

## API 迁移与调用点

本次是有意进行的 PID 接口替换，所有旧调用必须一次性迁移：

- `pid.h` 删除 `error_int/max_out/max_iout/dead_zone` 和旧的 `angle_pid/speed_pid` 外部声明，增加模式字段、模式枚举和新的四个公开函数签名。
- 全工程只有 `pid_control.c` 直接调用 `pid_init/pid_reset/pid_cal/pidout_limit`；其他模块继续通过 `pid_control.h` 访问控制功能。
- `pid_control.h` 完整重写状态结构与注释，删除“20 ms、累计编码器、主循环内部节流、公共速度环、IIR 滤波”等旧说明。
- 保留 `pid_control_init()`、`pid_control_update()`、`pid_control_set_angle_target()`、`pid_control_set_speed_target()`、`pid_control_stop()`、`pid_control_get_status()` 和 `pid_control_set_gain()` 的模块级入口。
- `pid_control_set_speed_target(float target)` 将输入限制在 `-200..200` 后保存为公共速度目标；蓝牙协议先做范围校验，因此正常命令不会依赖函数内部钳位。
- `bsp_hc05.c` 和 `oled_display.c` 按新的 `pid_control_status_t` 字段编译，不保留旧 `speed_target/speed_now/speed_out` 字段的兼容假数据。
- `timer.c` 取消注释并调用新的单轮次 `pid_control_update()`；`main.c` 不重复调用该函数。
- 修改后用全工程文本检索确认不存在旧 `speed_pid`、旧 `pid_init` 签名或 20 ms PID 注释，并确认 `pid_control_set_speed_target` 只有真实的公共速度目标语义。

## 注释约定

- 使用中文说明控制量、单位、方向和调用周期。
- 注释描述“为什么”和硬件语义，不重复逐行翻译代码。
- 所有 20 ms、累计编码器、公共速度环等旧注释同步删除或改为 50 ms 的真实行为。
- 不保留乱码注释。

## 安全与边界行为

- 上电 PID 参数为零、PWM 为零，不会自动驱动电机。
- 角度输出、每轮最终速度目标和 PWM 都有独立限幅，但限幅不改变 PID 公式。
- 角度 `P/I/D` 全为 0 时强制清除角度 PID 的旧积分和输出，因此只保留公共速度控制。
- `STOP!` 清零公共速度目标并立即复位全部运行历史、关闭两路 PWM。
- UART0 波形发送失败或未及时运行不会阻塞或改变 PID 状态；下一次发送读取最新快照。
- 本阶段不增加角度跨越 `-180°/180°` 的最短路径处理，以保持参考代码的 `target - now` 误差语义。
- 物理前进方向仍依赖电机接线和编码器 A/B 相位，源码构建通过不能替代架空车轮验证。

## 修改范围

计划修改：

- `Drivers/BSP/pid.c`
- `Drivers/BSP/pid.h`
- `Drivers/BSP/pid_control.c`
- `Drivers/BSP/pid_control.h`
- `Drivers/BSP/timer.c`
- `Drivers/BSP/bsp_hc05.c`
- `Drivers/BSP/oled_display.c`
- 必要时仅同步更新 `main.c` 中的误导性注释

明确不修改：

- 工程 `.syscfg`
- `Debug` 下的生成文件
- 电机和编码器引脚配置
- JY61P/WIT 串口配置
- 蓝牙波特率
- 其他竞赛任务状态机和页面功能

## 验证

1. 用已知误差序列核对位置式和增量式 PID 的 `pout/iout/dout/out`，确保与参考公式一致。
2. 核对纯速度模式：角度 `P/I/D` 全为 0 时左右轮目标都等于公共速度目标，且旧角度积分和输出已清零。
3. 核对纯角度模式：公共速度目标为 0 时，正、负角度输出能生成左右相反的目标方向。
4. 核对叠加模式：左右目标严格为“公共速度减/加角度输出”，并分别限制在 `-200..200`。
5. 核对编码器值只使用一次，不再做二次差分。
6. 核对左右速度 PID 状态相互独立，`P2/I2/D2` 同时更新两侧参数。
7. 核对输出限幅：角度修正 `-200..200`、每轮最终目标 `-200..200`、PWM 幅值 `0..1000`。
8. 核对 `ST/T2/Ot/T` 能设置正负公共速度目标，超范围命令不改变原目标。
9. 核对 `STOP!` 后公共速度目标、三路 PID、轮速目标和 PWM 均为零。
10. 核对 FireWater 输出严格为 `pid:<target>,<actual>\n`，其中目标是限幅后的最终左轮目标，周期不快于 50 ms，且只从主循环发送。
11. 对所有改动源文件运行编译和针对性差异检查。
12. 运行 MSPM0 SysConfig 静态检查，确认本设计未改变外设配置或生成文件。
13. 如本机构建环境可用，使用 CCS 自带 `gmake` 完成 Debug 构建；构建成功只证明源码和链接通过。
14. 硬件验证时架空车轮并限流，先确认左右编码器正向符号，再从低增益和小速度目标开始调参。

## 完成标准

- 工程只有一个角度位置式 PID 和两个相互独立的轮速增量式 PID。
- 三路计算公式与参考代码一致，控制周期与 SysConfig 的 50 ms 一致。
- 公共速度目标与角度输出按“左减右加”叠加，左右轮最终目标分别限幅，编码器反馈不被二次差分。
- FireWater 通过 UART0 在启用后显示左轮目标速度和实际速度两条曲线。
- 源码注释简洁、无乱码，且单位、方向和周期与实现一致。
- 源码级检查和可用构建通过；实车效果明确留给硬件测试确认。
