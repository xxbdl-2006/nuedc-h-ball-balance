# NUEDC 2026 MSPM0G3507 Board Test

This CCS project is a formal bring-up copy of the previous contest project.
It is currently focused on separate bring-up pages for IMU, gray sensor,
motor/encoder, and JDY31 Bluetooth tests, with low-flicker OLED updates.

## Pin/Driver Coverage

- OLED: PA1 SCL, PA0 SDA
- UART0 JDY31 Bluetooth/debug: PA10 TX, PA11 RX, 9600 baud
- UART1 K230/MaixCAM: PA8 TX, PA9 RX
- UART2 WIT/JY61P: PB15 TX, PB16 RX, 115200 baud
- UART3 Zhangdatou stepper: PB12 TX, PB13 RX
- BNO08X/BNO085 I2C: PB2 SCL, PB3 SDA
- Gray sensor: PB19, PB17, PA16, PA14, PB20, PB25, PA25, PA27
- Board keys: PB23, PB27
- Buzzer: PA29
- Encoders: left PB26/PA13, right PB5/PB4
- TB6612 motor pins and PWM remain configured. The Bluetooth protocol has no
  `RUN` command; use `STOP!` to clear targets and stop both motors.

## OLED Pages

Press KEY1 to switch pages. KEY2 runs the page-specific test action.

- Page 0: BNO08X/BNO085 I2C/SHTP stream on PB2/PB3.
- Page 1: JY61P/WIT UART2 angle stream on PB15/PB16.
- Page 2: 8-channel gray sensor digital input state.
- Page 3: TB6612 motor/encoder diagnostic modes.
- Page 4: JDY31 Bluetooth UART0 RX/TX counter and last-text display.

UART0 also provides diagnostic helpers, but the board-test main loop does not
enable the optional one-line-per-second logger so it will not pollute tuning
frames by default.

## Bluetooth Speed-Loop Tuning

UART0 now accepts `!`-terminated ASCII commands for two independent wheel
speed loops. The speed value is currently **encoder counts per 50 ms**, not
RPM or vehicle speed. Kp/Ki/Kd default to the values used by the previous
contest project: `1.37 / 0.40 / 0.15` on both wheels.

The reference tuning protocol is retained, with channel 1 assigned to the
left wheel and channel 2 assigned to the right wheel:

| Command | Effect | Accepted range |
| --- | --- | --- |
| `P1=<v>!` | Left Kp | 0..20 |
| `I1=<v>!` | Left Ki | 0..10 |
| `D1=<v>!` | Left Kd | 0..20 |
| `It=<v>!` or `T1=<v>!` | Left target | -200..200 counts/50 ms |
| `P2=<v>!` | Right Kp | 0..20 |
| `I2=<v>!` | Right Ki | 0..10 |
| `D2=<v>!` | Right Kd | 0..20 |
| `Ot=<v>!` or `T2=<v>!` | Right target | -200..200 counts/50 ms |
| `T=<v>!` | Set both targets | -200..200 counts/50 ms |
| `STOP!` | Stop immediately and clear both targets | no value |
| `Q!` | Send one status sample | no value |
| `CSV=1!` / `CSV=0!` | Enable/disable 5 Hz status stream | 0 or 1 |

Successful setting commands reply with `OK!`; malformed, unknown, oversized,
or out-of-range frames reply with `ERR,<reason>!`. CR/LF after a frame is
ignored, so Bluetooth terminal apps may enable automatic newline sending.

The query/CSV field order is compatible with the six-value reference plot:

```text
left_target,left_actual,left_output,right_target,right_actual,right_output
```

Telemetry uses rounded integers to keep the 9600-baud blocking transmit time
bounded. Streaming is off after reset and must be enabled explicitly.

### Safe First Test

1. Lift both wheels and current-limit the motor supply.
2. Send `STOP!` and confirm both motors are stationary.
3. Rotate each wheel by hand and verify the OLED motor page reports the
   expected sign for left and right encoder counts.
4. Set low targets such as `T=10!`.
5. Tune P first, then I, then D. Use `STOP!` immediately if either wheel runs
   in the wrong direction or oscillates strongly.
6. Do not label the value as RPM until encoder PPR, gearbox ratio, and wheel
   circumference have been measured and the conversion is added.

## Build Notes

Use MSPM0 SDK 2.05.01.x and SysConfig 1.24.x. Do not edit generated files
under `Debug`; regenerate them through CCS/SysConfig.

If copied `Debug` makefiles still reference an old absolute `D:\competiion`
path, open/import the project in CCS or CCS Theia with those product versions
and run a clean rebuild so CCS regenerates the build metadata locally.
