# Bluetooth Dual-Wheel Speed Loop Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a Bluetooth-tunable, explicitly started/stopped, 50 ms dual-wheel encoder speed loop to the current MSPM0G3507 board-test application.

**Architecture:** Keep the existing UART0, motor, encoder, and SysConfig assignments. The UART ISR only enqueues bytes; main-loop parsing applies validated commands to a single speed-control owner inside `motor_test.c`. Host-side C tests exercise the parser and speed-loop scheduler before production behavior is added.

**Tech Stack:** C11 host tests with MinGW GCC, MSPM0 DriverLib/SysConfig 1.24, MSPM0 SDK 2.05.01.x, CCS/TI ARM Clang project layout.

---

The requested project directory is itself untracked inside a dirty parent repository, so this plan executes in place and does not create commits. Staging or committing selected files would create a partial, misleading project snapshot and could mix with unrelated user changes.

## File map

- Create `tests/stubs/ti_msp_dl_config.h`: minimal UART/CMSIS declarations for host compilation.
- Create `tests/stubs/main.h`: isolates the legacy Bluetooth header from the embedded aggregate header during the RED build.
- Create `tests/stubs/clock.h`: host `tick_ms` declaration.
- Create `tests/test_speed_control.c`: fake encoder/motor backend and speed-loop behavior tests.
- Create `tests/test_bluetooth_tuning.c`: fake UART/motor-control backend and protocol tests.
- Create `tests/run_host_tests.ps1`: builds and runs both host test executables.
- Modify `Drivers/BSP/pid.h` and `Drivers/BSP/pid.c`: deterministic initialization and runtime-state reset.
- Modify `Drivers/BSP/motor_test.h` and `Drivers/BSP/motor_test.c`: dual-wheel 50 ms speed loop and status API.
- Modify `Drivers/BSP/bsp_hc05.h` and `Drivers/BSP/bsp_hc05.c`: RX ring, robust parser, ACK/error/query/CSV behavior.
- Modify `Drivers/BSP/encoder.h` and `Drivers/BSP/encoder.c`: 32-bit counters and atomic snapshot.
- Modify `Drivers/BSP/oled_display.c`: show closed-loop targets, measured counts, and outputs on the motor page.
- Modify `BOARD_TEST_README.md`: document commands, units, defaults, and hardware bring-up order.
- Do not modify `NUEDC_2026_MSPM0G3507_BoardTest.syscfg` or any file under `Debug/`.

### Task 1: Add failing host tests

**Files:**
- Create: `tests/stubs/ti_msp_dl_config.h`
- Create: `tests/stubs/main.h`
- Create: `tests/stubs/clock.h`
- Create: `tests/test_speed_control.c`
- Create: `tests/test_bluetooth_tuning.c`
- Create: `tests/run_host_tests.ps1`

- [ ] **Step 1: Define the wished-for speed-control API in the test**

The test declares and exercises these production signatures from `motor_test.h`:

```c
typedef enum {
    MOTOR_SIDE_LEFT = 0,
    MOTOR_SIDE_RIGHT,
    MOTOR_SIDE_BOTH
} motor_side_t;

typedef enum {
    MOTOR_PID_KP = 0,
    MOTOR_PID_KI,
    MOTOR_PID_KD
} motor_pid_term_t;

typedef struct {
    bool enabled;
    float left_target;
    float right_target;
    float left_actual;
    float right_actual;
    float left_output;
    float right_output;
    float left_kp;
    float left_ki;
    float left_kd;
    float right_kp;
    float right_ki;
    float right_kd;
    uint32_t update_count;
} motor_speed_status_t;

bool motor_speed_set_gain(motor_side_t side, motor_pid_term_t term, float value);
bool motor_speed_set_target(motor_side_t side, float value);
void motor_speed_set_enabled(bool enabled);
bool motor_speed_is_enabled(void);
void motor_speed_get_status(motor_speed_status_t *status);
```

Test cases use fake `tick_ms`, encoder deltas, and PWM sinks to prove:

```text
initially disabled and both PWM outputs are zero
default gains are 1.37 / 0.40 / 0.15 on both wheels
no control update occurs before 50 ms
at 50 ms, positive target with zero measurement produces positive PWM
left and right gain/target changes remain independent
negative target produces negative PWM
zero target forces the corresponding PWM to zero
100 ms delayed sampling with double encoder counts normalizes to the same count/50 ms measurement
disable immediately drives both PWM outputs to zero
out-of-range gain and target updates are rejected without changing state
```

- [ ] **Step 2: Define the wished-for Bluetooth parser API in the test**

`test_bluetooth_tuning.c` calls:

```c
void Bluetooth_ProcessReceivedByte(uint8_t byte);
void Bluetooth_Poll(void);
```

It feeds complete frames byte-by-byte and records calls to the fake motor-control API. Required cases:

```text
P1/I1/D1 affect only the left loop
P2/I2/D2 affect only the right loop
It and T1 set the left target
Ot and T2 set the right target
T sets both targets
RUN=1 enables; RUN=0 disables without changing targets; STOP disables and zeros targets
Q emits one six-field CSV status line
CSV=1 enables 5 Hz streaming and CSV=0 disables it
CR/LF between frames is ignored
integer, decimal, and negative numeric forms parse correctly
missing value, junk suffix, negative gain, overflow frame, and out-of-range values emit ERR and do not update control state
```

- [ ] **Step 3: Add the test runner**

`tests/run_host_tests.ps1` must compile with warnings as errors:

```powershell
gcc -std=c11 -Wall -Wextra -Werror -pedantic `
  -I tests/stubs -I Drivers/BSP -I Drivers/MSPM0 `
  tests/test_speed_control.c Drivers/BSP/motor_test.c Drivers/BSP/pid.c `
  -lm -o tests/.build/test_speed_control.exe

gcc -std=c11 -Wall -Wextra -Werror -pedantic `
  -I tests/stubs -I Drivers/BSP -I Drivers/MSPM0 `
  tests/test_bluetooth_tuning.c Drivers/BSP/bsp_hc05.c `
  -lm -o tests/.build/test_bluetooth_tuning.exe
```

The script then runs both executables and exits nonzero on the first failure.

- [ ] **Step 4: Run RED and confirm the failure reason**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tests/run_host_tests.ps1
```

Expected: compilation/link failure for missing `motor_speed_*` and `Bluetooth_ProcessReceivedByte` APIs. The failure must be caused by the absent feature, not by a malformed test scaffold.

### Task 2: Make PID state deterministic

**Files:**
- Modify: `Drivers/BSP/pid.h`
- Modify: `Drivers/BSP/pid.c`
- Test: `tests/test_speed_control.c`

- [ ] **Step 1: Add the state-reset declaration**

```c
void pid_reset_state(pid_t *pid);
```

The function preserves `pid_mode`, `p`, `i`, `d`, and `target`, while setting `now`, `error[]`, `error_int`, `pout`, `iout`, `dout`, and `out` to zero.

- [ ] **Step 2: Make initialization clear all fields**

Implement `pid_init()` using a zero initializer before setting mode and gains:

```c
*pid = (pid_t){0};
pid->pid_mode = mode;
pid->p = p;
pid->i = i;
pid->d = d;
```

Add a null guard to `pid_init()`, `pid_reset_state()`, and `pid_cal()`.

- [ ] **Step 3: Run the speed-control test**

Run the first GCC build/executable from Task 1. Expected: it still fails only because the speed-control API has not been implemented; PID compilation itself is warning-free.

### Task 3: Implement the single-owner 50 ms speed loop

**Files:**
- Modify: `Drivers/BSP/motor_test.h`
- Modify: `Drivers/BSP/motor_test.c`
- Test: `tests/test_speed_control.c`

- [ ] **Step 1: Add constants, types, and public API**

```c
#define MOTOR_SPEED_CONTROL_PERIOD_MS 50U
#define MOTOR_SPEED_TARGET_LIMIT      200.0f
#define MOTOR_SPEED_DEFAULT_KP        1.37f
#define MOTOR_SPEED_DEFAULT_KI        0.40f
#define MOTOR_SPEED_DEFAULT_KD        0.15f
```

Add the exact enums, status structure, and functions defined by Task 1.

- [ ] **Step 2: Initialize both wheel controllers**

In `motor_test_init()`:

```c
pid_init(&motorA, DELTA_PID,
    MOTOR_SPEED_DEFAULT_KP, MOTOR_SPEED_DEFAULT_KI, MOTOR_SPEED_DEFAULT_KD);
pid_init(&motorB, DELTA_PID,
    MOTOR_SPEED_DEFAULT_KP, MOTOR_SPEED_DEFAULT_KI, MOTOR_SPEED_DEFAULT_KD);
```

Keep closed-loop disabled and motors stopped after initialization.

- [ ] **Step 3: Implement validated setters**

- Kp/Kd range: `0.0f..20.0f`.
- Ki range: `0.0f..10.0f`.
- Target range: `-200.0f..200.0f`.
- Reject NaN with `value != value`.
- Apply `MOTOR_SIDE_BOTH` to both controllers.
- After changing a gain, call `pid_reset_state()` on the affected controller while preserving its target.

- [ ] **Step 4: Implement enable/disable semantics**

Enabling must:

```text
force open-loop test mode to zero
clear any accumulated encoder sample
reset both PID runtime states
set last sample time to the current tick
set enabled=true
```

Disabling must immediately call `set_left_PWM(0)` and `set_right_PWM(0)`, reset both PID states, and leave targets unchanged. `motor_test_stop()` additionally zeros both targets for emergency-stop behavior.

- [ ] **Step 5: Implement the 50 ms update**

When enabled and `tick_ms - last_encoder_ms >= 50`:

```c
elapsed_ms = tick_ms - last_encoder_ms;
encoder_update();
raw_left = get_encoder_count_L();
raw_right = get_encoder_count_R();
scale = 50.0f / (float)elapsed_ms;
motorA.now = (float)raw_left * scale;
motorB.now = (float)raw_right * scale;
```

Accumulate raw totals for the OLED. For each wheel, target zero means PWM zero plus `pid_reset_state()`; otherwise call `pid_cal()` and round the output to the nearest integer before the motor driver call. Set `last_encoder_ms` to the current tick and increment `update_count`.

When disabled, preserve the existing 100 ms encoder-only board-test sampling.

- [ ] **Step 6: Verify GREEN for speed control**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tests/run_host_tests.ps1
```

Expected: speed-control executable passes; Bluetooth executable still fails because the parser API is absent.

### Task 4: Replace ISR-side tuning with a bounded main-loop parser

**Files:**
- Modify: `Drivers/BSP/bsp_hc05.h`
- Modify: `Drivers/BSP/bsp_hc05.c`
- Test: `tests/test_bluetooth_tuning.c`

- [ ] **Step 1: Remove obsolete coupling**

Remove `#include "main.h"`, `BLUETOOTH_ENABLE_PID_TUNING`, `DataBuff`, `RxLine`, `RxBuffer`, `Is_Receiving`, `Get_Data()`, and `USART_PID_Adjust()`. Keep existing public send/receive helpers and diagnostic counters where they are still useful.

- [ ] **Step 2: Add the RX queue**

Use a power-of-two 64-byte ring:

```c
#define BLUETOOTH_RX_QUEUE_LEN 64U
static uint8_t bluetooth_rx_queue[BLUETOOTH_RX_QUEUE_LEN];
static volatile uint8_t bluetooth_rx_head;
static volatile uint8_t bluetooth_rx_tail;
```

The ISR drains available UART0 RX bytes into the ring and increments `Bluetooth_IrqCount`. It never parses or transmits. If the next head equals the tail, increment `Bluetooth_OverflowCount` and discard the new byte.

- [ ] **Step 3: Add a bounded frame collector and decimal parser**

Use a 31-character frame buffer plus terminator. Ignore CR/LF. On overflow, discard until the next `!` so a suffix cannot be mistaken for a new command.

The numeric parser accepts:

```text
12
-12
0.4
.4
-0.40
```

It rejects an empty value, sign-only input, multiple decimal points, nondigits, and exponent/NaN/Inf spellings.

- [ ] **Step 4: Implement the command table**

Map commands exactly as follows:

```text
P1/I1/D1 -> left Kp/Ki/Kd
P2/I2/D2 -> right Kp/Ki/Kd
It/T1 -> left target
Ot/T2 -> right target
T -> both targets
RUN=1 -> enable
RUN=0 -> disable, preserve targets
STOP -> disable and set both targets to zero
CSV=1/CSV=0 -> toggle 5 Hz streaming
Q -> one status record
```

Use `motor_speed_set_gain()` and `motor_speed_set_target()` as the final validation boundary. A rejected setter returns `ERR,RANGE!`.

- [ ] **Step 5: Implement responses without float printf**

Success: `OK!\r\n`.

Errors: `ERR,FMT!\r\n`, `ERR,CMD!\r\n`, `ERR,RANGE!\r\n`, or `ERR,LEN!\r\n`.

Status output uses rounded integers and the reference six-field order:

```c
snprintf(message, sizeof(message), "%d,%d,%d,%d,%d,%d\r\n", ...);
```

`CSV=1` emits no faster than every 200 ms from `Bluetooth_Poll()` and remains off after reset.

- [ ] **Step 6: Process only the software queue in the main loop**

`Bluetooth_Poll()` pops queued bytes, increments `Bluetooth_PollCount`, and calls public `Bluetooth_ProcessReceivedByte()`. It must not race the ISR by independently reading the hardware FIFO.

- [ ] **Step 7: Verify GREEN for the full host suite**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tests/run_host_tests.ps1
```

Expected: both executables pass with zero compiler warnings.

### Task 5: Make encoder sampling atomic

**Files:**
- Modify: `Drivers/BSP/encoder.h`
- Modify: `Drivers/BSP/encoder.c`
- Test: `tests/test_speed_control.c`

- [ ] **Step 1: Narrow counters to 32 bit**

Change `temp_count` to `volatile int32_t` and `count` to `int32_t`; update the internal helper pointer type accordingly.

- [ ] **Step 2: Snapshot both wheels in one short critical section**

Preserve and restore PRIMASK:

```c
uint32_t primask = __get_PRIMASK();
__disable_irq();
left_count = motor_encoder_L.temp_count;
right_count = motor_encoder_R.temp_count;
motor_encoder_L.temp_count = 0;
motor_encoder_R.temp_count = 0;
if (primask == 0U) {
    __enable_irq();
}
```

Assign public `count` and direction after interrupts are restored.

- [ ] **Step 3: Handle simultaneous A/B pending bits**

Replace the per-wheel `if ... else if` helper behavior with two independent `if` blocks so both pending edges are accounted for before both status bits are cleared.

- [ ] **Step 4: Re-run host tests and ARM syntax compilation**

Host tests must remain green. ARM syntax compilation is defined in Task 7.

### Task 6: Surface the loop on OLED and document operation

**Files:**
- Modify: `Drivers/BSP/oled_display.c`
- Modify: `BOARD_TEST_README.md`

- [ ] **Step 1: Update the motor page**

Call `motor_speed_get_status()` once and display integer-safe fields:

```text
CL:ON/OFF 50ms
L T:<target> N:<actual>
L O:<output>
R T:<target> N:<actual>
R O:<output>
```

Retain total encoder counts on the remaining line or in disabled board-test mode. Do not use `%f` in embedded `snprintf`.

- [ ] **Step 2: Document the command protocol**

Add the complete command table, defaults, range limits, six-field CSV order, explicit `RUN=1!` requirement, `STOP!` behavior, and the “counts/50 ms” unit warning to `BOARD_TEST_README.md`.

- [ ] **Step 3: Document safe bring-up**

The README must say: lift wheels, current-limit the supply, send `STOP!`, verify encoder signs at low manual rotation, then start with targets around 10–20 counts/50 ms before tuning P, I, and D.

### Task 7: Verify source, SysConfig, and build boundaries

**Files:**
- Inspect: all modified files
- Do not edit: `Debug/**`

- [ ] **Step 1: Run the full host tests fresh**

```powershell
powershell -ExecutionPolicy Bypass -File tests/run_host_tests.ps1
```

Expected: two test executables report PASS and the script exits 0.

- [ ] **Step 2: Run SysConfig static inspection**

```powershell
python C:\Users\11653\.agents\skills\mspm0-ccs\scripts\check_syscfg.py `
  "F:\Desktop Folder\大学\电赛\NUEDC_2026_MSPM0G3507_BoardTest-main"
```

Expected: `.syscfg` metadata and `SYSCFG_DL_init()` checks pass; no generated file was edited.

- [ ] **Step 3: Generate SysConfig to a temporary directory**

Use `C:\TI\sysconfig_1.24.0\sysconfig_cli.bat`, the active `.syscfg`, and `C:\TI\mspm0_sdk_2_05_01_00\.metadata\product.json`. Expected: exit 0 with generated names matching UART0, TIMA0 PWM, encoder GPIO, and `SYSCFG_DL_init()`.

- [ ] **Step 4: Compile modified firmware files for Cortex-M0+ syntax**

Use `C:\TI\gcc_arm_none_eabi_9_2_1\bin\arm-none-eabi-gcc.exe` with:

```text
-std=c11 -mcpu=cortex-m0plus -mthumb -D__MSPM0G3507__
-I Debug
-I C:\TI\mspm0_sdk_2_05_01_00\source
-I C:\TI\mspm0_sdk_2_05_01_00\source\third_party\CMSIS\Core\Include
-I Drivers\BSP -I Drivers\MSPM0 and the existing driver include directories
-fsyntax-only
```

Compile at least `pid.c`, `motor_test.c`, `bsp_hc05.c`, `encoder.c`, `oled_display.c`, and `main.c`. Expected: no errors; existing unrelated warnings must be reported separately.

- [ ] **Step 5: Attempt the existing CCS make build and classify its result honestly**

```powershell
gmake -C Debug clean all
```

The copied `Debug` rules currently point to absent `D:/competiion/Ti/...` SDK 2.11/SysConfig 1.26/toolchain paths. If the command fails for those paths, report it as stale generated build metadata, not as a firmware source failure. Do not edit generated makefiles; instruct CCS/CCS Theia to regenerate them from `.cproject` using SDK 2.05.01 and SysConfig 1.24.

- [ ] **Step 6: Review the final diff and requirements**

Confirm:

```text
no .syscfg pin changes
no Debug/generated edits
UART ISR contains no parser, PID, OLED, or transmit work
only motor_test owns closed-loop encoder snapshots and PWM updates
all documented commands are tested
boot and STOP leave both PWM outputs at zero
remaining hardware validation is explicitly identified
```
