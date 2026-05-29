# RS00 STM32 Documentation v1.1

## 1. Scope
This document describes the current `Motor_control` project status for RS00 motor CAN control on STM32F746.

Version: `v1.1`  
Project path: `STM32CubeIDE/workspace_1.19.0/Motor_control`

## 2. Current Status
- CAN communication can be diagnosed in real-time via UART (`LEC/TEC/REC/BOFF/EPVF/EWGF`).
- Live command control via debugger Live Expressions is enabled.
- Full mode test sequence command (`cmd=15`) is available.
- Auto mode-cycle is enabled by default (boot + periodic).
- Reusable RS00 library is now `rs00.h/.c`.
- `main.c` low-level protocol duplication removed; it now uses `rs00` API only.

## 3. Key Files
- Main application:
  - `Core/Src/main.c`
- RS00 reusable library:
  - `Core/Inc/rs00.h`
  - `Core/Src/rs00.c`
- Quick-start guide:
  - `README.md`
- Legacy library files removed:
  - `Core/Inc/robstride_rs00.h` (deleted)
  - `Core/Src/robstride_rs00.c` (deleted)

## 4. Live Expression Control
Use these Live Expressions:
- `g_live_cmd` (uint32)
- `g_live_u32` (uint32 argument)
- `g_live_f32` (float argument)
- `g_live_apply` (write `1` to execute)
- `g_live_auto_iq` (auto current ramp flag)

Execution rule:
1. Set `g_live_cmd`
2. Set argument (`g_live_u32` or `g_live_f32`) if needed
3. Set `g_live_apply = 1`

## 5. Command Map
- `1`  enable
- `2`  stop
- `3`  clear fault
- `4`  set mode (`g_live_u32`)
- `5`  set iq current (`g_live_f32`, A)
- `6`  set velocity (`g_live_f32`, rad/s)
- `7`  set CSP position (`g_live_f32`, rad)
- `8`  read all params
- `9`  active report on (`g_live_u32`, ms)
- `10` active report off
- `11` set zero
- `12` save params
- `13` auto iq on
- `14` auto iq off
- `15` run all controller mode tests

## 6. Run-All-Modes Test
Command:
- `g_live_cmd = 15`
- `g_live_apply = 1`

Sequence:
1. Operation (MIT)
2. Current
3. Velocity
4. CSP Position
5. PP Position
6. Return to Current mode with `Iq=0`

UART markers:
- `[TEST] all_modes begin`
- `[TEST] operation`
- `[TEST] current`
- `[TEST] velocity`
- `[TEST] csp`
- `[TEST] pp`
- `[TEST] all_modes done`

## 7. Control Theory (5 Modes)
### 7.1 Operation (MIT-like, Type 0x01)
- Control law (concept):  
  `tau = Kp*(q_ref - q) + Kd*(dq_ref - dq) + tau_ff`
- Inputs: position, velocity, `Kp`, `Kd`, feedforward torque.
- Use when you need compliant but responsive behavior (leg/joint style control).
- Too high `Kp/Kd` can cause oscillation/noise amplification.

### 7.2 Current Mode (Run mode = 3)
- Inner-loop torque-producing current (`Iq`) control.
- Approximate relation: motor torque is proportional to `Iq`.
- Best for direct torque feel and force control experiments.
- Safety: ramp `Iq` gradually, enforce current/torque limits.

### 7.3 Velocity Mode (Run mode = 2)
- Speed loop tracks `w_ref` (rad/s), usually with acceleration limiter.
- Typical chain: speed controller -> current controller.
- Good for continuous rotation tasks.
- Key params: speed target, accel (`IDX_ACC_RAD`), current limit (`IDX_LIMIT_CUR`).

### 7.4 CSP Position Mode (Run mode = 5)
- Cyclic synchronous position tracking (`q_ref` updated frequently).
- External controller/host sends position trajectory in real-time.
- Good for smooth trajectory tracking and robotics coordination.
- Key params: position target (`IDX_LOC_REF`), speed limit (`IDX_LIMIT_SPD`), torque limit.

### 7.5 PP Position Mode (Run mode = 1)
- Profile position: motor internally generates motion profile to target.
- Host gives target + constraints (speed/accel), driver plans motion.
- Simpler host-side control vs CSP, but less direct trajectory authority.
- Key params: `IDX_VEL_MAX`, `IDX_ACC_SET`, then `IDX_LOC_REF`.

### 7.6 Practical Tuning Order
1. Verify CAN + feedback stability (`LEC=0`, no bus-off).
2. Start with Current mode at low `Iq` (e.g., `0.2~0.5 A`).
3. Then Velocity mode with low accel/current limit.
4. Then CSP/PP position with conservative speed/torque limits.
5. Use Operation mode last for fine dynamic tuning (`Kp/Kd/tau_ff`).

## 8. Auto Mode Cycle
Default behavior in `main.c`:
- `ROBSTRIDE_AUTO_MODE_CYCLE = 1`
- Boot дээр шууд `run_all_modes` нэг удаа ажиллана.
- Дараа нь `ROBSTRIDE_AUTO_CYCLE_PERIOD_MS` хугацаагаар (default: `45000 ms`) дахин ажиллана.

Disable:
- `ROBSTRIDE_AUTO_MODE_CYCLE = 0`

## 9. Typical Manual Test Flow
1. Set current mode:
   - `g_live_u32=3`, `g_live_cmd=4`, `g_live_apply=1`
2. Enable:
   - `g_live_cmd=1`, `g_live_apply=1`
3. Set current:
   - `g_live_f32=0.5`, `g_live_cmd=5`, `g_live_apply=1`
4. Stop:
   - `g_live_cmd=2`, `g_live_apply=1`

## 10. UART Diagnostics
Main CAN diagnostic line:
- `[CAN] <TAG> LEC=... TEC=... REC=... BOFF=... EPVF=... EWGF=... INAK=...`

Interpretation quick guide:
- `LEC=0(NO_ERR), TEC=0, REC=0, BOFF=0` => bus healthy
- `LEC=3(ACK_ERR)` => ACK missing (peer/device not acknowledging)
- `LEC=5(BIT0_ERR)` with `BOFF=1` => physical layer issue (wiring/transceiver/bitrate mismatch)

## 11. Notes
- If newly added source files are not compiled, refresh project settings or ensure build lists include them.
- `fault=0x80` may still appear depending on motor state/calibration status; motor can still run if enable and current loop are active.
- If stale `Debug/` references appear after file deletions, run `Project > Clean` then `Build`.

## 12. Changelog
### v1.1
- Migrated documentation to `rs00.h/.c` as canonical RS00 library.
- Added auto mode-cycle behavior documentation.
- Added CAN diagnostic interpretation section.
- Marked legacy `robstride_rs00.*` as removed.
- Added control-theory summary for 5 control modes.

### v1.0
- Added debugger-driven live command control.
- Added full mode test command (`cmd=15`).
- Added reusable RS00 protocol library (`robstride_rs00.h/.c`).
- Fixed enable/stop ID packing to match working ESP32 behavior.
