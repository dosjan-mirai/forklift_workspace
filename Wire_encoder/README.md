# Wire_encoder

STM32F746 дээр ажиллах Briter wire/linear encoder reader firmware. Энэ төсөл encoder-ийн утгыг `CAN1`-ээр polling хийж уншаад raw count-оос displacement(mm) тооцдог.

## What It Does
- MCU: `STM32F746`
- Interface: `CAN1 (bxCAN)`
- Optional debug: `USART3`
- Default encoder ID: `0x01`
- Default CAN speed: `1 Mbps`
- Poll interval: `5 ms`

Main flow:
1. STM32 encoder руу read request frame явуулна.
2. Encoder CAN response буцаана.
3. Firmware raw 32-bit утгыг decode хийнэ.
4. Wrap-around correction, step filter, baseline ашиглаад displacement(mm) тооцно.

## CAN Protocol Used

Read request:

```text
StdId = ENCODER_ID
DLC   = 4
Data  = [0x04][ID][0x01][0x00]
```

Read response:

```text
StdId = ENCODER_ID
DLC   >= 7
Data  = [0x07][ID][0x01][raw0][raw1][raw2][raw3]
```

`raw0..raw3` нь little-endian 32-bit signed value.

## Current Firmware Behavior
- `ENCODER_ID = 0x01`
- `TOTAL_COUNTS = 98300`
- `RESOLUTION_MM = 0.0244f`
- `MAX_STEP_COUNTS = 5000`
- `REQUEST_INTERVAL_MS = 5`
- `RESPONSE_TIMEOUT_MS = 40`
- `CAN_DISCOVERY_MODE = 0`
- `CAN_BOOT_BAUD_CODE = CAN_BAUD_1M`
- `CAN_RUN_BAUD_CODE = CAN_BAUD_1M`
- `CAN_PUSH_BAUD_TO_ENCODER_AT_BOOT = 0`
- `CAN_ENABLE_AUTO_BAUD_SCAN = 0`
- `UART_LOG_ENABLE = 0`

Анхаарах зүйл:
- Одоогийн build дээр UART log default-аар унтраалттай. Тиймээс boot banner болон `printf()` log харагдахгүй.
- CAN RX callback нь request echo frame-ийг хаядаг.
- Зөвхөн encoder-ийн read response хэлбэртэй frame-үүдийг хүлээж авна.

## Relevant Files
- `Core/Src/main.c`: CAN init, polling loop, RX callback, raw value update
- `Core/Inc/briter_encoder_lib.h`: protocol helper, decode, wrap correction, baseline, displacement math
- `Wire_encoder.ioc`: CubeMX peripheral configuration

## Library Responsibilities
`Core/Inc/briter_encoder_lib.h` нь header-only helper бөгөөд дараахийг хийдэг:
- Read request frame build
- Read response frame validation
- Raw little-endian decode
- Wrap-around delta correction
- Spike rejection via `MAX_STEP_COUNTS`
- Auto baseline set
- Displacement(mm) calculation

## Build

```bash
make -C /home/dosjan/STM32CubeIDE/workspace_1.19.0/Wire_encoder/Debug all -j4
```

Build output:
- `Debug/Wire_encoder.elf`
- `Debug/Wire_encoder.bin`

## Flash And Verify
1. `Debug/Wire_encoder.elf`-ийг board руу flash хийнэ.
2. CAN bus дээр encoder `ID=0x01`, `1 Mbps` тохиргоотой байгаа эсэхийг шалгана.
3. Encoder хөдөлгөхөд firmware CAN request-үүдийг 5 ms тутам илгээнэ.
4. Хэрэв debug log хэрэгтэй бол `UART_LOG_ENABLE`-г `1` болгож rebuild хийнэ.

UART log enabled үед харагдах боломжтой мөрүүд:

```text
FW_V3_PROD_READ | boot=1 run=1 profile=3
RX id=0x001 dlc=7 | Raw: ... | Disp: ... mm
```

## Notes
- Displacement нь first valid sample-ийг baseline болгож тооцогдоно.
- Wrap correction нь `TOTAL_COUNTS / 2` босгоор ажиллана.
- Хэт огцом өөрчлөлтийг `MAX_STEP_COUNTS` ашиглаж шүүнэ.
- CAN bus-off алдаа гарвал firmware CAN stack-ийг stop/start хийж сэргээх гэж оролдоно.

## Known Build Issue
Зарим generated `Debug/Core/Src/subdir.mk` файлд `-fcyclomatic-complexity` орж ирвэл GCC build fail хийж болно.

Workaround:

```bash
sed -i 's/ -fcyclomatic-complexity//g' /home/dosjan/STM32CubeIDE/workspace_1.19.0/Wire_encoder/Debug/Core/Src/subdir.mk
```

Дараа нь build-ээ дахин ажиллуул.
