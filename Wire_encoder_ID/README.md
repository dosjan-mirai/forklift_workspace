# Wire Encoder CAN ID Config Tool

**STM32 Nucleo-F746ZG** дээр **Briter CAN Wire Encoder**-ийн CAN ID болон Baud rate-ийг нэг удаа тохируулах хэрэгсэл.

---

## Wiring Diagram


| Encoder утас | Өнгө | Холболт |
|---|---|---|
| VCC | 🔴 Улаан | 12~24V гадаад тэжээл |
| GND | ⚫ Хар | MCP2551 VSS, Nucleo GND |
| CAN_H | 🟢 Ногоон | MCP2551 CANH (pin 7) |
| CAN_L | ⚪ Цагаан | MCP2551 CANL (pin 6) |
| Функц | 🟡 Шар | **Холбохгүй** — хөвөгч үлдэх |

| MCP2551 | Nucleo | Тайлбар |
|---|---|---|
| TXD (pin 1) | PA12 / CN12-12 | CAN TX шууд |
| RXD (pin 4) | PA11 / CN12-10 | 10kΩ+20kΩ хуваагчаар (5V→3.3V) |
| VDD (pin 3) | +5V / CN11-18 | |
| VSS (pin 2) | GND / CN12-7 | |
| Rs (pin 8) | GND | 1 Mbps горим |

---

## main.c тохиргоо

`Core/Src/main.c` файлын **35-р мөр** орчмын 4 тодорхойлолтыг өөрчил:

```c
#define ENCODER_CURRENT_ID        0x01U   // Encoder-ийн одоогийн CAN ID
#define ENCODER_TARGET_ID         0x02U   // Солих CAN ID

#define ENCODER_CURRENT_BAUD_CODE WENC_BAUD_1M   // Одоогийн baud
#define ENCODER_TARGET_BAUD_CODE  WENC_BAUD_1M   // Солих baud
```

**Baud кодууд:**

| Тодорхойлолт | Хурд |
|---|---|
| `WENC_BAUD_500K` | 500 kbps (factory default) |
| `WENC_BAUD_1M` | 1 Mbps |
| `WENC_BAUD_250K` | 250 kbps |
| `WENC_BAUD_125K` | 125 kbps |
| `WENC_BAUD_100K` | 100 kbps |

---

## STM32CubeIDE — Flash хийх

```
1. File → Open Projects from File System → Wire_encoder_ID хавтас
2. main.c → 4 #define утгыг өөрчил
3. Ctrl+B  (Build)
4. F11     (Flash)
```

---

## LED үр дүн

| LED | Утга |
|---|---|
| 🟢 LD1 (PB0) ногоон | CONFIG OK — амжилттай |
| 🔴 LD3 (PB14) улаан | CONFIG FAILED — алдаа |

**Serial log:** USART3 / 115200 baud 8N1 (ST-Link VCP)

---

## Анхааруулга

> ⚠ MCP2551 RXD гаралт **5V** байдаг тул **10kΩ + 20kΩ** хуваагч заавал хэрэгтэй.  
> ⚠ MCP2551 Rs (pin 8) → **GND** холбоогүй бол CAN ажиллахгүй.  
> ⚠ Encoder-ийн **шар утас**-ыг ажиллах үед **холбохгүй**.
