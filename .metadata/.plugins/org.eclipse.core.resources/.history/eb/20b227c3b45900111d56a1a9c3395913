/*
 * WEncoder_CAN.h
 *
 * Briter wire encoder — standalone HAL driver (FreeRTOS-гүй).
 *
 * Ашиглах:
 *   1. WEncoder_CAN_Init(&hcan1)        — MX_CAN1_Init()-ийн дараа
 *   2. WEncoder_CAN_AddEncoder(...)     — encoder бүртгэх
 *   3. WEncoder_CAN_StartStack()        — CAN filter + start
 *   4. WEncoder_CAN_Poll()              — main loop дотор тогтмол дуудах
 *   5. WEncoder_CAN_RxCallback(&hcan1)  — HAL_CAN_RxFifo0MsgPendingCallback дотор
 */

#ifndef INC_WENCODER_CAN_H_
#define INC_WENCODER_CAN_H_

#include "main.h"
#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ═══════════════════════════════════════════════════════════════════════════
 *  ТОХИРГОО — хэрэгцээгээрээ өөрчил
 * ═══════════════════════════════════════════════════════════════════════════ */

#define WENC_MAX_ENCODERS           8U      /* Pool хэмжээ                   */
#define WENC_REQUEST_INTERVAL_MS    5U      /* Poll давтамж (ms)             */
#define WENC_ENCODER_TIMEOUT_MS     100U    /* Хариу ирэхгүй бол timeout     */

/* Baud code — datasheet Table 2 */
#define WENC_BAUD_500K  0x00U
#define WENC_BAUD_1M    0x01U
#define WENC_BAUD_250K  0x02U
#define WENC_BAUD_125K  0x03U
#define WENC_BAUD_100K  0x04U
#define WENC_RUN_BAUD   WENC_BAUD_1M   /* Ажиллах baud — энд тохируул */

/* ═══════════════════════════════════════════════════════════════════════════
 *  STRUCT
 * ═══════════════════════════════════════════════════════════════════════════ */

typedef struct {
    uint8_t  encoder_id;
    int32_t  total_counts;
    float    resolution_mm;
    int32_t  max_step_counts;
    int32_t  baseline;
    uint8_t  baseline_set;
} WEnc_Core_t;

typedef struct {
    uint8_t     can_id;
    char        label[16];
    WEnc_Core_t core;
    float       disp_mm;
    int32_t     raw;
    uint32_t    last_rx_ms;
    uint8_t     valid;
    uint8_t     in_use;
} WEnc_Encoder_t;

typedef struct {
    uint8_t  can_id;
    char     label[16];
    float    disp_mm;
    int32_t  raw;
    uint8_t  valid;
    uint32_t tick;
} WEnc_Reading_t;

typedef struct {
    WEnc_Encoder_t     pool[WENC_MAX_ENCODERS];
    uint8_t            count;
    CAN_HandleTypeDef *hcan;
    uint8_t            initialized;
} WEnc_Handle_t;

extern WEnc_Handle_t g_wenc;

/* ═══════════════════════════════════════════════════════════════════════════
 *  API
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief  Init. MX_CAN1_Init()-ийн дараа дуудна.
 */
void WEncoder_CAN_Init(CAN_HandleTypeDef *hcan);

/**
 * @brief  Encoder бүртгэх.
 * @param  can_id        CAN STD ID (0x01 … 0xFF)
 * @param  label         Нэр: "Lift", "Tilt" …
 * @param  total_counts  Нийт count
 * @param  resolution_mm Нэг count = мм (жш: 0.0244f)
 * @param  max_step      Нэг poll хоорондын хамгийн их алхам (spike filter)
 * @return Index (0..N-1), 0xFF = pool дүүрсэн эсвэл давхардсан ID
 */
uint8_t WEncoder_CAN_AddEncoder(uint8_t     can_id,
                                 const char *label,
                                 int32_t     total_counts,
                                 float       resolution_mm,
                                 int32_t     max_step);

/**
 * @brief  CAN filter + start. Бүх encoder бүртгэсний дараа дуудна.
 */
HAL_StatusTypeDef WEncoder_CAN_StartStack(void);

/**
 * @brief  CAN аль хэдийн Init+baud тохируулагдсан үед
 *         filter + notification + start хийнэ.
 *         Config tool шиг baud-ийг өөрөө удирддаг тохиолдолд ашиглана.
 */
HAL_StatusTypeDef WEncoder_CAN_FilterStart(void);

/**
 * @brief  Main loop дотор тогтмол дуудах — poll request илгээж,
 *         ISR-аас тэмдэглэгдсэн шинэ frame-үүдийг боловсруулна.
 */
void WEncoder_CAN_Poll(void);

/**
 * @brief  HAL_CAN_RxFifo0MsgPendingCallback дотор дуудна.
 *         STD frame (Briter) бол 1, бусад бол 0 буцаана.
 */
uint8_t WEncoder_CAN_RxCallback(CAN_HandleTypeDef *hcan);

/**
 * @brief  Index-ээр утга авах.
 * @return 1 = OK, 0 = invalid
 */
uint8_t WEncoder_CAN_GetByIndex(uint8_t index, WEnc_Reading_t *out);

/**
 * @brief  CAN ID-аар утга авах.
 * @return 1 = олдсон, 0 = олдоогүй
 */
uint8_t WEncoder_CAN_GetById(uint8_t can_id, WEnc_Reading_t *out);

/**
 * @brief  Бүртгэгдсэн encoder тоо.
 */
uint8_t WEncoder_CAN_GetCount(void);

/**
 * @brief  Timeout шалгах (WENC_ENCODER_TIMEOUT_MS-ээс хэтэрсэн үү).
 * @return 1 = timeout, 0 = OK
 */
uint8_t WEncoder_CAN_IsTimeout(uint8_t index);

/**
 * @brief  Одоогийн байрлалыг 0 болгох.
 */
void WEncoder_CAN_ZeroHere(uint8_t index);

#ifdef __cplusplus
}
#endif

#endif /* INC_WENCODER_CAN_H_ */
