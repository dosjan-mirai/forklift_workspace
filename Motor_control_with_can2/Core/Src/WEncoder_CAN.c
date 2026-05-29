/*
 * WEncoder_CAN.c
 *
 * Briter wire encoder — standalone HAL driver (FreeRTOS-гүй).
 *
 * RX замнал:
 *   ISR (RxCallback) → pending flag тавина
 *   Poll()           → flag шалгаж frame боловсруулна
 *
 * TX замнал:
 *   Poll() → WENC_REQUEST_INTERVAL_MS тутамд бүртгэлтэй encoder бүрт
 *             read request илгээнэ
 */

#include "WEncoder_CAN.h"

/* ═══════════════════════════════════════════════════════════════════════════
 *  PROTOCOL CONSTANTS
 * ═══════════════════════════════════════════════════════════════════════════ */

#define WENC_CMD_BYTE    0x04U
#define WENC_CMD_READ    0x01U
#define WENC_POLL_DLC    4U
#define WENC_RESP_DLC    7U

/* ═══════════════════════════════════════════════════════════════════════════
 *  GLOBAL HANDLE
 * ═══════════════════════════════════════════════════════════════════════════ */

WEnc_Handle_t g_wenc = {0};

/* ═══════════════════════════════════════════════════════════════════════════
 *  ISR → POLL pending buffer
 *  ISR нь frame-ийг энд хадгалж, flag тавина.
 *  Poll() нь flag шалгаж боловсруулна.
 *  Нэг л frame буфер — encoder тус бүрт ялгадаг тул хангалттай.
 * ═══════════════════════════════════════════════════════════════════════════ */

typedef struct {
    uint16_t std_id;
    uint8_t  dlc;
    uint8_t  data[8];
    uint8_t  pending;   /* 1 = Poll()-д боловсруулах frame байна */
} WEnc_RxSlot_t;

/* Encoder тус бүрт нэг slot */
static WEnc_RxSlot_t s_rx[WENC_MAX_ENCODERS];

/* ═══════════════════════════════════════════════════════════════════════════
 *  CORE PROTOCOL
 * ═══════════════════════════════════════════════════════════════════════════ */

static void core_Init(WEnc_Core_t *c, uint8_t id,
                      int32_t total, float res, int32_t max_step)
{
    c->encoder_id      = id;
    c->total_counts    = total;
    c->resolution_mm   = res;
    c->max_step_counts = max_step;
    c->baseline        = 0;
    c->baseline_set    = 0U;
}

static void core_BuildRequest(uint8_t id, uint8_t *data)
{
    data[0] = WENC_CMD_BYTE;
    data[1] = id;
    data[2] = WENC_CMD_READ;
    data[3] = 0x00U;
    data[4] = 0x00U;
    data[5] = 0x00U;
    data[6] = 0x00U;
    data[7] = 0x00U;
}

/* Echo шүүх: host-ийн өөрийн request буцаж ирвэл хаях */
static uint8_t core_IsEcho(const uint8_t *data, uint8_t dlc)
{
    return ((dlc      == WENC_POLL_DLC)  &&
            (data[0]  == WENC_CMD_BYTE)  &&
            (data[2]  == WENC_CMD_READ)  &&
            (data[3]  == 0x00U)) ? 1U : 0U;
}

/* Datasheet response: [0x07][ID][0x01][D0][D1][D2][D3] */
static uint8_t core_IsValidResponse(uint16_t std_id, uint8_t dlc,
                                     const uint8_t *data, uint8_t enc_id)
{
    return ((std_id   == (uint16_t)enc_id) &&
            (dlc      == WENC_RESP_DLC)    &&
            (data[0]  == WENC_RESP_DLC)    &&
            (data[1]  == enc_id)           &&
            (data[2]  == WENC_CMD_READ)) ? 1U : 0U;
}

/* Position: data[3]=LSB … data[6]=MSB */
static int32_t core_DecodeLE(const uint8_t *data)
{
    uint32_t u = ((uint32_t)data[3])        |
                 ((uint32_t)data[4] <<  8U) |
                 ((uint32_t)data[5] << 16U) |
                 ((uint32_t)data[6] << 24U);
    return (int32_t)u;
}

/* Spike filter */
static uint8_t core_AcceptStep(WEnc_Core_t *c, int32_t new_raw)
{
    if (!c->baseline_set) return 1U;
    int32_t delta = new_raw - c->baseline;
    if (delta < 0) delta = -delta;
    return (delta <= c->max_step_counts) ? 1U : 0U;
}

/* Эхний утгаар auto baseline тавих */
static void core_AutoBaseline(WEnc_Core_t *c, int32_t raw)
{
    if (c->baseline_set) return;
    c->baseline     = raw;
    c->baseline_set = 1U;
}

static float core_DisplacementMm(const WEnc_Core_t *c, int32_t raw)
{
    return (float)(raw - c->baseline) * c->resolution_mm;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  PRIVATE HELPERS
 * ═══════════════════════════════════════════════════════════════════════════ */

static HAL_StatusTypeDef prv_ApplyBitTiming(uint8_t baud_code)
{
    CAN_HandleTypeDef *h = g_wenc.hcan;

#if defined(WENC_APB1_12MHZ)
    /* APB1 = 12 MHz, BS1=4TQ, BS2=1TQ → P×6 */
    h->Init.SyncJumpWidth = CAN_SJW_1TQ;
    h->Init.TimeSeg1      = CAN_BS1_4TQ;
    h->Init.TimeSeg2      = CAN_BS2_1TQ;
    switch (baud_code) {
        case WENC_BAUD_500K: h->Init.Prescaler = 4;  break;  /* 12M/(4×6)=500K  */
        case WENC_BAUD_1M:   h->Init.Prescaler = 2;  break;  /* 12M/(2×6)=1M    */
        case WENC_BAUD_250K: h->Init.Prescaler = 8;  break;  /* 12M/(8×6)=250K  */
        case WENC_BAUD_125K: h->Init.Prescaler = 16; break;  /* 12M/(16×6)=125K */
        case WENC_BAUD_100K: h->Init.Prescaler = 20; break;  /* 12M/(20×6)=100K */
        default: return HAL_ERROR;
    }
#else
    /* APB1 = 48 MHz, BS1=13TQ, BS2=2TQ → P×16 */
    h->Init.SyncJumpWidth = CAN_SJW_1TQ;
    h->Init.TimeSeg1      = CAN_BS1_13TQ;
    h->Init.TimeSeg2      = CAN_BS2_2TQ;
    switch (baud_code) {
        case WENC_BAUD_500K: h->Init.Prescaler = 6;  break;  /* 48M/(6×16)=500K  */
        case WENC_BAUD_1M:   h->Init.Prescaler = 3;  break;  /* 48M/(3×16)=1M    */
        case WENC_BAUD_250K: h->Init.Prescaler = 12; break;  /* 48M/(12×16)=250K */
        case WENC_BAUD_125K: h->Init.Prescaler = 24; break;  /* 48M/(24×16)=125K */
        case WENC_BAUD_100K: h->Init.Prescaler = 30; break;  /* 48M/(30×16)=100K */
        default: return HAL_ERROR;
    }
#endif
    return HAL_OK;
}

/* can_id-аар encoder pointer буцаах */
static WEnc_Encoder_t *prv_Find(uint8_t can_id)
{
    for (uint8_t i = 0U; i < g_wenc.count; i++) {
        if (g_wenc.pool[i].in_use && g_wenc.pool[i].can_id == can_id)
            return &g_wenc.pool[i];
    }
    return NULL;
}

/* can_id-аар slot index буцаах */
static uint8_t prv_FindIndex(uint8_t can_id)
{
    for (uint8_t i = 0U; i < g_wenc.count; i++) {
        if (g_wenc.pool[i].in_use && g_wenc.pool[i].can_id == can_id)
            return i;
    }
    return 0xFFU;
}

static void prv_SendRequest(uint8_t can_id)
{
    CAN_TxHeaderTypeDef hdr;
    uint8_t  data[8];
    uint32_t mailbox;

    hdr.StdId              = can_id;
    hdr.IDE                = CAN_ID_STD;
    hdr.RTR                = CAN_RTR_DATA;
    hdr.DLC                = WENC_POLL_DLC;
    hdr.TransmitGlobalTime = DISABLE;

    core_BuildRequest(can_id, data);

    if (HAL_CAN_GetTxMailboxesFreeLevel(g_wenc.hcan) == 0U) return;
    (void)HAL_CAN_AddTxMessage(g_wenc.hcan, &hdr, data, &mailbox);
}

/* Pending slot дахь frame боловсруулах */
static void prv_ProcessSlot(uint8_t idx)
{
    WEnc_RxSlot_t  *slot = &s_rx[idx];
    WEnc_Encoder_t *e    = &g_wenc.pool[idx];

    if (core_IsEcho(slot->data, slot->dlc))                                  return;
    if (!core_IsValidResponse(slot->std_id, slot->dlc, slot->data, e->can_id)) return;

    int32_t raw = core_DecodeLE(slot->data);
    if (!core_AcceptStep(&e->core, raw))                                     return;

    core_AutoBaseline(&e->core, raw);

    e->raw        = raw;
    e->disp_mm    = core_DisplacementMm(&e->core, raw);
    e->last_rx_ms = HAL_GetTick();
    e->valid      = 1U;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  PUBLIC API
 * ═══════════════════════════════════════════════════════════════════════════ */

void WEncoder_CAN_Init(CAN_HandleTypeDef *hcan)
{
    memset(&g_wenc, 0, sizeof(g_wenc));
    memset(s_rx,    0, sizeof(s_rx));
    g_wenc.hcan        = hcan;
    g_wenc.initialized = 1U;
}

uint8_t WEncoder_CAN_AddEncoder(uint8_t     can_id,
                                 const char *label,
                                 int32_t     total_counts,
                                 float       resolution_mm,
                                 int32_t     max_step)
{
    if (!g_wenc.initialized)               return 0xFFU;
    if (g_wenc.count >= WENC_MAX_ENCODERS) return 0xFFU;
    if (prv_Find(can_id) != NULL)          return 0xFFU;

    uint8_t         idx = g_wenc.count;
    WEnc_Encoder_t *e   = &g_wenc.pool[idx];

    e->can_id = can_id;
    strncpy(e->label, label, sizeof(e->label) - 1U);
    e->label[sizeof(e->label) - 1U] = '\0';
    e->valid  = 0U;
    e->in_use = 1U;

    core_Init(&e->core, can_id, total_counts, resolution_mm, max_step);

    g_wenc.count++;
    return idx;
}

/* -----------------------------------------------------------------------
 * WEncoder_CAN_FilterStart
 *   CAN peripheral аль хэдийн Init + baud тохируулагдсан үед
 *   filter + notification + start хийнэ.
 *   Config tool шиг baud-ийг өөрөө удирддаг тохиолдолд шууд дуудна.
 * ----------------------------------------------------------------------- */
HAL_StatusTypeDef WEncoder_CAN_FilterStart(void)
{
    CAN_FilterTypeDef f = {0};
    /* WENC_FILTER_BANK: CAN1=0, CAN2=14 (STM32F7 dual-CAN)
     * WEncoder_CAN.h-д тохируулна.                           */
    f.FilterBank           = WENC_FILTER_BANK;
    f.FilterMode           = CAN_FILTERMODE_IDMASK;
    f.FilterScale          = CAN_FILTERSCALE_32BIT;
    f.FilterIdHigh         = 0x0000;
    f.FilterIdLow          = 0x0000;
    f.FilterMaskIdHigh     = 0x0000;
    f.FilterMaskIdLow      = 0x0000;
    f.FilterFIFOAssignment = CAN_RX_FIFO0;
    f.FilterActivation     = ENABLE;
    f.SlaveStartFilterBank = 14;   /* CAN1/CAN2 хуваах хил */

    if (HAL_CAN_ConfigFilter(g_wenc.hcan, &f) != HAL_OK)          return HAL_ERROR;
    if (HAL_CAN_ActivateNotification(g_wenc.hcan,
            CAN_IT_RX_FIFO0_MSG_PENDING |
            CAN_IT_BUSOFF               |
            CAN_IT_ERROR                |
            CAN_IT_LAST_ERROR_CODE) != HAL_OK)                     return HAL_ERROR;
    if (HAL_CAN_Start(g_wenc.hcan) != HAL_OK)                     return HAL_ERROR;
    return HAL_OK;
}

/* -----------------------------------------------------------------------
 * WEncoder_CAN_StartStack
 *   Normal ажиллагаанд: WENC_RUN_BAUD тохируулж, reinit хийж,
 *   filter + start хийнэ.
 * ----------------------------------------------------------------------- */
HAL_StatusTypeDef WEncoder_CAN_StartStack(void)
{
    if (prv_ApplyBitTiming(WENC_RUN_BAUD) != HAL_OK) return HAL_ERROR;
    if (HAL_CAN_DeInit(g_wenc.hcan)       != HAL_OK) return HAL_ERROR;
    if (HAL_CAN_Init(g_wenc.hcan)         != HAL_OK) return HAL_ERROR;
    return WEncoder_CAN_FilterStart();
}

void WEncoder_CAN_Poll(void)
{
    static uint32_t last_req[WENC_MAX_ENCODERS] = {0};
    uint32_t now = HAL_GetTick();

    /* 1. Pending frame-үүд боловсруулах */
    for (uint8_t i = 0U; i < g_wenc.count; i++) {
        if (s_rx[i].pending) {
            prv_ProcessSlot(i);
            s_rx[i].pending = 0U;
        }
    }

    /* 2. Read request илгээх */
    for (uint8_t i = 0U; i < g_wenc.count; i++) {
        if (!g_wenc.pool[i].in_use) continue;
        if ((now - last_req[i]) >= WENC_REQUEST_INTERVAL_MS) {
            prv_SendRequest(g_wenc.pool[i].can_id);
            last_req[i] = now;
        }
    }
}

uint8_t WEncoder_CAN_RxCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef hdr;
    uint8_t             data[8];

    if (hcan->Instance != g_wenc.hcan->Instance)                    return 0U;
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &hdr, data) != HAL_OK) return 0U;
    if (hdr.IDE != CAN_ID_STD)                                      return 0U;

    uint8_t idx = prv_FindIndex((uint8_t)hdr.StdId);
    if (idx == 0xFFU) return 0U;

    /* Slot-д хадгалж, pending flag тавина — Poll() боловсруулна */
    s_rx[idx].std_id  = (uint16_t)hdr.StdId;
    s_rx[idx].dlc     = (uint8_t)hdr.DLC;
    memcpy(s_rx[idx].data, data, 8U);
    s_rx[idx].pending = 1U;

    return 1U;
}

uint8_t WEncoder_CAN_GetByIndex(uint8_t index, WEnc_Reading_t *out)
{
    if (index >= g_wenc.count)      return 0U;
    if (!g_wenc.pool[index].in_use) return 0U;

    WEnc_Encoder_t *e = &g_wenc.pool[index];
    out->can_id  = e->can_id;
    out->disp_mm = e->disp_mm;
    out->raw     = e->raw;
    out->valid   = e->valid;
    out->tick    = e->last_rx_ms;
    memcpy(out->label, e->label, sizeof(out->label));
    return 1U;
}

uint8_t WEncoder_CAN_GetById(uint8_t can_id, WEnc_Reading_t *out)
{
    for (uint8_t i = 0U; i < g_wenc.count; i++) {
        if (g_wenc.pool[i].can_id == can_id && g_wenc.pool[i].in_use)
            return WEncoder_CAN_GetByIndex(i, out);
    }
    return 0U;
}

uint8_t WEncoder_CAN_GetCount(void)
{
    return g_wenc.count;
}

uint8_t WEncoder_CAN_IsTimeout(uint8_t index)
{
    if (index >= g_wenc.count)      return 1U;
    if (!g_wenc.pool[index].valid)  return 0U;
    return ((HAL_GetTick() - g_wenc.pool[index].last_rx_ms)
            > WENC_ENCODER_TIMEOUT_MS) ? 1U : 0U;
}

void WEncoder_CAN_ZeroHere(uint8_t index)
{
    if (index >= g_wenc.count)      return;
    if (!g_wenc.pool[index].in_use) return;

    WEnc_Encoder_t *e    = &g_wenc.pool[index];
    e->core.baseline     = e->raw;
    e->core.baseline_set = 1U;
    e->disp_mm           = 0.0f;
}
