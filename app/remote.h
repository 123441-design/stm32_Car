#ifndef REMOTE_H
#define REMOTE_H

#include <stdint.h>

/* 一帧中最多保存 32 个 xHH 字段；目前只保存原始值，不推测按键含义。 */
#define REMOTE_MAX_FIELDS 32U
/* 超过 2000 ms 未收到有效帧，就将 valid 置为 0。 */
#define REMOTE_TIMEOUT_MS 2000U

/* 当前实测数据：fields[0] 的每一位表示一个按键，可同时按多个。 */
#define REMOTE_KEY_Y       0x01U
#define REMOTE_KEY_B       0x02U
#define REMOTE_KEY_A       0x04U
#define REMOTE_KEY_X       0x08U
#define REMOTE_KEY_L1      0x10U
#define REMOTE_KEY_L2      0x40U
#define REMOTE_KEY_R1      0x20U
#define REMOTE_KEY_R2      0x80U

/* fields[1]：SELECT 与 START。 */
#define REMOTE_META_SELECT 0x01U
#define REMOTE_META_START  0x02U

/* fields[2] 的左侧方向键使用这些编码。 */
#define REMOTE_DIR_UP      0x00U
#define REMOTE_DIR_DOWN    0x04U
#define REMOTE_DIR_LEFT    0x06U
#define REMOTE_DIR_RIGHT   0x02U
#define REMOTE_DIR_IDLE    0x0FU


/* 根据 CH559 串口文本的开头区分两种数据格式。 */
typedef enum {
    REMOTE_PROTOCOL_NONE = 0,
    REMOTE_PROTOCOL_JOYSTICK,
    REMOTE_PROTOCOL_XBOX
} remote_protocol_t;

/* 最近一帧成功解析的数据，供后续按键映射和调试使用。 */
typedef struct {
    remote_protocol_t protocol;          /* Joystick 或 Xbox 文本格式。 */
    uint8_t fields[REMOTE_MAX_FIELDS];   /* 每个 xHH 转换后的一个字节。 */
    uint8_t field_count;                 /* 本帧实际解析出的字段数。 */
    uint32_t last_rx_tick;               /* 收到本帧时的毫秒计数。 */
    uint8_t valid;                       /* 1：数据仍有效；0：尚无数据或已超时。 */
} remote_frame_t;

/* 可在 Keil Watch 中查看；只有收到并解析完整的一行后才更新。
 * 超时只清除 valid，fields[] 保留上次数据，便于调试。
 */
extern volatile remote_frame_t remote_latest_frame;

/* 在一个任务中周期调用：读空串口环形缓冲区，不阻塞等待新数据。 */
void remote_poll(void);

#endif /* REMOTE_H */
