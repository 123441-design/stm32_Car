#include "remote.h"
#include "car.h"
#include "ps2_usart5.h"
#include "stm32f1xx_hal.h"

#include <string.h>

/* 存放 CH559 发来的一整行文本，预留末尾字符串结束符 '\0'。 */
#define REMOTE_LINE_BUFFER_SIZE 256U

volatile remote_frame_t remote_latest_frame;

/* 这几个变量跨多次 remote_poll() 调用保留尚未收完的一行。 */
static char line_buffer[REMOTE_LINE_BUFFER_SIZE];
static uint16_t line_length;
static uint8_t discarding_line; /* 当前行过长时，丢弃到下一个换行符。 */


/* x4、x5 的中位是 0x80；偏离中位不到 5 时按 0 处理。 */
static int16_t remote_axis(uint8_t raw)
{
    int16_t value = (int16_t)raw - 128;
    if (value > -5 && value < 5) {
        return 0;
    }
    return value;
}

/* 按当前实测的 Xbox 数据，生成小车使用的摇杆和按键状态。 */
static void update_car_remote(const remote_frame_t *frame)
{
    car_remote_t state;
    uint8_t buttons;

    memset(&state, 0, sizeof(state));
    if (frame->protocol != REMOTE_PROTOCOL_XBOX || frame->field_count == 0U) {
        car_remote = state;
        return;
    }

    /* 左摇杆：fields[3]、fields[4]；右摇杆：fields[5]、fields[6]。 */
    if (frame->field_count >= 5U) {
        state.left_x = remote_axis(frame->fields[3]);
        state.left_y = remote_axis(frame->fields[4]);
    }
    if (frame->field_count >= 7U) {
        state.right_x = remote_axis(frame->fields[5]);
        state.right_y = remote_axis(frame->fields[6]);
    }

    /* fields[0] 是按键位图：每一位独立，支持同时按多个键。 */
    buttons = frame->fields[0];
    state.y  = ((buttons & REMOTE_KEY_Y)  != 0U);
    state.b  = ((buttons & REMOTE_KEY_B)  != 0U);
    state.a  = ((buttons & REMOTE_KEY_A)  != 0U);
    state.x  = ((buttons & REMOTE_KEY_X)  != 0U);
    state.l1 = ((buttons & REMOTE_KEY_L1) != 0U);
    state.r1 = ((buttons & REMOTE_KEY_R1) != 0U);
    state.l2 = ((buttons & REMOTE_KEY_L2) != 0U);
    state.r2 = ((buttons & REMOTE_KEY_R2) != 0U);

    if (frame->field_count >= 2U) {
        buttons = frame->fields[1];
        state.select = ((buttons & REMOTE_META_SELECT) != 0U);
        state.start  = ((buttons & REMOTE_META_START)  != 0U);
    }

    /* 左侧方向键使用单个方向编码，0x0F 表示空闲。 */
    if (frame->field_count >= 3U) {
        switch (frame->fields[2]) {
            case REMOTE_DIR_UP:    state.up = 1U; break;
            case REMOTE_DIR_DOWN:  state.down = 1U; break;
            case REMOTE_DIR_LEFT:  state.left = 1U; break;
            case REMOTE_DIR_RIGHT: state.right = 1U; break;
            case REMOTE_DIR_IDLE:  break;
            default: break; /* 其他编码尚未实测。 */
        }
    }

    state.valid = 1U;
    car_remote = state;
}

/* 将单个十六进制字符转换为 0～15；非法字符返回 -1。 */
static int8_t hex_value(char ch)
{
    if (ch >= '0' && ch <= '9') {
        return (int8_t)(ch - '0');
    }
    if (ch >= 'A' && ch <= 'F') {
        return (int8_t)(ch - 'A' + 10);
    }
    if (ch >= 'a' && ch <= 'f') {
        return (int8_t)(ch - 'a' + 10);
    }
    return -1;
}

/*
 * 当前手柄的 HUB0_Xbox data: 字段说明；xN 对应 fields[N-1]。
 * x1：按键位图。Y=0x01，B=0x02，A=0x04，X=0x08，
 *     L1=0x10，R1=0x20，L2=0x40，R2=0x80；可同时按多个键。
 * x2：SELECT=0x01，START=0x02。
 * x3：左侧方向键。上=0x00，下=0x04，左=0x06，右=0x02，空闲=0x0F。
 * x4、x5：左摇杆 X、Y；当前按 0x80 为中位并减去 128，方向待实测核对。
 * x6、x7：右摇杆 X、Y；按同样方法转换。
 * x8 及后续字段：保留在原始帧中，尚未映射到 car_remote。
 */

/* 收到完整的一行后，验证前缀并把每个 xHH 字段转成字节。 */
/*接收的数据格式是以 "HUB0_Xbox data:" 开头的文本行，后面跟着若干个以空格分隔的 xHH 字段，其中 HH 是两位十六进制数。*/
static void parse_line(const char *line)
{
    static const char joystick_prefix[] = "HUB0_Joystick data:";
    static const char xbox_prefix[] = "HUB0_Xbox data:";
    remote_frame_t parsed;
    const char *cursor;
    int8_t high;
    int8_t low;

    memset(&parsed, 0, sizeof(parsed));

    /* 目前只识别已观察到的 Xbox 格式和示例工程的 Joystick 格式。 */
    if (strncmp(line, joystick_prefix, sizeof(joystick_prefix) - 1U) == 0) {
        parsed.protocol = REMOTE_PROTOCOL_JOYSTICK;
        cursor = line + sizeof(joystick_prefix) - 1U;
    } else if (strncmp(line, xbox_prefix, sizeof(xbox_prefix) - 1U) == 0) {
        parsed.protocol = REMOTE_PROTOCOL_XBOX;
        cursor = line + sizeof(xbox_prefix) - 1U;
    } else {
        return;
    }

    /* 字段之间用空格分隔；每个字段必须是 x 加两位十六进制数。 */
    //HUB0_Joystick data: xHH xHH xHH xHH xHH xHH xHH xHH\0
    while (*cursor != '\0') 
    {
        while (*cursor == ' ') 
        {
            cursor++;
        }
        if (*cursor == '\0') 
        {
            break;
        }

        if (parsed.field_count >= REMOTE_MAX_FIELDS || *cursor != 'x' ||
            cursor[1] == '\0' || cursor[2] == '\0') 
        {
            return;
        }

        /* cursor为文本类型，需要转化为数字 */
        high = hex_value(cursor[1]);//接收第一个文本H
        low = hex_value(cursor[2]);//接收第二个文本H
        if (high < 0 || low < 0) 
        {
            return;
        }

        parsed.fields[parsed.field_count++] = (uint8_t)((high << 4) | low);
        cursor += 3;//跳过 xHH
        if (*cursor != '\0' && *cursor != ' ') 
        {
            return;
        }
    }

    if (parsed.field_count == 0U) 
    {
        return;
    }
    /* 所有字段通过校验后，才一次性发布这一帧。 */
    parsed.last_rx_tick = HAL_GetTick();
    parsed.valid = 1U;
    remote_latest_frame = parsed;
    update_car_remote(&parsed);
}


/* 逐字节取出当前已收到的数据；没有数据时立即返回。 */
void remote_poll(void)
{
    uint8_t byte;

    while (ps2_usart5_read(&byte, 1U) == 1U) 
    {
        if (byte == '\n') 
        {
            /* 换行符表示一帧结束；兼容文本末尾的 \r\n。 */
            if (discarding_line == 0U) 
            {
                if (line_length > 0U && line_buffer[line_length - 1U] == '\r') 
                {
                    line_length--;
                }
                line_buffer[line_length] = '\0';
                parse_line(line_buffer);
            }
            line_length = 0U;
            discarding_line = 0U;
        } 
        else if (discarding_line == 0U) 
        {
            /* 尚未遇到换行符，继续把字符追加到当前行。 */
            if (line_length < REMOTE_LINE_BUFFER_SIZE - 1U) 
            {
                line_buffer[line_length++] = (char)byte;
            } 
            else 
            {
                /* 行太长：丢弃当前行，直到下一个换行符再恢复。 */
                line_length = 0U;
                discarding_line = 1U;
            }
        }
    }

    /* 后续控制小车时必须检查 valid，避免断线后继续使用旧指令。 */
    if (remote_latest_frame.valid != 0U &&
        (uint32_t)(HAL_GetTick() - remote_latest_frame.last_rx_tick) > REMOTE_TIMEOUT_MS) 
    {
        remote_latest_frame.valid = 0U;
        /* 收不到新数据时，清除小车遥控指令。 */
        {
            car_remote_t stopped = {0};
            car_remote = stopped;
        }
    }
}
