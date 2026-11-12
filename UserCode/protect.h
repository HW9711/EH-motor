#ifndef __PRO_H
#define __PRO_H


#define MOTOR_HARD_COMMAND_TIMEOUT_MS 200U /* 手柄控制帧最大租约固定200ms，不再由可调参数延长。 */

#include "mcctl.h"
#define MOTOR_OVERCURRENT_TIME_DEFAULT_MS 500U /* 参数22/24缺失、越界和旧协议均回退500ms。 */
#define MOTOR_OVERCURRENT_TIME_MIN_MS 500U /* 两项独立合法范围下限，单位ms。 */
#define MOTOR_OVERCURRENT_TIME_MAX_MS 1500U /* 两项独立合法范围上限，不允许长延时进入保护缓存。 */
uint16_t Protect_ValidateTime(uint32_t value); /* 校验完整数值后再转16位，防止截断绕过范围。 */
void Protect_ApplyCommandTimes(uint8_t motor_kind, uint16_t time22, uint16_t time24, uint8_t stopped); /* 只更新RAM时间，不清过流计数、不写Flash。 */
void Motor_Protect(void);
void Protect_Init(void);
void SystemProtect(void);

#endif
