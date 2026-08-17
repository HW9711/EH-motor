#ifndef __PRO_H
#define __PRO_H


#define MOTOR_HARD_COMMAND_TIMEOUT_MS 200U /* 手柄控制帧最大租约固定200ms，不再由可调参数延长。 */

#include "mcctl.h"
void Motor_Protect(void);
void Protect_Init(void);
void SystemProtect(void);

#endif
