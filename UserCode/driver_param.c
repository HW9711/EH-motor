#include "driver_param.h"

#include "mcctl.h"
#include "mcflash.h"
#include "protect.h"
#include "YSmcctl.h"
#include "YSprotect.h"
#include "YSstatemachine.h"
#include "youshua.h"

#include <string.h>

#define DRIVER_PARAM_ADDRESS 0xFDU /* 固定维护地址不跟随参数0中的旧通信地址变化。 */
#define DRIVER_PARAM_REQUEST_LEN 8U /* 读、写和Flash命令都使用固定8字节Modbus请求。 */
#define DRIVER_PARAM_READ_MAX 30U /* 单次最多读取30个16位参数，响应不超过串口70字节缓冲。 */
#define DRIVER_PARAM_BRUSHLESS_COUNT 67U /* 无刷只开放当前明确使用的0~66。 */
#define DRIVER_PARAM_BRUSHED_COUNT 62U /* 有刷只开放当前明确使用的0~61。 */
#define DRIVER_PARAM_VALUE_MAX 32767U /* 拒绝旧int16_t路径会回绕的0x8000~0xFFFF。 */
#define DRIVER_PARAM_COMMAND_BANK 0x7FU /* 0x7F00寄存器固定承载保存和恢复默认命令。 */
#define DRIVER_PARAM_ACTION_SAVE 0x01U /* 保存指定Bank当前RAM值到Flash。 */
#define DRIVER_PARAM_ACTION_DEFAULT 0x02U /* 立即恢复编译默认值并擦除指定Flash页。 */
#define DRIVER_PARAM_FLASH_MARK 0xAADDU /* 复用现有参数页有效标记。 */

typedef struct
{
    uint16_t *runtime_values; /* 控制算法实际读取的运行参数数组。 */
    uint16_t *default_values; /* Flash空白或恢复默认时复制的编译参数数组。 */
    uint16_t valid_count; /* 本版协议允许读写的有效索引数量。 */
    uint32_t flash_address; /* 当前Bank独立参数页首地址。 */
    uint8_t brushed; /* 1表示有刷参数，写入后重载有刷保护缓存。 */
} DriverParamBank_t;

/*
 * 函数功能：把Bank编码映射到运行数组、默认数组、有效数量和Flash页。
 * 输入参数：bank为0x10~0x13；result接收映射结果。
 * 返回参数：Bank有效返回1；未知Bank或空输出指针返回0。
 */
static uint8_t DriverParam_GetBank(uint8_t bank, DriverParamBank_t *result)
{
    if (result == NULL)
    {
        return 0U; /* 没有输出空间时不能继续处理维护请求。 */
    }

    switch (bank)
    {
        case 0x10U:
            result->runtime_values = MCPara; /* 无刷通道1运行参数。 */
            result->default_values = SMCPara; /* 无刷通道1编译默认。 */
            result->valid_count = DRIVER_PARAM_BRUSHLESS_COUNT; /* 仅开放0~66。 */
            result->flash_address = TEST_FLASH_ADDRESS_START; /* 沿用原无刷通道1页99。 */
            result->brushed = 0U;
            return 1U;

        case 0x11U:
            result->runtime_values = MCPara2; /* 无刷通道2运行参数。 */
            result->default_values = SMCPara2; /* 无刷通道2编译默认。 */
            result->valid_count = DRIVER_PARAM_BRUSHLESS_COUNT; /* 仅开放0~66。 */
            result->flash_address = TEST_FLASH_ADDRESS_START2; /* 沿用原无刷通道2页101。 */
            result->brushed = 0U;
            return 1U;

        case 0x12U:
            result->runtime_values = YSPara1; /* 有刷通道1运行参数。 */
            result->default_values = SYSPara1; /* 有刷通道1编译默认。 */
            result->valid_count = DRIVER_PARAM_BRUSHED_COUNT; /* 仅开放0~61。 */
            result->flash_address = TEST_FLASH_ADDRESS_START_YS1; /* 新增独立页103。 */
            result->brushed = 1U;
            return 1U;

        case 0x13U:
            result->runtime_values = YSPara2; /* 有刷通道2运行参数。 */
            result->default_values = SYSPara2; /* 有刷通道2编译默认。 */
            result->valid_count = DRIVER_PARAM_BRUSHED_COUNT; /* 仅开放0~61。 */
            result->flash_address = TEST_FLASH_ADDRESS_START_YS2; /* 新增独立页104。 */
            result->brushed = 1U;
            return 1U;

        default:
            return 0U; /* 未知Bank不允许落到任意内存地址。 */
    }
}

/*
 * 函数功能：判断无刷两路和有刷两路是否全部处于安全停机状态。
 * 输入参数：无。
 * 返回参数：全部停机返回1；任一路仍允许运行或状态机未回WAIT返回0。
 */
static uint8_t DriverParam_IsStopped(void)
{
    if ((App.FB.AllRun != 0U) || (App.FB2.AllRun != 0U))
    {
        return 0U; /* 任一路无刷仍在运行时禁止写参数和擦写Flash。 */
    }

    if ((App2.Ch1.AllowRun != 0U) || (App2.Ch2.AllowRun != 0U) ||
        (App2.Ch1.Status != CTLS_WAIT) || (App2.Ch2.Status != CTLS_WAIT))
    {
        return 0U; /* 有刷PWM尚未完全回到WAIT时同样禁止维护写操作。 */
    }

    return 1U;
}

/*
 * 函数功能：写参数或恢复默认后刷新停机状态下可以安全重载的保护和位置缓存。
 * 输入参数：bank_info描述本次修改的是有刷还是无刷参数。
 * 返回参数：无。
 */
static void DriverParam_Apply(const DriverParamBank_t *bank_info)
{
    if (bank_info->brushed != 0U)
    {
        YSProtect_Init(); /* 有刷过流阈值和实验确定的保护时间按新RAM值原样重载，不做联动换算。 */
    }
    else
    {
        Protect_Init(); /* 无刷系统保护和两通道过流缓存按当前参数表重载。 */
        Pos_Uart_Init(); /* 无刷位置与停止延时参数在停机状态下同步重载。 */
    }
}

/*
 * 函数功能：校验指定Flash参数页的标记和145个16位参数是否与RAM完全一致。
 * 输入参数：bank_info提供页地址和运行数组。
 * 返回参数：全部一致返回1；任一字不一致返回0。
 */
static uint8_t DriverParam_VerifyFlash(const DriverParamBank_t *bank_info)
{
    uint16_t index; /* 逐项复核，防止Flash底层失败却回成功帧。 */
    volatile const uint16_t *flash_values = (volatile const uint16_t *)(bank_info->flash_address + 2U); /* 首半字是0xAADD标记，参数从偏移2开始。 */

    if (*(volatile const uint16_t *)bank_info->flash_address != DRIVER_PARAM_FLASH_MARK)
    {
        return 0U; /* 页标记没有写成0xAADD时保存失败。 */
    }

    for (index = 0U; index < ParaNum; ++index)
    {
        if (flash_values[index] != bank_info->runtime_values[index])
        {
            return 0U; /* 任一参数不一致都不能向主控确认保存成功。 */
        }
    }

    return 1U;
}

/*
 * 函数功能：发送固定地址0xFD的Modbus异常响应。
 * 输入参数：USARTx为发送串口；seruart提供发送缓冲；function为异常功能码；error为异常原因。
 * 返回参数：无。
 */
static void DriverParam_SendError(USART_Type *USARTx, MCUART_Type *seruart, uint8_t function, uint8_t error)
{
    seruart->T_DATA[0] = DRIVER_PARAM_ADDRESS; /* 异常响应仍使用固定维护地址。 */
    seruart->T_DATA[1] = function; /* 03错误用0x83，06错误用0x86。 */
    seruart->T_DATA[2] = error; /* 02表示地址/参数非法，04表示设备或Flash失败，06表示运行中忙。 */
    seruart->TxCRC = CRC_Calc(seruart->T_DATA, 3U); /* 对前三字节计算Modbus CRC。 */
    seruart->T_DATA[3] = (uint8_t)(seruart->TxCRC & 0xFFU); /* CRC低字节先发。 */
    seruart->T_DATA[4] = (uint8_t)(seruart->TxCRC >> 8U); /* CRC高字节后发。 */
    Modbus_Send(USARTx, seruart->T_DATA, 5U); /* 异常响应固定5字节。 */
}

/*
 * 函数功能：发送06成功回显，前6字节与请求完全一致并重新计算CRC。
 * 输入参数：USARTx为发送串口；seruart提供原请求和发送缓冲。
 * 返回参数：无。
 */
static void DriverParam_SendWriteOk(USART_Type *USARTx, MCUART_Type *seruart)
{
    memcpy(seruart->T_DATA, seruart->R_DATA, 6U); /* Modbus 06成功响应必须回显地址、寄存器和数值。 */
    seruart->TxCRC = CRC_Calc(seruart->T_DATA, 6U); /* 对回显前6字节重新计算CRC。 */
    seruart->T_DATA[6] = (uint8_t)(seruart->TxCRC & 0xFFU); /* CRC低字节先发。 */
    seruart->T_DATA[7] = (uint8_t)(seruart->TxCRC >> 8U); /* CRC高字节后发。 */
    Modbus_Send(USARTx, seruart->T_DATA, DRIVER_PARAM_REQUEST_LEN); /* 成功响应固定8字节。 */
}

/*
 * 函数功能：从独立Flash页装载两路有刷参数，保持无刷原有装载路径不变。
 * 输入参数：无。
 * 返回参数：无。
 */
void DriverParam_Init(void)
{
    F4Flash_SMCPara_Read(TEST_FLASH_ADDRESS_START_YS1, YSPara1, ParaNum, SYSPara1); /* 页103有效则读Flash，否则复制通道1编译默认。 */
    F4Flash_SMCPara_Read(TEST_FLASH_ADDRESS_START_YS2, YSPara2, ParaNum, SYSPara2); /* 页104有效则读Flash，否则复制通道2编译默认。 */
}

/*
 * 函数功能：处理固定0xFD维护帧，完整隔离原地址1/2、0xEE和0xAA控制协议。
 * 输入参数：USARTx为当前串口；seruart为串口接收状态。
 * 返回参数：识别到0xFD返回1并阻止旧协议继续解析；其它地址返回0。
 */
uint8_t DriverParam_TryHandle(USART_Type *USARTx, MCUART_Type *seruart)
{
    DriverParamBank_t bank_info; /* 保存本帧Bank映射。 */
    uint8_t bank; /* 保存请求Bank。 */
    uint8_t index; /* 保存读写起始索引。 */
    uint16_t value; /* 保存06写入原始值。 */
    uint8_t count; /* 保存03读取数量。 */
    uint8_t data_offset; /* 保存03响应数据偏移。 */
    uint8_t item; /* 逐项复制读取参数。 */

    if ((seruart == NULL) || (seruart->Addr != DRIVER_PARAM_ADDRESS))
    {
        return 0U; /* 非维护地址原样交回旧协议处理。 */
    }

    if ((seruart->RxLen != DRIVER_PARAM_REQUEST_LEN) || (seruart->CalcCRC != seruart->RxCRC))
    {
        return 1U; /* 维护协议要求精确8字节和真实CRC，不保留旧0xAABB旁路。 */
    }

    if (seruart->FunCode == 0x03U)
    {
        bank = seruart->R_DATA[2]; /* 03寄存器高字节直接表示Bank。 */
        index = seruart->R_DATA[3]; /* 03寄存器低字节表示起始索引。 */
        count = seruart->R_DATA[5]; /* 数量低字节表示连续参数数量。 */
        if ((seruart->R_DATA[4] != 0U) || (count == 0U) || (count > DRIVER_PARAM_READ_MAX) ||
            (DriverParam_GetBank(bank, &bank_info) == 0U) ||
            ((uint16_t)index + count > bank_info.valid_count))
        {
            DriverParam_SendError(USARTx, seruart, 0x83U, 0x02U); /* 数量或范围非法返回地址错误。 */
            return 1U;
        }

        seruart->T_DATA[0] = DRIVER_PARAM_ADDRESS; /* 03响应固定维护地址。 */
        seruart->T_DATA[1] = 0x03U; /* 回显批量读功能码。 */
        seruart->T_DATA[2] = (uint8_t)(count * 2U); /* 字节数是参数数量的两倍。 */
        for (item = 0U; item < count; ++item)
        {
            data_offset = (uint8_t)(3U + item * 2U); /* 每个参数在响应中占两个连续字节。 */
            seruart->T_DATA[data_offset] = (uint8_t)(bank_info.runtime_values[index + item] >> 8U); /* 参数高字节先发。 */
            seruart->T_DATA[data_offset + 1U] = (uint8_t)(bank_info.runtime_values[index + item] & 0xFFU); /* 参数低字节后发。 */
        }
        seruart->SendLen = (uint8_t)(3U + count * 2U); /* CRC前响应长度。 */
        seruart->TxCRC = CRC_Calc(seruart->T_DATA, seruart->SendLen); /* 对完整响应数据计算CRC。 */
        seruart->T_DATA[seruart->SendLen] = (uint8_t)(seruart->TxCRC & 0xFFU); /* CRC低字节先发。 */
        seruart->T_DATA[seruart->SendLen + 1U] = (uint8_t)(seruart->TxCRC >> 8U); /* CRC高字节后发。 */
        Modbus_Send(USARTx, seruart->T_DATA, (uint16_t)seruart->SendLen + 2U); /* 发送完整03响应。 */
        return 1U;
    }

    if (seruart->FunCode != 0x06U)
    {
        DriverParam_SendError(USARTx, seruart, (uint8_t)(seruart->FunCode | 0x80U), 0x01U); /* 维护地址只支持03和06。 */
        return 1U;
    }

    if (DriverParam_IsStopped() == 0U)
    {
        DriverParam_SendError(USARTx, seruart, 0x86U, 0x06U); /* 任一路未停机返回设备忙。 */
        return 1U;
    }

    if ((seruart->R_DATA[2] == DRIVER_PARAM_COMMAND_BANK) && (seruart->R_DATA[3] == 0U))
    {
        bank = seruart->R_DATA[4]; /* Flash命令的数值高字节指定Bank。 */
        if (DriverParam_GetBank(bank, &bank_info) == 0U)
        {
            DriverParam_SendError(USARTx, seruart, 0x86U, 0x02U); /* 未知Bank拒绝Flash操作。 */
            return 1U;
        }

        if (seruart->R_DATA[5] == DRIVER_PARAM_ACTION_SAVE)
        {
            FLASH_Write(bank_info.flash_address, bank_info.runtime_values, ParaNum); /* 保存完整145项以保持现有Flash布局一致。 */
            if (DriverParam_VerifyFlash(&bank_info) == 0U)
            {
                DriverParam_SendError(USARTx, seruart, 0x86U, 0x04U); /* 写后复核失败不能回成功。 */
                return 1U;
            }
        }
        else if (seruart->R_DATA[5] == DRIVER_PARAM_ACTION_DEFAULT)
        {
            memcpy(bank_info.runtime_values, bank_info.default_values, sizeof(uint16_t) * ParaNum); /* 默认值立即进入RAM，不要求重启。 */
            F4Flash_SMCPara_Erase(bank_info.flash_address); /* 擦除保存页，使下次上电继续使用编译默认。 */
            if (*(volatile const uint16_t *)bank_info.flash_address == DRIVER_PARAM_FLASH_MARK)
            {
                DriverParam_SendError(USARTx, seruart, 0x86U, 0x04U); /* 标记仍有效说明擦除失败。 */
                return 1U;
            }
            DriverParam_Apply(&bank_info); /* 恢复默认后立即重载安全缓存。 */
        }
        else
        {
            DriverParam_SendError(USARTx, seruart, 0x86U, 0x02U); /* 只接受01保存和02恢复默认。 */
            return 1U;
        }

        DriverParam_SendWriteOk(USARTx, seruart); /* Flash动作完成并验证后才回显成功。 */
        return 1U;
    }

    bank = seruart->R_DATA[2]; /* 普通06寄存器高字节表示Bank。 */
    index = seruart->R_DATA[3]; /* 普通06寄存器低字节表示参数索引。 */
    value = ((uint16_t)seruart->R_DATA[4] << 8U) | seruart->R_DATA[5]; /* 参数值按大端拼成无符号16位。 */
    if ((DriverParam_GetBank(bank, &bank_info) == 0U) || (index == 0U) ||
        (index >= bank_info.valid_count) || (value > DRIVER_PARAM_VALUE_MAX))
    {
        DriverParam_SendError(USARTx, seruart, 0x86U, 0x02U); /* 地址索引0只读，且拒绝越界和高位回绕值。 */
        return 1U;
    }

    bank_info.runtime_values[index] = value; /* 只修改明确选择的一个参数，不联动改变任何保护时间。 */
    DriverParam_Apply(&bank_info); /* 停机状态下立即重载保护和位置缓存。 */
    DriverParam_SendWriteOk(USARTx, seruart); /* 写RAM和重载完成后回显成功。 */
    return 1U;
}
