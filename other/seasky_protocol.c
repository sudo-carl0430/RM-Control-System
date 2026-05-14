/**
 * @file seasky_protocol.c
 * @author Liu Wei
 * @author modified by Neozng
 * @brief 湖南大学RoBoMatster串口通信协议
 * @version 0.1
 * @date 2022-11-03
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "seasky_protocol.h"
#include "crc8.h"
#include "crc16.h"
#include "string.h"

/*获取CRC8校验码*/
uint8_t Get_CRC8_Check(uint8_t *pchMessage,uint16_t dwLength)
{
    return crc_8(pchMessage,dwLength);
}
/*检验CRC8数据段*/
static uint8_t CRC8_Check_Sum(uint8_t *pchMessage, uint16_t dwLength)
{
    uint8_t ucExpected = 0;
    if ((pchMessage == 0) || (dwLength <= 2))
        return 0;
    ucExpected = crc_8(pchMessage, dwLength - 1);
    return (ucExpected == pchMessage[dwLength - 1]);
}

/*获取CRC16校验码*/
uint16_t Get_CRC16_Check(uint8_t *pchMessage,uint32_t dwLength)
{
    return crc_16(pchMessage,dwLength);
}

/*检验CRC16数据段*/
static uint16_t CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength)
{
    uint16_t wExpected = 0;
    if ((pchMessage == 0) || (dwLength <= 2))
    {
        return 0;
    }
    wExpected = crc_16(pchMessage, dwLength - 2);
    return (((wExpected & 0xff) == pchMessage[dwLength - 2]) && (((wExpected >> 8) & 0xff) == pchMessage[dwLength - 1]));
}

/*检验数据帧头*/
static uint8_t protocol_heade_Check(protocol_rm_struct *pro, uint8_t *rx_buf)
{
    if (rx_buf[0] == PROTOCOL_CMD_ID)
    {
        pro->header.sof = rx_buf[0];
        if (CRC8_Check_Sum(&rx_buf[0], 4))
        {
            pro->header.data_length = (rx_buf[2] << 8) | rx_buf[1];
            pro->header.crc_check = rx_buf[3];
            pro->cmd_id = (rx_buf[5] << 8) | rx_buf[4];
            return 1;
        }
    }
    return 0;
}

/*
    此函数根据待发送的数据更新数据帧格式以及内容，实现数据的打包操作
    后续调用通信接口的发送函数发送tx_buf中的对应数据
*/
void get_protocol_send_data(uint8_t send_id,          // 信号id (改为1字节)
                            uint16_t flags_register,  // 16位寄存器
                            float *tx_data,           // 待发送的float数据
                            uint8_t float_length,     // float的数据长度
                            uint8_t *tx_buf,          // 待发送的数据帧
                            uint16_t *tx_buf_len)     // 待发送的数据帧长度
{
    static uint16_t crc16;
    static uint16_t data_len;

    // 计算数据部分长度: 只有float数据，不包括寄存器
    data_len = float_length * 4;
    
    /*帧头部分*/
    tx_buf[0] = 'S';                    // 帧头第一个字节
    tx_buf[1] = 'P';                    // 帧头第二个字节

    /*数据的信号id (1字节)*/
    tx_buf[2] = send_id;                // 信号ID (1字节)

    /*float数据段*/
    for (int i = 0; i < 4 * float_length; i++)
    {
        tx_buf[i + 3] = ((uint8_t *)(&tx_data[i / 4]))[i % 4];
    }

    /*建立16位寄存器 - 放在float数据之后，CRC16之前*/
    tx_buf[3 + data_len] = flags_register & 0xff;        // 寄存器低字节
    tx_buf[3 + data_len + 1] = (flags_register >> 8) & 0xff; // 寄存器高字节

    /*整包校验*/
    // CRC16计算范围: 从帧头开始到数据结束的所有字节（包括新位置的寄存器）
    // 计算长度 = 3字节帧头 + data_len字节数据 + 2字节寄存器
    crc16 = crc_16(&tx_buf[0], 3 + data_len + 2);
    tx_buf[3 + data_len + 2] = crc16 & 0xff;     // CRC16低字节
    tx_buf[3 + data_len + 3] = (crc16 >> 8) & 0xff; // CRC16高字节

    *tx_buf_len = 3 + data_len + 2 + 2; // 总长度 = 3字节帧头 + data_len字节数据 + 2字节寄存器 + 2字节CRC16
}
/*
    此函数用于处理接收数据，
    返回数据内容的id
*/
uint16_t get_protocol_info(uint8_t *rx_buf,          // 接收到的原始数据
                           uint16_t *flags_register, // 接收数据的16位寄存器地址
                           uint8_t *rx_data)         // 接收的float数据存储地址
{
    // 放在静态区,避免反复申请栈上空间
    static protocol_rm_struct pro;
    static uint16_t date_length;

//    if (protocol_heade_Check(&pro, rx_buf))
//    {
        date_length = OFFSET_BYTE + pro.header.data_length;
        // if (CRC16_Check_Sum(&rx_buf[0], date_length))
        // {
            *flags_register = (rx_buf[7] << 8) | rx_buf[6];
            memcpy(rx_data, rx_buf + 8, pro.header.data_length - 2);
            return pro.cmd_id;
        // }
//   }
    return 0;
}
