//
// Created by zz23ecl on 22-7-14.
//

#ifndef FTDI_TEST_RS232_H
#define FTDI_TEST_RS232_H

#include <iostream>

using namespace std;



class RS232{
public:
    RS232(void);
    ~RS232(void);

public:
    uint8_t CalcCRCByte(uint8_t u8Byte,uint8_t u8CRC);
    /** CRC计算
    *
    *
    *  @param:  u8 Byte CRC
    *  @return: u8  CRC
    *  @note:
    *  @see:
    */

    uint8_t readBuffer(uint8_t *temp,uint32_t *value);
    /*******************************************/
    /** 读取FaulHaber发送的字符串
    *
    *
    *  @param:  u8*  Buffer 即接收到的字符串
    *          u32*  value  得到的数据大小
    *
    *  @return: u8   type 返回的数据类型
    *                     1：实时位置  2：实时速度  3：实时力矩   4:目标位置 5：目标速度 6-9：statusword
    *                     6：闭锁 7：待机 8：开机 9：操作
    *  @note:
    *  @see:
    */

    void writeControlWord(uint8_t temp[],uint8_t );
    /*******************************************/
    /** 生成向FaulHaber写控制字的字符串
    *
    *
    *  @param:
    *  u8* temp 储存生成的数组
    *  u8   controlword  0f 0d 06 07....
    *  @return:
    *  @note:
    *  @see:
    */

    void writeOperationMode(uint8_t temp[],int8_t type);
    /*******************************************/
    /** 生成向FaulHaber修改控制模式的字符串
    *
    *
    *  @param:
    *  u8* temp 储存生成的数组
    *  u8  type 控制模式的种类
    *               1：PP
    *               3：PV
    *  @return:
    *  @note:
    *  @see:
    */

    void getData(uint8_t temp[],int32_t data,uint8_t type);
    /*******************************************/
    /** 生成向FaulHaber写数据的字符串
    *
    *
    *  @param:
    *  u8* temp 储存生成的数组
    *  int data 需要写入的数据
    *  u8  type 数据类型
    *               1：目标速度
    *               2：目标位置
    *  @return:
    *  @note:
    *  @see:
    */

    void getDataRequest(uint8_t temp[],uint8_t type);
    /*******************************************/
    /** 生成向FaulHaber发送希望得到数据的命令请求
    *
    *
    *  @param:
    *  u8* temp 储存生成的数组
    *  u8  type 数据类型
    *               1：实际转矩
    *               2：实际速度
    *               3：实际转矩
    *  @return:
    *  @note:  使用方法，先构造一个长度为九的u8数组 uint8_t temp[8];getDataRequest(temp,0x01);
    *  @see:
    */

    int str2data(string str);
    /*******************************************/
    /** 将串口监听到的十六进制字符串转化为具体的数据大小
    *
    *
    *  @param:
    *  string str 十六进制字符串
    *  @return:
    *  int data 字符串得到的数据
    *  @note:
    *  @see:
    */

    bool is_hex(char c, int& v);
    /*******************************************/
    /** 字符转换用的子函数
    *
    *
    *  @param:
    *  char C 输入的字符 int& v要得到数据
    *  @return:
    *  bool 是否为十六进制字符
    *  @note:
    *  @see:
    */

};




#endif //FTDI_TEST_RS232_H
