//
// Created by zz23ecl on 22-7-14.
//

#include "RS232.h"

//CRC cal
uint8_t RS232::CalcCRCByte(uint8_t u8Byte, uint8_t u8CRC) {
    uint8_t i;
    u8CRC = u8CRC ^ u8Byte;
    for (i = 0; i <8 ; i++) {
        if (u8CRC & 0x01) {
            u8CRC = (u8CRC >> 1) ^ 0xD5;
        }
        else {
            u8CRC >>= 1;
        }
    }
    return u8CRC;
}

//读取接收到的字符串
uint8_t RS232::readBuffer(uint8_t *temp, uint32_t *value) {

    uint8_t length = *(temp + 1);
    uint8_t node = *(temp + 2);
    uint8_t command = *(temp + 3);
    uint8_t crc = *(temp + length - 2);
    uint16_t index;
    uint8_t subindex;
    uint16_t statusword;
    uint8_t type;
    uint32_t v = 0;
    //这里只有结点1
    if (node != 0x01) {
        printf("Error node %x", node);
        return 0;
    }
    switch (command) {
        //比较关键的就是0x01和0x05 两个部分
        //0x01 Response of SDO read service
        //控制参数speed pos这些就在这里
        case 0x01: {
            index = ((*(temp + 5)) << 8) + (*(temp + 4));
            subindex = *(temp + 6);
            for (uint8_t i = 0; i < length - 7; i++)
                v += *(temp + 7 + i) << (i * 8);
            *value = v;
            switch (index) {
                //实时位置
                case 0x6064: {
                    type = 1;
                    break;
                }
                    //实时速度
                case 0x606C: {
                    type = 2;
                    break;
                }
                    //实时力矩
                case 0x6077: {
                    type = 3;
                    break;
                }
                    //目标位置
                case 0x607A: {
                    type = 4;
                    break;
                }
                    //目标速度
                case 0x60FF: {
                    type = 5;
                    break;
                }
            }
            break;
        }
            //0x02 Response of SDO write request
        case 0x02: {
            index = ((*(temp + 5)) << 8) + (*(temp + 4));
            subindex = *(temp + 6);
            //这里的返回的是他们的object entry，不需要使用
            break;
        }
            //0x04  Response of write controlword
        case 0x04: {
            if (*(temp + 4) == 0) {
                printf("Error code=0,is OK!");
            }
            break;
        }
            //0x05 Receive statusword  U16
        case 0x05: {
            subindex = *(temp + 6);
            statusword = (*(temp + 7)) + ((*(temp + 8)) << 8);
            statusword = statusword & 0x00FF;
            //闭锁
            if (statusword & 0x40)
                type = 6;
            else {
                //待机
                if (!(statusword - 0x21) & 0xF)
                    type = 7;
                //开机
                if (!(statusword - 0x23) & 0xF)
                    type = 8;
                //操作
                if (!(statusword & 0x27) & 0xF)
                    type = 9;
            }
            break;
        }
        default: {
            printf("unknow message!");
            break;
        }
    }
    return type;
}

//写控制字
void RS232::writeControlWord(uint8_t temp[], uint8_t ctrlwlb) {
    temp[0] = 'S';
    temp[1] = 0x06;
    temp[2] = 0x01;
    temp[3] = 0x04;
    temp[4] = ctrlwlb;
    temp[5] = 0x00;
    uint8_t CRC = 0xFF;
    for (uint8_t i = 1; i <6; i++)
        CRC = CalcCRCByte(temp[i], CRC);
    temp[6] = CRC;
    temp[7] = 'E';
}

//写控制模式
void RS232::writeOperationMode(uint8_t temp[],int8_t type){
    uint8_t length = 0x08;
    temp[0] = 'S';
    temp[1] = 0x08;
    temp[2] = 0x01;
    temp[3] = 0x02;
    temp[4] = 0x60;
    temp[5] = 0x60;
    temp[6] = 0x00;
    if(type==1)
        temp[7]=0x01;
    if(type==3)
        temp[7] =0X03;
    uint8_t CRC = 0xFF;
    for (uint8_t i = 1; i <length; i++)
        CRC = CalcCRCByte(temp[i], CRC);
    temp[8] =CRC;
    temp[9]='E';
}
//写目标数据
void RS232::getData(uint8_t temp[], int32_t data, uint8_t type) {
    uint8_t length = 0x0b;
    uint8_t command = 0x02;
    uint8_t NodeNum = 0x01;
    uint8_t indexHB;
    uint8_t indexLB;
    uint8_t subindex;
    switch (type) {
        //速度
        case 1: {
            subindex = 0x00;
            indexLB = 0xFF;
            indexHB = 0x60;
            break;
        }
            //位置
        case 2: {
            subindex = 0x00;
            indexLB = 0x7A;
            indexHB = 0x60;
            break;
        }
        default: {
            printf("Erorr Data Type!");
            break;
        }
    }

    temp[0] = 'S';
    temp[1] = length;
    temp[2] = NodeNum;
    temp[3] = command;
    temp[4] = indexLB;
    temp[5] = indexHB;
    temp[6] = 0x00;
    /********************************************/
    /*  这里有个问题是写入数据是按低位写还是高位优先写？ */
    /*               问题解决，按低位写            */
    /********************************************/
    //先按照低位先的来
    temp[7] = data & 0xff;
    temp[8] = data >> 8 & 0xff;
    temp[9] = data >> 16 & 0xff;
    temp[10] = data >> 24 & 0xff;
    uint8_t CRC = 0xFF;
    for (int i = 1; i < length; i++)
        CRC = CalcCRCByte(temp[i], CRC);
    //printf("%x\n",CRC);
    temp[11] = CRC;
    temp[12] = 'E';
}

//得到发送希望得到数据的请求的字符串
void RS232::getDataRequest(uint8_t temp[], uint8_t type) {
    uint8_t indexLB;
    uint8_t indexHB;
    uint8_t CRC = 0xFF;
    uint8_t length = 0x07;
    //实际转矩
    if (type == 1) {
        indexLB = 0x77;
        indexHB = 0x60;
    }
        //实际速度
    else if (type == 2) {
        indexLB = 0x6C;
        indexHB = 0x60;
    }
        //实际位置
    else if (type == 3) {
        indexLB = 0x64;
        indexHB = 0x60;
    }
    temp[0] = 'S';
    temp[1] = length;
    temp[2] = 0x01;
    temp[3] = 0x01;
    temp[4] = indexLB;
    temp[5] = indexHB;
    temp[6] = 0x00;
    for (int i = 1; i <length; i++)
        CRC = CalcCRCByte(temp[i], CRC);
    temp[7] = CRC;
    temp[8] = 'E';
}

// 十六进制字符转换数字
int RS232::str2data(string str) {
    int Data;
    size_t cnt = str.size();
    for (int i = 0; cnt; i++, cnt--) {
        if (!str[i]) {
            return false;
        }
        int v = 0;
        if (is_hex(str[i], v)) {
            Data = Data * 16 + v;
        }
    }
    return Data;
}

//判断是否为十六进制字符
bool RS232::is_hex(char c, int &v) {
    if (0x20 <= c && isdigit(c)) {
        v = c - '0';
        return true;
    } else if ('A' <= c && c <= 'F') {
        v = c - 'A' + 10;
        return true;
    } else if ('a' <= c && c <= 'f') {
        v = c - 'a' + 10;
        return true;
    }
    return false;
}