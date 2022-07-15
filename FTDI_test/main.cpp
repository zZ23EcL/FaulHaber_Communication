#include <iostream>
#include <thread>
#include <chrono>
#include "ftd2xx.h"
#include "RS232.h"
using namespace std;


int main() {
    /***********************/
    /*         FTDI        */
    /***********************/
    bool receivable=false;
    FT_HANDLE ftHandle;
    FT_STATUS ftStatus;
    DWORD EventDWord;
    DWORD TxBytes;
    DWORD RxBytes;
    DWORD BytesReceived;
    DWORD BytesWritten;
    char RxBuffer[256];
    /***********************/
    /*       FaulHaber     */
    /***********************/
    RS232 FH;
    unsigned char controlw[8];
    unsigned char setMode[10];
    unsigned char data[13];


    ftStatus = FT_Open(0,&ftHandle);
    if (ftStatus == FT_OK) {
        printf("connect succesed...");
        receivable=true;
        thread ([&]{
            while(receivable)
            {
                FT_GetStatus(ftHandle,&RxBytes,&TxBytes,&EventDWord);
                if(RxBytes>0){
                    ftStatus= FT_Read(ftHandle,RxBuffer,RxBytes,&BytesReceived);
                }
                //deal with buffer,get data from here such as actual pos and speed,bytesreceived is num,rxbuffer is data
            }
        });
    }
    else {
        printf("Error! Can not open!");
    }
    //start
    FH.writeControlWord(controlw,0x06);
    FT_Write(ftHandle,controlw,sizeof(controlw),&BytesWritten);
    FH.writeControlWord(controlw,0x07);
    FT_Write(ftHandle,controlw,sizeof(controlw),&BytesWritten);
    FH.writeControlWord(controlw,0x0F);
    FT_Write(ftHandle,controlw,sizeof(controlw),&BytesWritten);

    //PV Mode
    FH.writeOperationMode(setMode,0x03);
    FT_Write(ftHandle,setMode,sizeof(setMode),&BytesWritten);

    FH.getData(data,-1000,1);
    FT_Write(ftHandle,data,sizeof(data),&BytesWritten);

    //cancel enable
    FH.writeControlWord(controlw,0x0D);
    FT_Write(ftHandle,controlw,sizeof(controlw),&BytesWritten);
    getchar();
    FT_Close(ftHandle);
    return 0;
}
