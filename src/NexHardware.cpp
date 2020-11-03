/**
 * @file NexHardware.cpp
 *
 * The implementation of base API for using Nextion. 
 *
 * @author  Wu Pengfei (email:<pengfei.wu@itead.cc>)
 * @date    2015/8/11
 * @copyright 
 * Copyright (C) 2014-2015 ITEAD Intelligent Systems Co., Ltd. \n
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 */
#include "NexHardware.h"

#define NEX_RET_CMD_FINISHED (0x01)
#define NEX_RET_EVENT_LAUNCHED (0x88)
#define NEX_RET_EVENT_UPGRADED (0x89)
#define NEX_RET_EVENT_TOUCH_HEAD (0x65)
#define NEX_RET_EVENT_POSITION_HEAD (0x67)
#define NEX_RET_EVENT_SLEEP_POSITION_HEAD (0x68)
#define NEX_RET_CURRENT_PAGE_ID_HEAD (0x66)
#define NEX_RET_STRING_HEAD (0x70)
#define NEX_RET_NUMBER_HEAD (0x71)
#define NEX_RET_INVALID_CMD (0x00)
#define NEX_RET_INVALID_COMPONENT_ID (0x02)
#define NEX_RET_INVALID_PAGE_ID (0x03)
#define NEX_RET_INVALID_PICTURE_ID (0x04)
#define NEX_RET_INVALID_FONT_ID (0x05)
#define NEX_RET_INVALID_BAUD (0x11)
#define NEX_RET_INVALID_VARIABLE (0x1A)
#define NEX_RET_INVALID_OPERATION (0x1B)

/*
 * Receive uint32_t data. 
 * 
 * @param number - save uint32_t data. 
 * @param timeout - set timeout time. 
 *
 * @retval true - success. 
 * @retval false - failed.
 *
 */
bool recvRetNumber(uint32_t *number, uint32_t timeout)
{
    bool ret = false;
    uint8_t temp[8] = {0};

    if (!number)
    {
        goto __return;
    }

    nexSerial.setTimeout(timeout);
    if (sizeof(temp) != nexSerial.readBytes((char *)temp, sizeof(temp)))
    {
        goto __return;
    }

    if (temp[0] == NEX_RET_NUMBER_HEAD && temp[5] == 0xFF && temp[6] == 0xFF && temp[7] == 0xFF)
    {
        *number = ((uint32_t)temp[4] << 24) | ((uint32_t)temp[3] << 16) | (temp[2] << 8) | (temp[1]);
        ret = true;
    }

__return:

    if (ret)
    {
        dbSerialPrint("recvRetNumber :");
        dbSerialPrintln(*number);
    }
    else
    {
        dbSerialPrintln("recvRetNumber err");
        printError(temp);
    }

    return ret;
}

/*
 * Receive string data. 
 * 
 * @param buffer - save string data. 
 * @param len - string buffer length. 
 * @param timeout - set timeout time. 
 *
 * @return the length of string buffer.
 *
 */
uint16_t recvRetString(char *buffer, uint16_t len, uint32_t timeout)
{
    uint16_t ret = 0;
    bool str_start_flag = false;
    uint8_t cnt_0xff = 0;
    String temp = String("");
    uint8_t c = 0;
    long start;

    if (!buffer || len == 0)
    {
        goto __return;
    }

    start = millis();
    while (millis() - start <= timeout)
    {
        while (nexSerial.available())
        {
            c = nexSerial.read();
            if (str_start_flag)
            {
                if (0xFF == c)
                {
                    cnt_0xff++;
                    if (cnt_0xff >= 3)
                    {
                        break;
                    }
                }
                else
                {
                    temp += (char)c;
                }
            }
            else if (NEX_RET_STRING_HEAD == c)
            {
                str_start_flag = true;
            }
        }

        if (cnt_0xff >= 3)
        {
            break;
        }
    }

    ret = temp.length();
    ret = ret > len ? len : ret;
    strncpy(buffer, temp.c_str(), ret);

__return:

    dbSerialPrint("recvRetString[");
    dbSerialPrint(temp.length());
    dbSerialPrint(",");
    dbSerialPrint(temp);
    dbSerialPrintln("]");

    return ret;
}

/*
 * Send command to Nextion.
 *
 * @param cmd - the string of command.
 */
void sendCommand(const char *cmd)
{
    while (nexSerial.available())
    {
        nexSerial.read();
    }

    nexSerial.print(cmd);
    //nexSerial.write(cmd);
    nexSerial.write(0xFF);
    nexSerial.write(0xFF);
    nexSerial.write(0xFF);
}

/*
 * Command is executed successfully. 
 *
 * @param timeout - set timeout time.
 *
 * @retval true - success.
 * @retval false - failed. 
 *
 */
bool recvRetCommandFinished(uint32_t timeout)
{
    bool ret = false;
    uint8_t temp[4] = {0};

    nexSerial.setTimeout(timeout);
    if (sizeof(temp) != nexSerial.readBytes((char *)temp, sizeof(temp)))
    {
        ret = false;
    }

    if (temp[0] == NEX_RET_CMD_FINISHED && temp[1] == 0xFF && temp[2] == 0xFF && temp[3] == 0xFF)
    {
        ret = true;
    }

    if (ret)
    {
        dbSerialPrintln("recvRetCommandFinished ok");
    }
    else
    {
        dbSerialPrintln("recvRetCommandFinished err");
        printError(temp);
    }

    return ret;
}

bool nexInit(void)
{
    bool ret1 = false;
    bool ret2 = false;

    dbSerialBegin(115200);
    // use the extended begin function
    nexSerial.begin(115200, SERIAL_8N1, 16, 17, false);
    delay(100);
    sendCommand("");
    sendCommand("bkcmd=1");
    ret1 = recvRetCommandFinished(100);
    sendCommand("page 0");
    ret2 = recvRetCommandFinished(100);
    return ret1 && ret2;
}

void nexLoop(NexTouch *nex_listen_list[])
{
    static uint8_t __buffer[10];

    uint16_t i;
    uint8_t c;

    while (nexSerial.available() > 0)
    {
        delay(10);
        c = nexSerial.read();

        if (NEX_RET_EVENT_TOUCH_HEAD == c)
        {
            if (nexSerial.available() >= 6)
            {
                __buffer[0] = c;
                for (i = 1; i < 7; i++)
                {
                    __buffer[i] = nexSerial.read();
                }
                __buffer[i] = 0x00;

                if (0xFF == __buffer[4] && 0xFF == __buffer[5] && 0xFF == __buffer[6])
                {
                    NexTouch::iterate(nex_listen_list, __buffer[1], __buffer[2], (int32_t)__buffer[3]);
                }
            }
        }
    }
}

/* 
*   Prints a discriptive error message
*/
void printError(uint8_t *errNr)
{
    switch (errNr[0])
    {
    case 0x00:
        dbSerial.println("Error : instruction sent by user has failed");
        break;
    case 0x01:
        dbSerial.println("Error : instruction sent by user has successful");
        break;
    case 0x02:
        dbSerial.println("Error : invalid Component ID or name was used");
        break;
    case 0x03:
        dbSerial.println("Error : invalid Page ID or name was used");
        break;
    case 0x04:
        dbSerial.println("Error : invalid Picture ID was used");
        break;
    case 0x05:
        dbSerial.println("Error : invalid Font ID was used");
        break;
    case 0x06:
        dbSerial.println("Error : file operation failed");
        break;
    case 0x09:
        dbSerial.println("Error : instructions with CRC validation fails their CRC check");
        break;
    case 0x11:
        dbSerial.println("Error : invalid Baud rate was used");
        break;
    case 0x12:
        dbSerial.println("Error : invalid Waveform ID or Channel # was used");
        break;
    case 0x1A:
        dbSerial.println("Error : invalid Variable name or invalid attribute was used");
        break;
    case 0x1B:
        dbSerial.println("Error : Operation of Variable is invalid. ie: Text assignment t0.txt=abc or\n"
                         " t0.txt=23, Numeric assignment j0.val='50″ or j0.val=abc");
        break;
    case 0x1C:
        dbSerial.println("Error : attribute assignment failed to assign");
        break;
    case 0x1D:
        dbSerial.println("Error : EEPROM Operation has failed");
        break;
    case 0x1E:
        dbSerial.println("Error : the number of instruction parameters is invalid");
        break;
    case 0x1F:
        dbSerial.println("Error : an IO operation has failed");
        break;
    case 0x20:
        dbSerial.println("Error : an unsupported escape character is used");
        break;
    case 0x23:
        dbSerial.println("Error : variable name is too long. Max length is 29 characters: 14 "
                         "for page + '.' + 14 for component.");
        break;
    case 0x70:
        dbSerial.print("Return value: ");
        for (int i = 1; i < sizeof(errNr); i++)
        {
            dbSerial.print(errNr[i]);
        }
        break;
    default:
        dbSerial.println("Error : Unknown failure: " + String(errNr[0], HEX));

        break;
    }
}