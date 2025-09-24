/*
 *  Copyright (c) Texas Instruments Incorporated 2025
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <enet_apputils.h>
#include <kernel/dpl/ClockP.h>

#include "udp_test.h"
#include "static_udp.h"

#define PAY_LOAD_LEN   (64U)
#define NUM_PACKETS    (100U)

uint8_t gPayload[PAY_LOAD_LEN];

const EnetUDP_AddrInfo enetAppAddrInfo =
{
    .srcMac = {0x70, 0xff,0x76,0x1d,0xec,0xf2},
    .dstMac = {0x7c,0xc2,0xc6,0x54,0x57,0x6c},
    .srcIP  = {0xC0,0xA8,0x01,0x14}, /* 192.168.1.20 */
    .dstIP  = {0xC0,0xA8,0x01,0x15}, /* 192.168.1.21 */
    .srcPortUDP = 4096,
    .dstPortUDP = 4096,
};

int32_t EnetApp_rawUdpTest(void)
{
    int32_t status;
    int32_t sendStatus = 0;
    uint8_t header[COMBINED_HEADER_SIZE] = {0};

    status = EnetApp_initApp();

    if(status != ENET_SOK)
    {
        EnetAppUtils_print("Failed to init CPSW: %d\r\n", status);
        return status;
    }
     
    /* Header is prepared  once and will be used for all the packets */
    EnetApp_createCompleteHeader(&header[0],PAY_LOAD_LEN,&enetAppAddrInfo);
    
    for(int i=0;i<NUM_PACKETS;i++)
    {
        /* filling payload with Random Data */
        int8_t r = rand();
        for(int j=0;j<PAY_LOAD_LEN;j++){
            gPayload[j] = r;
        }

        sendStatus = EnetApp_sendUDP(&header[0], gPayload, PAY_LOAD_LEN);

        if(sendStatus==0)
        {
            EnetAppUtils_print("Packet %d sent successfully\r\n",i+1);
        }
        else 
        {
            EnetAppUtils_print("Failed to send packet %d\r\n",i+1);
        }        
        ClockP_usleep(20000);
    }

    if(sendStatus==0)
    {
        EnetAppUtils_print("All packets sent successfully\r\n");
    }
    else
    {
        EnetAppUtils_print("Failed to send all packets\r\n");
    }

    status = EnetApp_deinitApp();

    if(status != ENET_SOK)
    {
        EnetAppUtils_print("Failed to deinit CPSW: %d\r\n", status);
        return status;
    }

    return status;
}
