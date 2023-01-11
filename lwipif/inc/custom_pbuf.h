/*
 * Copyright (C) 2023 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*!
 * \file  custom_pbuf.h
 *
 * \brief Rx Custom Pbuf for LwIP.
 */

#include <stdint.h>
#include <enet.h>
#include <pbufQ.h>

void custom_pbuf_free(struct pbuf *p);

typedef void * Rx_CustomPbuf_Args;

typedef struct Rx_CustomPbuf_s
{
    /*! Custom pbuf structure. Keep this as the first member as we dereference pbuf as custom pbuf. */
    struct pbuf_custom p;

    /*! next points to the next custom pbuf in the pbuf chain in a circular fashion,
     *  unlike pbuf->next, this never equals NULL when the pbuf is in use by the stack.
     *  If the pbuf chain contains only one pbuf then it points to itself. */
    struct Rx_CustomPbuf_s *next;

    /*! alivePbufCount stores the number of pbufs in the pbuf chain that are currently in use by the stack.
     *  This value should be same for all the pbufs in a pbuf chain and decrements when pbuf_free
     *  is called on any pbuf in the pbuf chain. This equals zero only when pbuf_free is called on
     *  every pbuf of the chain. */
    uint32_t alivePbufCount;

    /*! customPbufArgs points to the Rx handle having all the Queues */
    Rx_CustomPbuf_Args customPbufArgs;

    /*! Original Buffer ptr of the pbuf->payload. Store this as the LwIP stack shifts the payload as needed. */
    uint8_t *orgBufPtr;

    /*! Original Buffer allocated length */
    uint32_t orgBufLen;
} Rx_CustomPbuf;
