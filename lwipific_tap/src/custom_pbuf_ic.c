/**
 * Copyright (c) 2001-2004 Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 *
 * Author: Adam Dunkels <adam@sics.se>
 *
 */

/**
 * Copyright (c) 2024 Texas Instruments Incorporated
 *
 * Functions to handle Rx Custom Pbuf: icCustom_pbuf_free is called from LwIP stack hook.
 *
 */

#include "custom_pbuf_ic.h"
#include <../../intercore/intercore.h>

static inline void custom_pbuf_ic_init(Ic_CustomPbuf *cPbuf)
{
    cPbuf->orgBufLen       = 1536U;
    cPbuf->p.pbuf.len      = cPbuf->orgBufLen;
    cPbuf->p.pbuf.payload  = cPbuf->orgBufPtr;
    cPbuf->p.pbuf.next     = NULL;
}

/*!
 *  @b custom_pbuf_ic_free
 *  @n
 *  This function is called from pbuf_free for pbufs having custom flag set.
 *  This is designed to handle only single pbufs. This free function will enqueue the pbufs
 *  back to the free pbuf queue maintained within this driver. When a pbuf is needed from this pool,
 *  we can get it by dequeueing it from the free pbuf queue.
 *
 *  \param[in]  p
 *      the pbuf to be freed.
 *
 *  \retval
 *      NONE
 */
void custom_pbuf_ic_free(struct pbuf *p)
{
    Ic_CustomPbuf *cPbuf = (Ic_CustomPbuf*)p;
    Ic_Object_Handle hIcObj = (Ic_Object_Handle)cPbuf->customPbufArgs;

    custom_pbuf_ic_init(cPbuf);
    /* Enqueue the pbuf into freePbufInfoQ */

    pbufQ_ic_enQ(&hIcObj->freePbufQ, (struct pbuf *)cPbuf);
}
