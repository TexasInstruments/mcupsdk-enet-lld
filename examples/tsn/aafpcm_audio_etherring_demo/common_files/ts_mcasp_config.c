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

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "drivers/hw_include/cslr.h"
#include <kernel/dpl/AddrTranslateP.h>
#include <drivers/mcasp.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static void TsMcasp_selAuxClk(uint8_t sysAux, uint8_t clkSel, int instIdx);

static void TsMcasp_writeMMR(const uint32_t addr, uint32_t val);

/* ========================================================================== */
/*                           Global Variables                                 */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t TsMcasp_tsMcaspConfig(int instIdx)
{
    int status = SystemP_SUCCESS;

    /* Open MCASP driver after enabling the HCLK */
    gMcaspHandle[instIdx] = MCASP_open(instIdx, &gMcaspOpenParams[instIdx]);
    if(NULL == gMcaspHandle[instIdx])
    {
        DebugP_logError("MCASP open failed for instance %d !!!\r\n", instIdx);
        DebugP_assert(false);
    }

    MCASP_startTransferTx(gMcaspHandle[instIdx]);

    return status;
}

void TsMcasp_TxCb(MCASP_Handle handle,
                          MCASP_Transaction *transaction)
{
    /* Empty Tx Callback. */
}