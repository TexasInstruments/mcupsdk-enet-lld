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

/**
 *  \file enet_udma_defines.h
 *
 *  \brief Enet UDMA defined header file.
 */

#ifndef ENET_UDMA_DEFINES_H_
#define ENET_UDMA_DEFINES_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#if (ENET_SCICLIENT_AVAILABLE == 1)
#include <drivers/sciclient.h>
#endif


/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#if (ENET_SCICLIENT_AVAILABLE == 1)

#define ENET_UDMA_RING_MODE_RING                           TISCI_MSG_VALUE_RM_RING_MODE_RING        
#define ENET_UDMA_CH_PAUSE_ON_ERROR_DISABLED               TISCI_MSG_VALUE_RM_UDMAP_CH_PAUSE_ON_ERROR_DISABLED
#define ENET_UDMA_TX_CH_FILT_PSWORDS_DISABLED              TISCI_MSG_VALUE_RM_UDMAP_TX_CH_FILT_PSWORDS_DISABLED
#define ENET_UDMA_TX_CH_FILT_PSWORDS_ENABLED               TISCI_MSG_VALUE_RM_UDMAP_TX_CH_FILT_PSWORDS_ENABLED
#define ENET_UDMA_TX_CH_FILT_EINFO_DISABLED                TISCI_MSG_VALUE_RM_UDMAP_TX_CH_FILT_EINFO_DISABLED
#define ENET_UDMA_RX_FLOW_ERR_DROP                         TISCI_MSG_VALUE_RM_UDMAP_RX_FLOW_ERR_DROP
#define ENET_UDMA_RX_FLOW_PS_END_PD                        TISCI_MSG_VALUE_RM_UDMAP_RX_FLOW_PS_END_PD
#define ENET_UDMA_RX_FLOW_PSINFO_PRESENT                   TISCI_MSG_VALUE_RM_UDMAP_RX_FLOW_PSINFO_PRESENT
#define ENET_UDMA_RX_FLOW_EINFO_PRESENT                    TISCI_MSG_VALUE_RM_UDMAP_RX_FLOW_EINFO_PRESENT
#define ENET_UDMA_RX_FLOW_SRC_SELECT_SRC_TAG               TISCI_MSG_VALUE_RM_UDMAP_RX_FLOW_SRC_SELECT_SRC_TAG
#define ENET_UDMA_RX_FLOW_SIZE_THRESH_MAX                  TISCI_MSG_VALUE_RM_UDMAP_RX_FLOW_SIZE_THRESH_MAX
#define ENET_UDMA_RM_RING_MODE_MESSAGE                     TISCI_MSG_VALUE_RM_RING_MODE_MESSAGE 
#define ENET_UDMA_RM_MON_SRC_ELEM_CNT                      TISCI_MSG_VALUE_RM_MON_SRC_ELEM_CNT

#elif (ENET_SCICLIENT_AVAILABLE == 0)
#define ENET_UDMA_RING_MODE_RING                           (0x0u)        
#define ENET_UDMA_CH_PAUSE_ON_ERROR_DISABLED               (0u)
#define ENET_UDMA_TX_CH_FILT_PSWORDS_DISABLED              (0u)
#define ENET_UDMA_TX_CH_FILT_PSWORDS_ENABLED               (1u)
#define ENET_UDMA_TX_CH_FILT_EINFO_DISABLED                (0u)
#define ENET_UDMA_RX_FLOW_ERR_DROP                         (0u)
#define ENET_UDMA_RX_FLOW_PS_END_PD                        (0u)
#define ENET_UDMA_RX_FLOW_PSINFO_PRESENT                   (1u)
#define ENET_UDMA_RX_FLOW_EINFO_PRESENT                    (1u)
#define ENET_UDMA_RX_FLOW_SRC_SELECT_SRC_TAG               (4u)
#define ENET_UDMA_RX_FLOW_SIZE_THRESH_MAX                  (7u)
#define ENET_UDMA_RM_RING_MODE_MESSAGE                     (0x1u)
#define ENET_UDMA_RM_MON_SRC_ELEM_CNT                      (0u)

#else
#error "SCICLIENT availability params not supported"
#endif

#endif /* #ifndef ENET_UDMA_DEFINES_H_ */
