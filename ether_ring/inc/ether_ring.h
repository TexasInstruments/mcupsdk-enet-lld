/*
 *  Copyright (c) Texas Instruments Incorporated 2024
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

/*!
 * \file  ether_ring.h
 *
 * \brief This file contains the top-level API of the ether_ring driver.
 */

#ifndef ETHER_RING_H_
#define ETHER_RING_H_
/*!
 * \ingroup  ENET_ETHER_RING
 * \defgroup ETHER_RING CPSW
 *
 * The ETHER_RING module implements the module functions for
 * ENET_ETHER_RING API set.
 *
 * @{
 */
/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stddef.h>
#include <stdint.h>
#include <include/core/enet_queue.h>
#include <enet_dma.h>
#include <include/enet.h>
#include <include/enet_cfg.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */
/*! \brief Size of Ether-Ring Header */
#define ETHERRING_HEADER_SIZE                                           (4U)

/*! \brief Size of Lookup table for Duplicate packet rejection */
#define ETHERRING_LOOKUP_TABLE_SIZE                                     (256U*256U)

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */
/*!
 * \brief Config structure for Ether Ring
 *
 * The parameter structure for Ether Ring configuration
 */
typedef struct EtherRing_Cfg_s
{
    /*! Last Byte of Host Mac Address */
    uint8_t hostMacAddLastByte;

    /*! Is ether-ring configured */
    bool isCfg;

} EtherRing_Cfg;

/*!
 * \brief This structure stores the Ether Ring Stats
 *
 * The parameter structure for Ether Ring Stats
 */
typedef struct
{
    /*! EtherRing Lookup table for Duplicate packets */
    int8_t etherRingSeqLookUp[ETHERRING_LOOKUP_TABLE_SIZE];

    /*! Original packet count */
    uint64_t etherRingNonDuplicatedPktCount;

    /*! Duplicated packet count */
    uint64_t etherRingDuplicatedRxPacketCount;

    /*! Count of submitted packets via EtherRing */
    uint64_t etherRingSubmittedPacketCount;

}EtherRingStats;

/**
 * \brief
 *  Packet device information
 *
 * \details
 *  This structure caches the device info.
 */
typedef struct EtherRing_Obj_s
{
    /*! Enet Handle */
    Enet_Handle hEnet;

    /*! CPDMA Rx channel number for Ether-ring */
    uint32_t rxChNum;

    /*! CPDMA Rx channel handle for Ether-ring */
    EnetDma_RxChHandle hRxCh;

    /*! CPDMA TX channel number for Ether-ring */
    uint32_t txChNum;

    /*! CPDMA TX channel handle for Ether-ring */
    EnetDma_TxChHandle hTxCh;

    /*! Tells whether the Ether_ring handle is already allocated */
    bool isAllocated;

    /*! Sequence number of previous submitted packet */
    uint8_t prevSequenceNumber;

    /*! EtherRing Stats for duplicate packet rejection*/
    EtherRingStats etherRingStats;
}
EtherRing_Obj, *EtherRing_Handle;


typedef EnetQ EtherRing_pktQ;
/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
/*
 * Functions Provided by our translation layer code
 */
/*!
 * \brief Set default data path parameters.
 *
 * \param hEnet             [IN] Enet Handle
 * \param appCoreId         [IN] Application core Id
 * \param pEtherRingCfg     [IN] EtherRing configuration pointer.
 *
 * \retval Ether-ring handle if opened. Otherwise, NULL.
 *
 */
EtherRing_Handle EtherRing_open(Enet_Handle hEnet,
                                uint32_t appCoreId,
                                const void *pEtherRingCfg);

/*!
 * \brief Close EtherRing (data path).
 *
 * \param hEtherRing     [IN] Void pointer to the EtherRing handle
 *
 * \return \ref void
 */
void EtherRing_close(void *hEtherRing);

/*!
 * \brief Submit a queue of ready (full) packets to TX channel.
 *
 * \param hEtherRing  [IN] Void pointer to the Ether-ring handle
 * \param pSubmitQ    [IN] Pointer to #EtherRing_pktQ structure containing packets
 *                        to be submitted.
 *
 * \return \ref Enet_ErrorCodes
 */
int32_t EtherRing_submitTxPktQ(void *hEtherRing,
                               EtherRing_pktQ *pSubmitQ);

/*!
 * \brief Retrieve queue of TX free (empty) packets from TX channel.
 *
 * \param hEtherRing  [IN] Void pointer to the Ether-ring handle
 * \param pRetrieveQ  [OUT] Pointer to #EtherRing_pktQ structure where
 *                        packets from hardware are retrieved and returned to
 *                        application
 *
 * \return \ref Enet_ErrorCodes
 */
int32_t EtherRing_retrieveTxPktQ(void *hEtherRing,
                                 EtherRing_pktQ *pRetrieveQ);

/*!
 * \brief Submit queue of RX free (empty) packets for reception to RX channel.
 *
 * \param hEtherRing  [IN] Void pointer to the Ether-ring handle
 * \param pSubmitQ    [IN] Pointer to #EtherRing_pktQ structure containing packets
 *                         to be submitted to channel
 *                    [OUT] Returned packets for which driver couldn't allocate the DMA
 *                          descriptors.
 *
 * \return \ref Enet_ErrorCodes
 */
int32_t EtherRing_submitRxPktQ(void *hEtherRing,
                               EtherRing_pktQ *pSubmitQ);

/*!
 * \brief Retrieve queue of RX ready (full) packets from RX channel.
 *
 * \param hEtherRing  [IN] Void pointer to the Ether-ring handle
 * \param pRetrieveQ  [OUT] Pointer to #EtherRing_pktQ structure where
 *                          packets from hardware are retrieved and returned to
 *                          application
 *
 *  \return \ref Enet_ErrorCodes
 */
int32_t EtherRing_retrieveRxPktQ(void *hEtherRing,
                                 EtherRing_pktQ *pRetrieveQ);

/*!
 * \brief Attaches the Tx DMA channel given by application to Ether-ring Middleware .
 *
 * \param hEtherRing  [IN] void Pointer to Ether-ring handle
 * \param hTxCh       [IN] EtherRing TX channel handle for the TX channel we
 *                         wish to submit packets
 *
 * \param txChNum     [IN] Tx Channel Number
 *
 *  \return \ref void
 */
void EtherRing_attachTxDmaHandle(void *hEtherRing,
                                EnetDma_TxChHandle hTxCh,
                                int32_t txChNum);

/*!
 * \brief Attaches the RX DMA channel given by application to Ether-ring Middleware .
 *
 * \param hEtherRing  [IN] void Pointer to Ether-ring handle
 * \param hRxCh       [IN] EtherRing RX channel handle for the RX channel we
 *                         wish to retrieve packets
 *
 * \param rxChNum     [IN] RX Channel Number
 *
 *  \return \ref void
 */
void EtherRing_attachRxDmaHandle(void *hEtherRing,
                                EnetDma_RxChHandle hRxCh,
                                int32_t rxChNum);

/*!
 * \brief Periodic function that needs to be called from application.
 *
 * \param hEtherRing  [IN] Void pointer to the Ether-ring handle
 *
 *  \return \ref void
 */
void EtherRing_periodicTick(void *hEtherRing);


#ifdef __cplusplus
}
#endif

#endif /* ETHER_RING_H_ */

/*! @} */
