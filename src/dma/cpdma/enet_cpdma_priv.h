/*
 *  Copyright (c) Texas Instruments Incorporated 2020
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
 *  \file enet_cpdma_priv.h
 *
 *  \brief ENT CPDMA private header file.
 */

#ifndef ENET_CPDMA_PRIV_H_
#define ENET_CPDMA_PRIV_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <string.h>
#include <csl_cpswitch.h>
#include <include/core/enet_dma.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/*! \brief Ethernet Jumbo frame size */
#define ENET_CPDMA_JUMBO_PACKET_SIZE    (9000U)

/*! Maximum number of channels supported by the DMA */
#define ENET_CPDMA_MAX_CHANNELS         (8U)


/* CPDMA teardown acknowledgement value */
#define ENET_CPDMA_TEARDOWN_DESC        ((uint32_t)0xFFFFFFFCU)

/* CPDMA channel override feature to override the default identity based mapping of packet priority to switch priority */
#define ENET_CPDMA_CHANNEL_OVERRIDE      (ENET_BIT(0U))

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/*!
 * \brief FHOST Checksum encapsulated word
 *
 */
typedef uint32_t EnetCpdma_ChecksumEncapWord_s;

/*!
 * \brief CPPI3.0 Buffer descriptor format.
 *
 *  The following is the format of a single buffer descriptor
 *  on the EMAC.
 */
typedef struct EnetCpdma_cppiDesc_s
{
    /*!
    * \brief        Pointer to next descriptor in chain.
    */
    volatile struct EnetCpdma_cppiDesc_s *pNext  __attribute__ ((aligned(ENETDMA_CACHELINE_ALIGNMENT)));
    /*!
    * \brief        Pointer to data buffer.
    */
    uint8_t         *pBuffer;
    /*!
    * \brief        Buffer Offset(MSW) and Length(LSW).
    */
    uint32_t        bufOffLen;
    /*!
    * \brief        Packet Flags(MSW) and Length(LSW).
    */
    uint32_t        pktFlgLen;
    /*
     * The below fields are not used by CPDMA HW. These are stored here for SW purpose.
     */
    /*!
    * \brief        Fhost Chksum Encap Info.
    */
    EnetCpdma_ChecksumEncapWord_s chkSumInfo  __attribute__ ((aligned(8)));
    /*!
    * \brief        Original buffer allocated length.
    */
    uint16_t        orgBufLen;
} __attribute__((packed)) EnetCpdma_cppiDesc;

#define ENET_CPDMA_DESC_PKT_FLAG_SOP                  (0x80000000U)
#define ENET_CPDMA_DESC_PKT_FLAG_EOP                  (0x40000000U)
#define ENET_CPDMA_DESC_PKT_FLAG_OWNER                (0x20000000U)
#define ENET_CPDMA_DESC_PKT_FLAG_EOQ                  (0x10000000U)

#define ENET_CPDMA_DESC_PSINFO_TDOWNCMPLT_FLAG        (0x08000000U)
#define ENET_CPDMA_DESC_PSINFO_PASSCRC_FLAG           (0x04000000U)
#define ENET_CPDMA_DESC_PSINFO_RX_LONG_FLAG           (0x02000000U)
#define ENET_CPDMA_DESC_PSINFO_RX_SHORT_FLAG          (0x01000000U)
#define ENET_CPDMA_DESC_PSINFO_RX_MAC_CTL_FLAG        (0x00800000U)
#define ENET_CPDMA_DESC_PSINFO_RX_OVERRUN_FLAG        (0x00400000U)
#define ENET_CPDMA_DESC_PSINFO_RX_PKT_ERR_MASK        (0x00300000U)
#define ENET_CPDMA_DESC_PSINFO_RX_VLAN_ENCAP_MASK     (0x00080000U)
#define ENET_CPDMA_DESC_PKT_FLAG_CHKSUM_ENCAP         (0x00004000U)
#define ENET_CPDMA_DESC_PSINFO_RX_CHECKSUM_INFO_MASK  (0x00001000U)
#define ENET_CPDMA_DESC_PSINFO_RX_PACKET_LEN_MASK     (0x00000FFFU)

/* Macros to get from port field from packet descriptor */
#define ENET_CPDMA_DESC_PSINFO_FROM_PORT_MASK         (0x00070000U)
#define ENET_CPDMA_DESC_PSINFO_FROM_PORT_SHIFT        (16U)

/* Macros to set and get to port field from packet descriptor */
#define ENET_CPDMA_DESC_PSINFO_TO_PORT_MASK           (0x00030000U)
#define ENET_CPDMA_DESC_PSINFO_TO_PORT_SHIFT          (16U)

/* Macros to set to port enable field in packet descriptor */
#define ENET_CPDMA_DESC_PSINFO_TO_PORT_ENABLE_MASK    (0x00100000U)

#define ENET_CPDMA_DESC_PSINFO_TO_PORT_ENABLE_SHIFT   (20U)

/* CPSW Checksum offload Encap Info length */
#define ENET_CPDMA_ENCAPINFO_CHECKSUM_INFO_LEN        (sizeof(EnetCpdma_ChecksumEncapWord_s))

/* Core Id for cpdma socs with only single core support */
#define ENET_CPDMA_DEFAULT_SINGLE_CORE_ID             (0U)

/*!
 * \brief DMA transfer direction (rx/tx) enum
 */
typedef enum EnetCpdma_Dir_s
{
    /*! DMA RX flow  */
    ENET_CPDMA_DIR_RX = 0,

    /*! DMA Tx channel   */
    ENET_CPDMA_DIR_TX = 1,

    /*! DMA Number of directions */
    ENET_CPDMA_DIR_MAX = 2
} EnetCpdma_Dir;

/**
 *  \brief Channel Info structure
 *
 */
typedef struct EnetCpdma_ChInfo_s
{
    /*! Channel Number */
    uint32_t          chNum;

    /*! Channel Direction */
    uint32_t          chDir;

    /*! Maximum Number of Buffer Descriptors allocated for this channel */
    uint32_t          numBD;

    /*! Buffer size */
    uint32_t          bufSize;

    /*! CPSW DMA event callback function - this function will be called when
     *   the registered packets are received on RX Flow*/
    EnetDma_PktNotifyCb notifyCb;

    /*! Argument to be used for the callback routines (it should mean something
     *  to layer into which the callback calls) */
    void *hCbArg;
} EnetCpdma_ChInfo;

typedef struct EnetCpdma_TxChObj_s
{
    /*! Channel init params */
    EnetCpdma_OpenTxChPrms txChPrms;

    /*! Enet Dma handle*/
    EnetDma_Handle hEnetDma;

    /*! Desc Chan Info */
    EnetCpdma_ChInfo chInfo;

    /*! Channel initialization state */
    bool initFlag;

    /*! App private */
    void *appPriv;

    /*! Tx channel app packet and DMA desc stats */
    EnetDma_TxChStats stats;
} EnetCpdma_TxChObj;

typedef struct EnetCpdma_RxChObj_s
{
    /*! Channel init params */
    EnetCpdma_OpenRxChPrms rxChPrms;

    /*! CPSW Dma handle*/
    EnetDma_Handle hEnetDma;

    /*! Desc Chan Info */
    EnetCpdma_ChInfo chInfo;

    /*! Channel initialization state */
    bool initFlag;

    /*! App private */
    void *appPriv;
    /*! Rx channel app packet and DMA desc stats */
    EnetDma_RxChStats stats;
} EnetCpdma_RxChObj;

typedef struct EnetCpdma_DescHistoryEntry_s {
    volatile EnetCpdma_cppiDesc *listBegin;
    volatile EnetCpdma_cppiDesc *listEnd;
    EnetCpdma_cppiDesc *cpDesc;
    uint32_t unackedCpDescCount;
    volatile EnetCpdma_cppiDesc *unackedCpDescFirst;
} EnetCpdma_DescHistoryEntry;

#define ENETCPDMA_MAX_DESC_HISTORY (16U)

typedef struct EnetCpdma_DescHistory_s {
    EnetCpdma_DescHistoryEntry descEntry[ENETCPDMA_MAX_DESC_HISTORY];
    uint32_t descHistoryCount;
} EnetCpdma_DescHistory;

/**
 *  \brief Transmit/Receive Descriptor Channel Structure
 *
 *  (One receive and up to 8 transmit in this example)
 */
typedef struct EnetCpdma_DescCh_s {
    /*! Pointer to parent structure */
    EnetDma_Handle hEnetDma;
    /*! Packets queued as desc */
    EnetDma_PktQ  descQueue;
    /*! Packets waiting for TX desc */
    EnetDma_PktQ  waitQueue;
    /*! Packets waiting for TX desc */
    EnetDma_PktQ  freeQueue;
    /*! Channel info */
    EnetCpdma_ChInfo   *chInfo;
    /*! Max number of desc (buffs) */
    uint32_t          descMax;
    /*! Current number of desc */
    uint32_t          descFreeCount;
    /*! Current number of desc with H/W */
    uint32_t          descSubmittedCount;
    /*! First desc location */
    EnetCpdma_cppiDesc *pDescFirst;
    /*! Last desc location */
    EnetCpdma_cppiDesc *pDescLast;
    /*! Location to read next desc */
    EnetCpdma_cppiDesc *pDescRead;
    /*! Location to write next desc */
    EnetCpdma_cppiDesc *pDescWrite;
    /*! Location to write next desc */
    volatile EnetCpdma_cppiDesc *pDescTail;
    EnetCpdma_DescHistory descHistory;
    uint32_t unackedCpDescCount;
    volatile EnetCpdma_cppiDesc *unackedCpDescFirst;
} EnetCpdma_DescCh;


/**
 *  \brief DMA module
 *
 *  This is an internal/private driver structure and should not be used
 *  or modified by the caller.
 */
typedef struct EnetCpdma_DrvObj_s
{
    /*! CPSW NAVSS instance */
    Enet_Type enetType;

    /*! CPSW susbsystem register overlay */
    CSL_Xge_cpsw_ss_sRegs *cpswSsRegs;

    /*! CPDMA register overlay */
    CSL_CpdmaRegs *cpdmaRegs;

    /*! CPPI descriptor memory */
    void*   cppiRamBase;

    /*! total size of the reserved CPPI descriptor area in bytes */
    uint32_t    descSize;

    /*! Maximum number of CPPI descriptors allocated */
    uint32_t maxBds;

    /*! Number of CPPI descriptors allocated so far */
    uint32_t numBdsAllocated;

    /*! Num of Tx channels available */
    uint32_t numTxChans;

    /*! Num of Rx channels available */
    uint32_t numRxChans;

    /*! Module initialization state */
    bool initFlag;

    /*! Tx Control struct ptrs */
    EnetCpdma_DescCh  txCppi[ENET_CPDMA_MAX_CHANNELS];
    /*! Rx Control struct ptrs */
    EnetCpdma_DescCh  rxCppi[ENET_CPDMA_MAX_CHANNELS];
    /*! channel initialized ? */
    bool            chIsInit[ENET_CPDMA_DIR_MAX][ENET_CPDMA_MAX_CHANNELS];
    /*! channel open ? */
    bool            chIsOpen[ENET_CPDMA_DIR_MAX][ENET_CPDMA_MAX_CHANNELS];
    /*! teardown pending ? */
    bool            tdPending[ENET_CPDMA_DIR_MAX][ENET_CPDMA_MAX_CHANNELS];
    /*! TX  Completion Threshold count */
    int32_t         txIntThreshold[ENET_CPDMA_MAX_CHANNELS];
    /*! State of each port */
    uint32_t        portState;

    /* Status related fields */
    /*! Either NO_ERROR or combination of error bits from above status codes */
    uint32_t        hwStatus;
    /*! If error, contains DMASTATUS register contents                 */
    uint32_t        hwErrInfo;
	/*! Peripheral features */
    uint32_t        features;
    /*! coreId expected by cpdma module */
    uint32_t        cpdmaCoreId;
    /*! RX buffer Offset */
    uint32_t        rxBufOffset;
    /*! RX Interrupt per Msec */
    uint32_t        rxInterruptPerMSec;
    /*! TX Interrupt per Msec */
    uint32_t        txInterruptPerMSec;
    /* Enable channel override flag */
    uint32_t        enChOverrideFlag;
    /* Reset ongoing flag */
    uint32_t        isResetOngoing;
} EnetCpdma_DrvObj;

/* ========================================================================== */
/*                         Global Variables                                   */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
#if 0

/**
 *  \brief CPSW DMA Open function.
 *
 *  This API opens the CPSW DMA RX channel and sets data path module configuration.
 *  This function should be called only by Master core and should be called before
 *  calling any IOCTLs.
 *
 *  \param hEnetDma  [IN] CPSW DMA module handle.
 *  \param rxChInitPrms  [IN] CPSW RX channel init parameters.
 *  \param totalRxFlowCount [IN] Total number of Rx flows to be allocated to the
 *                               channel
 *
 *  \return \ref Enet_ErrorCodes
 */
int32_t EnetUdma_openRxCh(EnetDma_Handle hEnetDma,
                         const EnetUdma_RxChInitPrms *pRxChInitPrms,
                         uint32_t totalRxFlowCount);

/**
 *  \brief CPSW DMA close function.
 *
 *  Stop and close the Rx channel and set deinit DMA module configuration.
 *
 *  \param hRxCh  [IN] Handle to CPSW DMA RX channel returned from call to
 *                   EnetUdma_open.
 *
 *  \return \ref Enet_ErrorCodes
 */
int32_t EnetUdma_closeRxCh(EnetDma_Handle hEnetDma);

#endif

/*! Rx flow object allocation function  */
EnetCpdma_RxChObj *EnetCpdma_memMgrAllocRxChObj(void);

/*! Rx flow object free function  */
void EnetCpdma_memMgrFreeRxChObj(EnetCpdma_RxChObj *pRxChObj);

/*! TX channel object allocation function  */
EnetCpdma_TxChObj *EnetCpdma_memMgrAllocTxChObj(void);

/*! TX channel object free function  */
void EnetCpdma_memMgrFreeTxChObj(EnetCpdma_TxChObj *pTxChObj);

/*! Drv object memory manager init  */
void EnetCpdma_memMgrInit(void);

/*! Drv object memory manager deinit function  */
void EnetCpdma_memMgrDeInit(void);

void EnetCpdmaStats_updateNotifyStats(EnetDma_CbStats *pktStats,
                                      uint32_t pktCnt,
                                      uint32_t timeDiff);

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

static inline void EnetCpdmaStats_addCnt(uint64_t *statCnt,
                                         uint64_t addCnt)
{
#if defined(ENETDMA_INSTRUMENTATION_ENABLED)
    *statCnt += addCnt;
#endif
}

#ifdef __cplusplus
}
#endif

#endif /* #ifndef ENET_CPDMA_PRIV_H_ */
