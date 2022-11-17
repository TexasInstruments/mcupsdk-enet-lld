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
 *  \file enet_dma_pktutils.h
 *
 *  \brief Enet DMA utility API to check packet and descriptor ownership states
 *         during development and debug.
 */

/*!
 * \ingroup  ENET_DMA_API
 * \defgroup ENET_PKTUTILS_API Enet packet utils API and data structures.
 *
 * @{
 */
#ifndef ENET_DMA_PKTUTILS_H_
#define ENET_DMA_PKTUTILS_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdarg.h>
#include <include/core/enet_utils.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define ENET_UTILS_GET_STATE(state, mask, bitshift)    ((*(state) & (mask)) >> (bitshift))
#define ENET_UTILS_SET_STATE(state, mask, bitshift, value) \
    ({                                                     \
         *(state) &= ~(mask);                              \
         *(state) |= ((value) << (bitshift));              \
     })

#define ENET_UTILS_DRIVER_STATE_MASK            (0x000000FFU)
#define ENET_UTILS_DRIVER_STATE_BIT_SHIFT       (0U)

#define ENET_DMA_MEMMGR_STATE_MASK          (0x0000FF00U)
#define ENET_DMA_MEMMGR_STATE_BIT_SHIFT     (8U)

#define ENET_UTILS_APP_STATE_MASK               (0x00FF0000U)
#define ENET_UTILS_APP_STATE_BIT_SHIFT          (16U)

#define ENET_UTILS_GET_PKT_DRIVER_STATE(state)          \
    (ENET_UTILS_GET_STATE(state,                        \
                          ENET_UTILS_DRIVER_STATE_MASK, \
                          ENET_UTILS_DRIVER_STATE_BIT_SHIFT))

#define ENET_UTILS_SET_PKT_DRIVER_STATE(state, value)        \
    (ENET_UTILS_SET_STATE(state,                             \
                          ENET_UTILS_DRIVER_STATE_MASK,      \
                          ENET_UTILS_DRIVER_STATE_BIT_SHIFT, \
                          value))

#define ENET_UTILS_GET_PKT_MEMMGR_STATE(state)          \
    (ENET_UTILS_GET_STATE(state,                          \
                          ENET_DMA_MEMMGR_STATE_MASK, \
                          ENET_DMA_MEMMGR_STATE_BIT_SHIFT))

#define ENET_UTILS_SET_PKT_MEMMGR_STATE(state, value)        \
    (ENET_UTILS_SET_STATE(state,                               \
                          ENET_DMA_MEMMGR_STATE_MASK,      \
                          ENET_DMA_MEMMGR_STATE_BIT_SHIFT, \
                          value))

#define ENET_UTILS_GET_PKT_APP_STATE(state)          \
    (ENET_UTILS_GET_STATE(state,                     \
                          ENET_UTILS_APP_STATE_MASK, \
                          ENET_UTILS_APP_STATE_BIT_SHIFT))

#define ENET_UTILS_SET_PKT_APP_STATE(state, value)        \
    (ENET_UTILS_SET_STATE(state,                          \
                          ENET_UTILS_APP_STATE_MASK,      \
                          ENET_UTILS_APP_STATE_BIT_SHIFT, \
                          value))

#define ENET_DMA_MEMMGR_DESCSTATE_MASK                      (0x000000FFU)
#define ENET_DMA_MEMMGR_DESCSTATE_SHIFT                     (0U)

#define ENET_UTILS_GET_DESC_MEMMGR_STATE(state)             \
    (ENET_UTILS_GET_STATE(state,                              \
                          ENET_DMA_MEMMGR_DESCSTATE_MASK, \
                          ENET_DMA_MEMMGR_DESCSTATE_SHIFT))

#define ENET_UTILS_SET_DESC_MEMMGR_STATE(state, value)       \
    (ENET_UTILS_SET_STATE(state,                               \
                          ENET_DMA_MEMMGR_DESCSTATE_MASK,  \
                          ENET_DMA_MEMMGR_DESCSTATE_SHIFT, \
                          value))

/*!
 * \brief Enet DMA Descriptor state.
 *
 * DMA descriptor state used by Enet DMA driver for sanity check.
 */
typedef enum EnetDma_DescStateMemMgr_e
{
    /*! Desc free */
    ENET_DESCSTATE_MEMMGR_FREE = 0U,

    /*! Desc Allocated */
    ENET_DESCSTATE_MEMMGR_ALLOC
} EnetDma_DescStateMemMgr;

/*!
 * \brief App Pkt state.
 *
 * Packet state for application
 */
typedef enum EnetDma_PktStateApp_e
{
    /*! Packet with APP Free queue*/
    ENET_PKTSTATE_APP_WITH_FREEQ = 0U,

    /*! Packet with APP Ready queue*/
    ENET_PKTSTATE_APP_WITH_READYQ,

    /*! Packet with Driver*/
    ENET_PKTSTATE_APP_WITH_DRIVER
} EnetDma_PktStateApp;

/*!
 * \brief DMA Pkt state.
 *
 * Packet state for DMA
 */
typedef enum EnetDma_PktStateDma_e
{
    /*! Packet with HW  */
    ENET_PKTSTATE_DMA_WITH_HW = 0U,

    /*! Packet not with HW  */
    ENET_PKTSTATE_DMA_NOT_WITH_HW
} EnetDma_PktStateDma;

/*!
 * \brief Enet DMA Pkt state.
 *
 * Packet state for memory manager
 */
typedef enum EnetDma_PktStateMemMgr_e
{
    /*! Packet free */
    ENET_PKTSTATE_MEMMGR_FREE = 0U,

    /*! Packet Allocated */
    ENET_PKTSTATE_MEMMGR_ALLOC,
} EnetDma_PktStateMemMgr;

/*!
 * \brief Enet DMA Pkt state.
 *
 * Type of Enet DMA Pkt state.
 */
typedef enum EnetDma_PktStateModuleType_e
{
    /*! Module type is app */
    ENET_PKTSTATE_MODULE_APP = 0U,

    /*! Module type is DMA Driver  */
    ENET_PKTSTATE_MODULE_DRIVER,

    /*! Module type is MEM Mgr */
    ENET_PKTSTATE_MODULE_MEMMGR,
} EnetDma_PktStateModuleType;

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/*!
 * \brief Packet state check & set API for app, memory manager and dma driver.
 *
 * This development time API checks expected state and sets new packet state. This
 * is used by modules handling the packet to confirm the packet is returned and allocated
 * in correct state. Memutils checks (free, allocated), dma driver checks (with Hw,
 * with driver) and app checks which queue it belongs
 *
 *  \param pStateVar     [IN/OUT] Pointer to packet state variable of packet structure
 *                       #EnetDma_Pkt
 *  \param module        [IN] Module id. Refer to #EnetDma_PktStateModuleType
 *  \param expectedState [IN] State packet is expected to be.
 *  \param newState      [IN] New state packet is set to.
 *
 * \note - Optional for MemMgr and app. DMA driver mandatory uses when ENET_CFG_DEV_ERROR
 *         is defined.
 *
 */
static inline void EnetDma_checkPktState(uint32_t *pStateVar,
                             EnetDma_PktStateModuleType module,
                             uint32_t expectedState,
                             uint32_t newState);

/*!
 * \brief Descriptor state check & set API for descriptor memory manager.
 *
 * This development time API checks expected state and sets new descriptor state. This
 * is used by modules handling the Desc to confirm the Desc is returned and allocated
 * in correct state. Only memutils modules uses it now for checking if desc is free or
 * allocated while allocating or freeing desc respectively.
 *
 *  \param pStateVar     [IN/OUT] Pointer to Desc state variable.
 *  \param expectedState [IN] State Desc is expected to be.
 *  \param newState      [IN] New state Desc is set to.
 *
 * \note - memory manager may not allocate the descState as this is optional check for
 *        MemMgr.
 */
static inline void EnetDma_checkDescState(uint32_t *pStateVar,
                              EnetDma_DescStateMemMgr expectedState,
                              EnetDma_DescStateMemMgr newState);

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

static inline void EnetDma_checkDescState(uint32_t *pStateVar,
                              EnetDma_DescStateMemMgr expectedState,
                              EnetDma_DescStateMemMgr newState)
{
    Enet_assert(ENET_UTILS_GET_DESC_MEMMGR_STATE(pStateVar) == expectedState);
    ENET_UTILS_SET_DESC_MEMMGR_STATE(pStateVar, newState);
}

static inline void EnetDma_checkPktState(uint32_t *pStateVar,
                             EnetDma_PktStateModuleType module,
                             uint32_t expectedState,
                             uint32_t newState)
{
    switch (module)
    {
        case ENET_PKTSTATE_MODULE_APP:
            Enet_assert(ENET_UTILS_GET_PKT_APP_STATE(pStateVar) == expectedState);
            ENET_UTILS_SET_PKT_APP_STATE(pStateVar, newState);
            break;

        case ENET_PKTSTATE_MODULE_DRIVER:
            Enet_assert(ENET_UTILS_GET_PKT_DRIVER_STATE(pStateVar) == expectedState);
            ENET_UTILS_SET_PKT_DRIVER_STATE(pStateVar, newState);
            break;

        case ENET_PKTSTATE_MODULE_MEMMGR:
            Enet_assert(ENET_UTILS_GET_PKT_MEMMGR_STATE(pStateVar) == expectedState);
            ENET_UTILS_SET_PKT_MEMMGR_STATE(pStateVar, newState);
            break;

        default:
            break;
    }
}

#ifdef __cplusplus
}
#endif

#endif /* #ifndef ENET_DMA_PKTUTILS_H_ */

/*!
 *  @}
 */
