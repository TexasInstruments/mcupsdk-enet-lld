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

/*!
 * \file  enet_types.h
 *
 * \brief This file contains the basic types using across the Enet driver.
 */

/*!
 * \addtogroup ENET_MAIN_API
 *
 * @{
 */

#ifndef ENET_TYPES_H_
#define ENET_TYPES_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <stdbool.h>
#include <drivers/hw_include/csl_types.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */

/*!
 * \anchor Enet_ErrorCodes
 * \name Enet Error Codes
 *
 * Error codes returned by the Enet driver APIs.
 *
 * @{
 */

/* Enet driver error codes are same as CSL's to maintain consistency */

/*! \brief Success. */
#define ENET_SOK                              (CSL_PASS)

/*! \brief Operation in progress. */
#define ENET_SINPROGRESS                      (1)

/*! \brief Generic failure error condition (typically caused by hardware). */
#define ENET_EFAIL                            (CSL_EFAIL)

/*! \brief Bad arguments (i.e. NULL pointer). */
#define ENET_EBADARGS                         (CSL_EBADARGS)

/*! \brief Invalid parameters (i.e. value out-of-range). */
#define ENET_EINVALIDPARAMS                   (CSL_EINVALID_PARAMS)

/*! \brief Time out while waiting for a given condition to happen. */
#define ENET_ETIMEOUT                         (CSL_ETIMEOUT)

/*! \brief Allocation failure. */
#define ENET_EALLOC                           (CSL_EALLOC)

/*! \brief Unexpected condition occurred (sometimes unrecoverable). */
#define ENET_EUNEXPECTED                      (CSL_EALLOC - 1)

/*! \brief The resource is currently busy performing an operation. */
#define ENET_EBUSY                            (CSL_EALLOC - 2)

/*! \brief Already open error. */
#define ENET_EALREADYOPEN                     (CSL_EALLOC - 3)

/*! \brief Operation not permitted. */
#define ENET_EPERM                            (CSL_EALLOC - 4)

/*! \brief Operation not supported. */
#define ENET_ENOTSUPPORTED                    (CSL_EALLOC - 5)

/*! \brief Resource not found. */
#define ENET_ENOTFOUND                        (CSL_EALLOC - 6)

/*! \brief Unknown IOCTL. */
#define ENET_EUNKNOWNIOCTL                    (CSL_EALLOC - 7)

/*! \brief Malformed IOCTL (args pointer or size not as expected). */
#define ENET_EMALFORMEDIOCTL                  (CSL_EALLOC - 8)

/*! @} */

/*! \brief Macro to get the size of an array. */
#define ENET_ARRAYSIZE(x)                     (sizeof(x) / sizeof(x[0]))

/*! \brief Macro to set bit at given bit position. */
#define ENET_BIT(n)                           (1U << (n))

/*! \brief Macro to get bit at given bit position. */
#define ENET_GET_BIT(val, n)                  (((val) & ENET_BIT(n)) >> (n))

/*! \brief Macro to check if bit at given bit position is set. */
#define ENET_IS_BIT_SET(val, n)               (((val) & ENET_BIT(n)) != 0U)

/*! \brief Macro to check if value is not zero. */
#define ENET_NOT_ZERO(val)                    ((uint32_t)0U != (uint32_t)(val))

/** \brief Field EXTract macro. We can't use CSL macro as it appends shift/mask with CSL_*/
#define ENET_FEXT(reg, PER_REG_FIELD)                                        \
    (((reg) & PER_REG_FIELD##_MASK) >> PER_REG_FIELD##_SHIFT)

/** \brief Field INSert macro. We can't use CSL macro as it appends shift/mask with CSL_*/
#define ENET_FINS(reg, PER_REG_FIELD, val)                                   \
    ((reg) = ((reg) & ~PER_REG_FIELD##_MASK)                                 \
      | (((val) << PER_REG_FIELD##_SHIFT) & PER_REG_FIELD##_MASK))

/*! \brief Macro to perform round-up division. */
#define ENET_DIV_ROUNDUP(val, div)            (((val) + (div) - 1) / (div))

/*! \brief Version field is not supported. */
#define ENET_VERSION_NONE                     (0xFFFFFFFFU)

/*! \brief MAC address length in bytes/octets. */
#define ENET_MAC_ADDR_LEN                     (6U)

/*! \brief Organization Unique Id (OUI) address length in bytes/octets. */
#define ENET_OUI_ADDR_LEN                     (3U)

/*! \brief IPv4 address length in bytes/octets. */
#define ENET_IPv4_ADDR_LEN                    (4U)

/*! \brief IPv6 address length in bytes/octets. */
#define ENET_IPv6_ADDR_LEN                    (16U)

/*! \brief Packet priority. */
#define ENET_PRI_NUM                          (8U)

/*! \brief Lowest packet priority. */
#define ENET_PRI_MIN                          (0U)

/*! \brief Highest packet priority. */
#define ENET_PRI_MAX                          (ENET_PRI_NUM - 1U)

/*! \brief Type of Service (ToS) priority. */
#define ENET_TOS_PRI_NUM                      (64U)

/*! \brief Lowest ToS priority. */
#define ENET_TOS_PRI_MIN                      (0U)

/*! \brief Highest ToS priority. */
#define ENET_TOS_PRI_MAX                      (ENET_TOS_PRI_NUM - 1U)

/*! \brief Maximum value for VLAN ID. */
#define ENET_VLAN_ID_MAX                      (4095U)

/*! \brief EtherType value for PTP over Ethernet Annex F (IEEE 802.3). */
#define ENET_ETHERTYPE_PTP                    (0x88F7U)

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/*!
 * \brief Ethernet peripheral type.
 */
typedef enum Enet_Type_e
{
    /*! "Null" peripheral - for testing only */
    ENET_NULL,

    /*! ICSSG_MAC: 1 MAC port */
    ENET_ICSSG_DUALMAC,

    /*! ICSSG: 1 host port + 2 MAC ports */
    ENET_ICSSG_SWITCH,

    /*! GMAC_3G: 1 host port + 2 MAC ports */
    ENET_GMAC_3G,

    /*! CPSW_2G: 1 host port + 1 MAC port */
    ENET_CPSW_2G,

    /*! CPSW_3G: 1 host port + 2 MAC port */
    ENET_CPSW_3G,

    /*! CPSW_5G: 1 host port + 4 MAC ports */
    ENET_CPSW_5G,

    /*! CPSW_9G: 1 host port + 8 MAC ports */
    ENET_CPSW_9G,

    /*! Number of Ethernet Peripheral types - For internal use only */
    ENET_TYPE_COUNT,
} Enet_Type;

/*! \brief Number of supported Ethernet peripheral types. */
#define ENET_TYPE_NUM                         (ENET_TYPE_COUNT + 1U)

/*!
 * \brief Enet driver magic value, used to indicate if driver is open or not.
 */
typedef enum Enet_Magic_e
{
    /*! Magic number used to identify when driver has been opened. */
    ENET_MAGIC = 0xABABABABU,

    /*! Magic number used to identify when driver is closed. */
    ENET_NO_MAGIC = 0x0U,
} Enet_Magic;

/*!
 * \brief CRC type.
 */
typedef enum Enet_CrcType_e
{
    /*! Ethernet CRC type */
    ENET_CRC_ETHERNET   = 0U,

    /*! Castagnoli CRC type */
    ENET_CRC_CASTAGNOLI,
} Enet_CrcType;

/*!
 * \brief Ethernet events.
 *
 * Used for any Enet APIs that are event related such as:
 * - Ethernet event types to poll for in Enet_poll() function. A poll bit mask
 *   can be constructed with any of these poll events.
 * - Event callback registration, i.e. link up, link down, timestamp push events,
 *   etc.
 */
typedef enum Enet_Event_e
{
    /*! No event */
    ENET_EVT_NONE = 0U,

    /*! Response to an asynchronous command.
     *
     * This event type doesn't require any additional argument in Enet_poll(),
     * caller should pass arg = NULL and argSize = 0. */
    ENET_EVT_ASYNC_CMD_RESP = ENET_BIT(0U),

    /*! Software timestamp push event. Currently not used. */
    ENET_EVT_TIMESTAMP_SWPUSH = ENET_BIT(1U),

    /*! Hardware timestamp push event. Currently not used. */
    ENET_EVT_TIMESTAMP_HWPUSH = ENET_BIT(2U),

    /*! Timestamp half rollover event. Currently not used. */
    ENET_EVT_TIMESTAMP_ROLLOVER = ENET_BIT(3U),

    /*! Timestamp rollover event. Currently not used. */
    ENET_EVT_TIMESTAMP_HALFROLLOVER = ENET_BIT(4U),

    /*! Timestamp receive event. Currently not used.  */
    ENET_EVT_TIMESTAMP_RX = ENET_BIT(5U),

    /*! Timestamp transmit event.
     *  For CPSW, the timestamp is returned in clock cycles. For ICSSG, the
     *  timestamp is returned in nanoseconds.
     *
     *  This event requires the port number be passed to Enet_poll(), caller should
     *  pass the port number as arg and argSize = sizeof(#Enet_MacPort). */
    ENET_EVT_TIMESTAMP_TX = ENET_BIT(6U),

    /*! Timestamp compare event.  Currently not used. */
    ENET_EVT_TIMESTAMP_COMP = ENET_BIT(7U),

    /*! Timestamp host transmit event.  Currently not used. */
    ENET_EVT_TIMESTAMP_HOSTTX = ENET_BIT(8U),

    /*! Any event. It could be used to flush all events when no callback
     *  is passed */
    ENET_EVT_ANY = 0xFFFFFFFFU,
} Enet_Event;

/*!
 * \brief Version of a peripheral or module.
 *
 * Version of a peripheral or module.  It's the type returned by any GET_VERSION
 * IOCTL command.
 *
 * Version fields that are not supported or applicable will be set to
 * #ENET_VERSION_NONE.
 */
typedef struct Enet_Version_s
{
    /*! Major version */
    uint32_t maj;

    /*! Minor version */
    uint32_t min;

    /*! RTL version */
    uint32_t rtl;

    /*! Identification */
    uint32_t id;

    /*! Other version value. No specific definition enforced */
    uint32_t other1;

    /*! Other version value. No specific definition enforced */
    uint32_t other2;
} Enet_Version;

/*!
 * \brief MAC interface speed.
 */
typedef enum Enet_Speed_e
{
    /*! 10 Mbps */
    ENET_SPEED_10MBIT = 0U,

    /*! 100 Mbps */
    ENET_SPEED_100MBIT,

    /*! 1 Gbps */
    ENET_SPEED_1GBIT,

    /*! Speed determined automatically */
    ENET_SPEED_AUTO,
} Enet_Speed;

/*!
 * \brief MAC interface duplexity.
 */
typedef enum Enet_Duplexity_e
{
    /*! Half duplex */
    ENET_DUPLEX_HALF = 0U,

    /*! Full duplex */
    ENET_DUPLEX_FULL,

    /*! Duplexity determined automatically */
    ENET_DUPLEX_AUTO,
} Enet_Duplexity;

/*!
 * \brief Double tagging VLAN type.
 */
typedef enum Enet_VlanTagType_e
{
    /*! Inner or customer tag */
    ENET_VLAN_TAG_TYPE_INNER = 0U,

    /*! Outer or service tag */
    ENET_VLAN_TAG_TYPE_OUTER,
} Enet_VlanTagType;

/*!
 * \brief VLAN tag.
 */
typedef struct Enet_VlanTag_s
{
    /*! Tag protocol identifier (TPID) */
    uint16_t tpid;

    /*! Priority code point (PCP) */
    uint8_t pcp;

    /*! Drop elegible indicator (DEI) */
    uint8_t dei;

    /*! VLAN id (VID) */
    uint16_t vlanId;

    /*! Whether is an inner or outer VLAN tag */
    Enet_VlanTagType tagType;
} Enet_VlanTag;

/*!
 * \brief MAC port.
 */
typedef enum Enet_MacPort_e
{
    /*! First MAC port */
    ENET_MAC_PORT_FIRST = 0U,

    /*! MAC port 1 */
    ENET_MAC_PORT_1     = ENET_MAC_PORT_FIRST,

    /*! MAC port 2 */
    ENET_MAC_PORT_2     = 1U,

    /*! MAC port 3 */
    ENET_MAC_PORT_3     = 2U,

    /*! MAC port 4 */
    ENET_MAC_PORT_4     = 3U,

    /*! MAC port 5 */
    ENET_MAC_PORT_5     = 4U,

    /*! MAC port 6 */
    ENET_MAC_PORT_6     = 5U,

    /*! MAC port 7 */
    ENET_MAC_PORT_7     = 6U,

    /*! MAC port 8 */
    ENET_MAC_PORT_8     = 7U,

    /*! Last MAC port - Used to count the number of ports */
    ENET_MAC_PORT_LAST  = ENET_MAC_PORT_8,
} Enet_MacPort;

/*!
 * \brief Event callback.
 *
 * \param evt          Event being registered for
 * \param evtNum       Event number (0 if single event)
 * \param evtCbArgs    Callback argument given during registration in
 *                     Enet_registerEventCb()
 * \param arg1         Event specific argument
 * \param arg2         Event specific argument
 */
typedef void (*Enet_EventCallback)(Enet_Event evt,
                                   uint32_t evtNum,
                                   void *evtCbArgs,
                                   void *arg1,
                                   void *arg2);

/*! \brief Number of MAC ports - For internal use only. */
#define ENET_MAC_PORT_NUM                     ((uint32_t)ENET_MAC_PORT_LAST + 1U)

/*!
 * \brief Normalize #Enet_MacPort.
 *
 * Macro to normalize #Enet_MacPort. It takes an #Enet_MacPort enum and converts
 * it to a zero-based index.
 */
#define ENET_MACPORT_NORM(n)                  ((n) - ENET_MAC_PORT_FIRST)

/*!
 * \brief De-normalize #Enet_MacPort.
 *
 * Macro to denormalize MAC port number. It takes a zero-based port number and
 * converts it to a #Enet_MacPort enum.
 */
#define ENET_MACPORT_DENORM(n)                ((Enet_MacPort)((n) + ENET_MAC_PORT_FIRST))

/*!
 * \brief Convert #Enet_MacPort to an integer id.
 *
 * Converts #Enet_MacPort to an integer id that corresponds to the port number.
 * This macro is expected to be used in prints/traces to let the user know the
 * port number in a way that is consistent with the #Enet_MacPort names.
 */
#define ENET_MACPORT_ID(n)                    ((n) - ENET_MAC_PORT_FIRST + 1U)

/*! \brief MAC invalid port number. Used for error checks only. */
#define ENET_MAC_PORT_INV                     ((Enet_MacPort)0xFFFFU)

/*! \brief TX traffic class invalid value. Used for error checks only. */
#define ENET_TRAFFIC_CLASS_INV                ((uint32_t)0xFFFFU)

/*! \brief Convert #Enet_MacPort to a bit mask.
 *
 * Converts a #Enet_MacPort to a bit mask to be used in APIs that take a
 * port mask.
 */
#define ENET_MACPORT_MASK(n)                  (ENET_BIT(ENET_MACPORT_NORM(n)))

/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/*!
 * \brief Check if Ethernet peripheral type is part of CPSW family.
 *
 * \param enetType  Ethernet peripheral type
 *
 * \return true if CPSW family, false otherwise.
 */
static inline bool Enet_isCpswFamily(Enet_Type enetType);

/*!
 * \brief Check if Ethernet peripheral type is part of ICSS family.
 *
 * \param enetType  Ethernet peripheral type
 *
 * \return true if ICSS family, false otherwise.
 */
static inline bool Enet_isIcssFamily(Enet_Type enetType);

/* ========================================================================== */
/*                        Deprecated Function Declarations                    */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

static inline bool Enet_isCpswFamily(Enet_Type enetType)
{
    return ((enetType == ENET_GMAC_3G) ||
            (enetType == ENET_CPSW_2G) ||
            (enetType == ENET_CPSW_3G) ||
            (enetType == ENET_CPSW_5G) ||
            (enetType == ENET_CPSW_9G));
}

static inline bool Enet_isIcssFamily(Enet_Type enetType)
{
    return ((enetType == ENET_ICSSG_DUALMAC) ||
            (enetType == ENET_ICSSG_SWITCH));
}

#ifdef __cplusplus
}
#endif

#endif /* ENET_TYPES_H_ */

/*! @} */
