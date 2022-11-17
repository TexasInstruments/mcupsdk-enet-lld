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
 * \file  enet_ioctl.h
 *
 * \brief This file contains the type definitions and helper macros for the
 *        Enet IOCTL interface.
 */

/*!
 * \addtogroup ENET_MAIN_API
 *
 * @{
 */

#ifndef ENET_IOCTL_H_
#define ENET_IOCTL_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <stdbool.h>
#include <enet_cfg.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */

/*! \brief IOCTL type bit offset. */
#define ENET_IOCTL_TYPE_OFFSET                (24U)

/*! \brief IOCTL hardware peripheral bit offset. */
#define ENET_IOCTL_PER_OFFSET                 (16U)

/*! \brief IOCTL major number bit offset. */
#define ENET_IOCTL_MAJOR_OFFSET               (8U)

/*! \brief IOCTL minor number bit offset. */
#define ENET_IOCTL_MINOR_OFFSET               (0U)

/*! \brief Helper macro to get the IOCTL type (public/private). */
#define ENET_IOCTL_GET_TYPE(x)                ((x) & 0xFF000000U)

/*! \brief Helper macro to get the peripheral type. */
#define ENET_IOCTL_GET_PER(x)                 ((x) & 0x00FF0000U)

/*! \brief Helper macro to get the IOCTL major number. */
#define ENET_IOCTL_GET_MAJ(x)                 ((x) & 0x0000FF00U)

/*! \brief Helper macro to get the IOCTL major number. */
#define ENET_IOCTL_GET_MIN(x)                 ((x) & 0x000000FFU)

/*! \brief Helper macro to set the IOCTL type. */
#define ENET_IOCTL_TYPE(x)                    ((x) << ENET_IOCTL_TYPE_OFFSET)

/*! \brief Helper macro to set the IOCTL type. */
#define ENET_IOCTL_PER(x)                     ((x) << ENET_IOCTL_PER_OFFSET)

/*! \brief Helper macro to set the IOCTL major number. */
#define ENET_IOCTL_MAJ(x)                     ((x) << ENET_IOCTL_MAJOR_OFFSET)

/*! \brief Helper macro to set the IOCTL minor number. */
#define ENET_IOCTL_MIN(x)                     ((x) << ENET_IOCTL_MINOR_OFFSET)

/*!
 * \brief Set null args for an IOCTL command that takes no arguments.
 */
#define ENET_IOCTL_SET_NO_ARGS(prms) \
    {                                \
        (prms)->inArgs      = NULL;  \
        (prms)->inArgsSize  = 0U;    \
        (prms)->outArgs     = NULL;  \
        (prms)->outArgsSize = 0U;    \
    }

/*!
 * \brief Set the input args for an IOCTL command.
 */
#define ENET_IOCTL_SET_IN_ARGS(prms, in)     \
    {                                        \
        (prms)->inArgs      = (in);          \
        (prms)->inArgsSize  = sizeof(*(in)); \
        (prms)->outArgs     = NULL;          \
        (prms)->outArgsSize = 0U;            \
    }

/*!
 * \brief Set the output args for an IOCTL command.
 */
#define ENET_IOCTL_SET_OUT_ARGS(prms, out)    \
    {                                         \
        (prms)->inArgs      = NULL;           \
        (prms)->inArgsSize  = 0U;             \
        (prms)->outArgs     = (out);          \
        (prms)->outArgsSize = sizeof(*(out)); \
    }

/*!
 * \brief Set the input and output args for an IOCTL command.
 */
#define ENET_IOCTL_SET_INOUT_ARGS(prms, in, out) \
    {                                            \
        (prms)->inArgs      = (in);              \
        (prms)->inArgsSize  = sizeof(*(in));     \
        (prms)->outArgs     = (out);             \
        (prms)->outArgsSize = sizeof(*(out));    \
    }

/*!
 * \brief Helper macro used to add an entry in a IOCTL valid information of
 *        type #Enet_IoctlValidate.
 */
#define ENET_IOCTL_VALID_PRMS(cmdId, inSize, outSize)       \
    [ENET_IOCTL_GET_MIN(cmdId)] =                           \
    {                                                       \
        .cmd         = (cmdId),                             \
        .inArgsSize  = (inSize),                            \
        .outArgsSize = (outSize),                           \
    }


/*!
 * \brief Helper macro used to first register IOCTL handler and then invoke the
 *        IOCTL
 *
 * This macro replaces the previous Enet_ioctl function which Issue an operation on the 
 * Enet Peripheral.
 *
 * Issues a control operation on the Enet Peripheral.  The IOCTL parameters
 * should be built using the helper macros provided by the driver:
 * - #ENET_IOCTL_SET_NO_ARGS - If the IOCTL command doesn't take any arguments
 * - #ENET_IOCTL_SET_IN_ARGS - If the IOCTL command takes only input arguments
 * - #ENET_IOCTL_SET_OUT_ARGS - If the IOCTL command takes only output arguments
 * - #ENET_IOCTL_SET_INOUT_ARGS - If the IOCTL command takes input and output arguments
 *
 * \param hEnet        Enet driver handle
 * \param coreId       Caller's core id
 * \param ioctlCmd     IOCTL command Id
 * \param prms         IOCTL parameters
 * \param status       return status of the Enet_ioctl. 
 *                     #ENET_SOK if operation is synchronous and it was successful.
 *                     #ENET_SINPROGRESS if operation is asynchronous and was initiated sucessfully.
 *                     \ref Enet_ErrorCodes in case of any failure.
 */
#define ENET_IOCTL(hEnet, coreId, ioctlCmd,prms,status)                           \
    do {                                                                          \
        extern int32_t Enet_ioctl(Enet_Handle enetHandle,                         \
                   uint32_t ioctlCoreId,                                          \
                   uint32_t cmd,                                                  \
                   Enet_IoctlPrms *ioctlPrms);                                    \
                                                                                  \
        extern int32_t Enet_ioctl_register_##ioctlCmd(Enet_Handle enetHandle,     \
                                                      uint32_t ioctlCoreId);      \
                                                                                  \
        status = Enet_ioctl_register_##ioctlCmd(hEnet, coreId);                   \
        if (ENET_SOK == status)                                                   \
        {                                                                         \
            status = Enet_ioctl(hEnet, coreId, ioctlCmd,prms);                    \
        }                                                                         \
    } while (0)


/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/*!
 * \brief IOCTL types.
 */
enum Enet_IoctlType_e
{
    /*! Public IOCTL type (default type if omitted) */
    ENET_IOCTL_TYPE_PUBLIC  = ENET_IOCTL_TYPE(1U),

    /*! Private IOCTL type */
    ENET_IOCTL_TYPE_PRIVATE = ENET_IOCTL_TYPE(2U),
};

/*!
 * \brief IOCTL peripheral types.
 */
enum Enet_IoctlPer_e
{
    /*! Generic IOCTLs */
    ENET_IOCTL_PER_GENERIC = ENET_IOCTL_PER(0U),

    /*! CPSW specific IOCTL (agnostic of number of CPSW ports) */
    ENET_IOCTL_PER_CPSW = ENET_IOCTL_PER(1U),

    /*! ICSSG specific IOCTL */
    ENET_IOCTL_PER_ICSSG = ENET_IOCTL_PER(2U),

    /*! GMAC specific IOCTL */
    ENET_IOCTL_PER_GMAC = ENET_IOCTL_PER(3U),
};

/*!
 * \brief IOCTL base number
 */
enum Enet_IoctlMajor_e
{
    /*! Enet peripheral IOCTL base */
    ENET_IOCTL_PER_BASE      = ENET_IOCTL_MAJ(1U),

    /*! FDB module IOCTL base */
    ENET_IOCTL_FDB_BASE      = ENET_IOCTL_MAJ(2U),

    /*! TimeSync module IOCTL base */
    ENET_IOCTL_TIMESYNC_BASE = ENET_IOCTL_MAJ(3U),

    /*! Host port module IOCTL base */
    ENET_IOCTL_HOSTPORT_BASE = ENET_IOCTL_MAJ(4U),

    /*! MAC port module IOCTL base */
    ENET_IOCTL_MACPORT_BASE  = ENET_IOCTL_MAJ(5U),

    /*! MDIO module IOCTL base */
    ENET_IOCTL_MDIO_BASE     = ENET_IOCTL_MAJ(6U),

    /*! Statistics module IOCTL base */
    ENET_IOCTL_STATS_BASE    = ENET_IOCTL_MAJ(7U),

    /*! PHY module IOCTL base */
    ENET_IOCTL_PHY_BASE      = ENET_IOCTL_MAJ(9U),

    /*! Resource Manager module IOCTL base */
    ENET_IOCTL_RM_BASE       = ENET_IOCTL_MAJ(10U),

    /*! TAS module IOCTL base */
    ENET_IOCTL_TAS_BASE       = ENET_IOCTL_MAJ(11U),
};

/*!
 * \brief Enet IOCTL param
 *
 * IOCTL params structure argument for all Enet IOCTL commands
 */
typedef struct Enet_IoctlPrms_s
{
    /*! Pointer to an actual input args struct for the IOCTL cmd */
    const void *inArgs;

    /*! Size of #inArgs structure */
    uint32_t inArgsSize;

    /*! Pointer to an actual output args struct for the IOCTL cmd */
    void *outArgs;

    /*! Size of #outArgs structure */
    uint32_t outArgsSize;
} Enet_IoctlPrms;

/*!
 * \brief Enet IOCTL expected param sizes.
 *
 * Expected input and output argument sizes of an Enet IOCTL.  This is useful
 * for validation purposes only.
 */
typedef struct Enet_IoctlValidate_s
{
    /*! Command id */
    uint32_t cmd;

    /*! Expected size of IOCTL input argument (#Enet_IoctlPrms.inArgsSize) */
    uint32_t inArgsSize;

    /*! Expected size of IOCTL output argument (#Enet_IoctlPrms.outArgsSize) */
    uint32_t outArgsSize;
} Enet_IoctlValidate;

/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/*!
 * \brief Check IOCTL input args for commands that expect an input
 *
 * Helper function to check the input arguments passed to an IOCTL for
 * commands that expect an input argument.
 *
 * \param prms         IOCTL parameters pointer
 * \param inArgsSize   Expected size of the input argument
 *
 * \return ENET_SOK if inArgs pointer is not NULL and inArgsSize matches the
 *         expected size. Otherwise, ENET_EMALFORMEDIOCTL.
 */
static inline uint32_t Enet_checkInArgs(const Enet_IoctlPrms *prms,
                                        uint32_t inArgsSize);

/*!
 * \brief Check IOCTL input args for commands that don't expect an input
 *
 * Helper function to check the input arguments passed to an IOCTL for
 * commands that don't expect an input.
 *
 * \param prms         IOCTL parameters pointer
 *
 * \return ENET_SOK if inArgs pointer is NULL and inArgsSize is 0.
 *         Otherwise, ENET_EMALFORMEDIOCTL.
 */
static inline uint32_t Enet_checkNoInArgs(const Enet_IoctlPrms *prms);

/*!
 * \brief Check IOCTL output args for commands that expect an output
 *
 * Helper function to check the output arguments passed to an IOCTL for
 * commands that expect an output argument.
 *
 * \param prms         IOCTL parameters pointer
 * \param outArgsSize  Expected size of the output argument
 *
 * \return ENET_SOK if outArgs pointer is not NULL and outArgsSize matches the
 *         expected size. Otherwise, ENET_EMALFORMEDIOCTL.
 */
static inline uint32_t Enet_checkOutArgs(const Enet_IoctlPrms *prms,
                                         uint32_t outArgsSize);

/*!
 * \brief Check IOCTL output args for commands that don't have output
 *
 * Helper function to check the output arguments passed to an IOCTL for
 * commands that don't have an output argument.
 *
 * \param prms         IOCTL parameters pointer
 *
 * \return ENET_SOK if outArgs pointer is NULL and outArgsSize is 0.
 *         Otherwise, ENET_EMAILFORMEDIOCTL.
 */
static inline uint32_t Enet_checkNoOutArgs(const Enet_IoctlPrms *prms);

/*!
 * \brief Check IOCTL input and output args
 *
 * Helper function to check the input and output arguments passed to an IOCTL
 * for commands that expect input and output arguments.
 *
 * \param prms         IOCTL parameters pointer
 * \param inArgsSize   Expected size of the input argument
 * \param outArgsSize  Expected size of the output argument
 *
 * \return ENET_SOK if outArgs pointer is NULL and outArgsSize is 0.
 *         Otherwise, ENET_EMAILFORMEDIOCTL.
 */
static inline uint32_t Enet_checkInOutArgs(const Enet_IoctlPrms *prms,
                                           uint32_t inArgsSize,
                                           uint32_t outArgsSize);

#if ENET_CFG_IS_ON(DEV_ERROR)
/*!
 * \brief Validate IOCTL command parameters.
 *
 * Validate IOCTL parameters being passed against the expected parameters
 * for a given IOCTL command.
 *
 * The caller of this function must pass an array of valid IOCTLs via
 * #validIoctls arguments.  The minor number of the IOCTL command is used
 * as an index of the array.
 *
 * It's recommended that a given component (EnetPer or EnetMod) only validates
 * the IOCTL commands owned by such component.  Generic IOCTL commands (i.e.
 * those defined by Enet) will be checked by the Enet core layer.
 *
 * This functions returns #ENET_EMALFORMEDIOCTL if any of the following
 * conditions is detected:
 *  - IOCTL params pointer is NULL.
 *  - IOCTL command id doesn't match valid IOCTL table command id.
 *  - IOCTL input argument size doesn't match the expected size for the given
 *    command id.
 *  - IOCTL output argument size doesn't match the expected size for the given
 *    command id.
 *  - IOCTL input argument pointer is NULL but the argument size is not 0.
 *  - IOCTL output argument pointer is NULL but the argument size is not 0.
 *
 * \param cmd              IOCTL command Id
 * \param prms             IOCTL parameters
 * \param validIoctls      Array of valid IOCTLs and their expected params sizes
 * \param numValidIoctls   Number of elements in the valid IOCTL array
 *
 * \retval #ENET_SOK             IOCTL params met the valid criteria (params sizes).
 * \retval #ENET_MALFORMEDIOCTL  IOCTL parameters didn't meet the valid criteria
 *                               and one of the conditions described above was found.
 * \retval #ENET_EINVALIDPARAMS  IOCTL minor number is not in the passed #validIoctls
 *                               table.
 */
int32_t Enet_validateIoctl(uint32_t cmd,
                           const Enet_IoctlPrms *prms,
                           const Enet_IoctlValidate *validIoctls,
                           uint32_t numValidIoctls);
#endif

/* ========================================================================== */
/*                        Deprecated Function Declarations                    */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

static inline uint32_t Enet_checkInArgs(const Enet_IoctlPrms *prms,
                                        uint32_t inArgsSize)
{
    return ((prms != NULL) &&
            (prms->inArgs != NULL) &&
            (prms->inArgsSize == inArgsSize)) ?
           ENET_SOK : ENET_EMALFORMEDIOCTL;
}

static inline uint32_t Enet_checkNoInArgs(const Enet_IoctlPrms *prms)
{
    return ((prms != NULL) &&
            (prms->inArgs == NULL) &&
            (prms->inArgsSize == 0U)) ?
           ENET_SOK : ENET_EMALFORMEDIOCTL;
}

static inline uint32_t Enet_checkOutArgs(const Enet_IoctlPrms *prms,
                                         uint32_t outArgsSize)
{
    return ((prms != NULL) &&
            (prms->outArgs != NULL) &&
            (prms->outArgsSize == outArgsSize)) ?
           ENET_SOK : ENET_EMALFORMEDIOCTL;
}

static inline uint32_t Enet_checkNoOutArgs(const Enet_IoctlPrms *prms)
{
    return ((prms != NULL) &&
            (prms->outArgs == NULL) &&
            (prms->outArgsSize == 0U)) ?
           ENET_SOK : ENET_EMALFORMEDIOCTL;
}

static inline uint32_t Enet_checkInOutArgs(const Enet_IoctlPrms *prms,
                                           uint32_t inArgsSize,
                                           uint32_t outArgsSize)
{
    return ((prms != NULL) &&
            (prms->inArgs != NULL) &&
            (prms->inArgsSize == inArgsSize) &&
            (prms->outArgs != NULL) &&
            (prms->outArgsSize == outArgsSize)) ?
           ENET_SOK : ENET_EMALFORMEDIOCTL;
}

#ifdef __cplusplus
}
#endif

#endif /* ENET_IOCTL_H_ */

/*! @} */
