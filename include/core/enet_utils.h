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
 * \file  enet_utils.h
 *
 * \brief This file contains the type definitions and function prototypes
 *        of the Enet Utils functionality.
 */

#ifndef ENET_UTILS_H_
#define ENET_UTILS_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stddef.h>
#include <string.h>
#include <enet_cfg.h>
#include <include/core/enet_types.h>
#include <include/core/enet_mod_macport.h>
#include <priv/core/enet_trace_priv.h>
#include <include/phy/enetphy.h>

#if defined(__KLOCWORK__) || defined(__cplusplus)
#include <stdlib.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */

/*!
 * \brief Macro to get container structure from one of its members' addresses.
 */
#define container_of(ptr, type, member)                      \
    ({                                                       \
         const typeof(((type *)0)->member) * __mptr = (ptr); \
         (type *)((char *)__mptr - offsetof(type, member));  \
     })

/*!
 * \brief Unused variable.
 */
#define ENET_UNUSED(x)                        (x = x)

/*!
 * \brief Macro to create a bit mask.
 */
#define ENET_MK_ONES(c_ebit, c_sbit)          (((1U << (((c_ebit) - (c_sbit)) + 1U)) - 1U) << (c_sbit))

/*! \brief Align a value by performing a round-up operation */
#define ENET_UTILS_ALIGN(x, y)                ((((x) + ((y) - 1)) / (y)) * (y))

/*! \brief Macro to determine if an address is aligned to a given size. */
#define ENET_UTILS_IS_ALIGNED(addr, alignSz)  (((uintptr_t)addr & ((alignSz) - 1U)) == 0U)

/*! \brief Macro to copy arrays. They must be of the same size */
#define ENET_UTILS_ARRAY_COPY(dst, src)                                  \
    do                                                                   \
    {                                                                    \
        /* dst argument of macro should be array and not pointer.*/      \
        ENET_UTILS_COMPILETIME_ASSERT(sizeof(dst) != sizeof(uintptr_t)); \
        memcpy(dst, src, (ENET_ARRAYSIZE(dst) * sizeof(dst[0])));  \
    } while (0)

/*!
 * \brief Assertion.
 */
#if ENET_CFG_IS_ON(ASSERT)
#if defined(__KLOCWORK__) || defined(__cplusplus)
#define Enet_assert(cond, ...)            do { if (!(cond)) abort(); } while (0)
#else /* !defined(__KLOCWORK__) && !defined(__cplusplus) */
#define Enet_assert(cond, ...)                               \
    do {                                                     \
        bool assertCond = (bool)(cond);                      \
        ENETTRACE_ERR_IF(!assertCond, __VA_ARGS__);          \
        EnetUtils_assertLocal(assertCond,                    \
                              (const char *)# cond,          \
                              (const char *)__FILE__,        \
                              (int32_t)__LINE__);            \
    } while (0)
#endif /* defined(__KLOCWORK__) || defined(__cplusplus) */
#else /* !ENET_CFG_IS_ON(ASSERT) */
#define Enet_assert(cond, ...)           (void)(cond)
#endif /* ENET_CFG_IS_ON(ASSERT) */

/*!
 * \brief Development-time assertion.
 */
#if ENET_CFG_IS_ON(ASSERT)
#if defined(__KLOCWORK__) || defined(__cplusplus)
#define Enet_devAssert(cond, ...)         do { if (!(cond)) abort(); } while (0)
#else /* !defined(__KLOCWORK__) && !defined(__cplusplus) */
#define Enet_devAssert(cond, ...)                            \
    do {                                                     \
        ENETTRACE_ERR_IF(!(bool)(cond), __VA_ARGS__);        \
        EnetUtils_assertLocal((bool)(cond),                  \
                              (const char *)# cond,          \
                              (const char *)__FILE__,        \
                              (int32_t)__LINE__);            \
    } while (0)
#endif /* defined(__KLOCWORK__) || defined(__cplusplus) */
#else /* !ENET_CFG_IS_ON(ASSERT) || !ENET_CFG_IS_ON(DEV_ERROR) */
#define Enet_devAssert(cond, ...)             (void)(cond)
#endif /* ENET_CFG_IS_ON(ASSERT) && ENET_CFG_IS_ON(DEV_ERROR) */

/*!
 * \brief Compile-time assertion.
 */
#define ENET_UTILS_COMPILETIME_ASSERT(cond)                 \
    do {                                                    \
        typedef char ErrorCheck[((cond) == true) ? 1 : -1]; \
        ErrorCheck a = {0};                                 \
        a[0U] = a[0U];                                      \
    } while (0)

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/*!
 * \brief Info/debug print function prototype.
 *
 * This function is used by the driver to print info/debug messages.
 *
 * \param fmt          Formatted string followed by variable arguments
 */
typedef void (*Enet_Print)(const char *fmt, ...);

/*!
 * \brief Virtual-to-physical address translation callback function.
 *
 * This function is used by the driver to convert virtual address to physical
 * address.
 *
 * \param virtAddr      Virtual address
 * \param appData       Callback pointer passed during translation
 *
 * \return Translated physical address
 */
typedef uint64_t (*Enet_VirtToPhys)(const void *virtAddr, void *appData);

/*!
 * \brief Physical-to-virtual address translation callback function.
 *
 * This function is used by the driver to convert physical address to virtual
 * address.
 *
 * \param phyAddr       Physical address
 * \param appData       Callback pointer passed during translation
 *
 * \return Translated virtual address
 */
typedef void *(*Enet_PhysToVirt)(uint64_t phyAddr, void *appData);

/*!
 * \brief Enet utils parameters.
 */
typedef struct EnetUtils_Cfg_s
{
    /*! If not NULL, this function will be called to print debug/info message
     *  with appropriate string */
    Enet_Print print;

    /*! If not NULL, this function will be called to translate virtual address
     *  to physical address to be provided to the Enet driver.
     *  If NULL, the driver will assume a one-one mapping. */
    Enet_VirtToPhys virtToPhys;

    /*! If not NULL, this function will be called to translate physical address
     *  to virtual address to access the pointer returned by the Enet driver.
     *  If NULL, the driver will assume a one-one mapping. */
    Enet_PhysToVirt physToVirt;
} EnetUtils_Cfg;

/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/*!
 * \brief Initialize utils module.
 *
 * Utils module initialization function. Should be only called from the Enet
 * top-level module.
 *
 * \param cfg   Pointer to the initialization parameters
 */
void EnetUtils_init(const EnetUtils_Cfg *cfg);

/*!
 * \brief De-initialize utils module.
 */
void EnetUtils_deinit(void);

/*!
 * \brief Print function.
 *
 * Prints the provided formatted string.
 *
 * \param fmt          Formatted string followed by variable arguments
 */
void EnetUtils_printf(const char *fmt,
                      ...);

/*!
 * \brief Print function for va_list.
 *
 * Prints the provided formatted string.
 *
 * \param fmt          Formatted string
 * \param args         Arg list pointing to the arguments in the format string
 */
void EnetUtils_vprintf(const char *fmt,
                       va_list args);

/*!
 * \brief Returns minimum of two numbers.
 *
 * \param num1     First number
 * \param num2     Second number
 *
 * \return Minimum number
 */
uint32_t EnetUtils_min(uint32_t num1,
                       uint32_t num2);

/*!
 * \brief Returns minimum of two numbers.
 *
 * \param num1     First number
 * \param num2     Second number
 *
 * \return Maximum number
 */
uint32_t EnetUtils_max(uint32_t num1,
                       uint32_t num2);

/*!
 * \brief Busy loop for a given amount of cycles.
 *
 * \param delayVal  Delay time
 */
void EnetUtils_delay(uint32_t delayVal);

/*!
 * \brief Convert a virtual address to physical address.
 *
 * \param virtAddr   Virtual address
 * \param appData    Auxiliary data
 *
 * \return Physical address
 */
uint64_t EnetUtils_virtToPhys(const void *virtAddr,
                              void *appData);

/*!
 * \brief Convert a physical address to virtual address.
 *
 * \param physAddr   Physical address
 * \param appData    Auxiliary data
 *
 * \return Virtual address
 */
void *EnetUtils_physToVirt(uint64_t physAddr,
                           void *appData);

#if ENET_CFG_IS_ON(ASSERT) && !defined(__KLOCWORK__)
/*!
 * \brief Local assert used by Enet_assert() macro.
 *
 * \param cond        Boolean expression
 * \param str         String with condition to be printed if expression is false
 * \param fileName    Filename
 * \param lineNum     Line number
 */
static inline void EnetUtils_assertLocal(bool condition,
                                         const char *str,
                                         const char *fileName,
                                         int32_t lineNum);
#endif

/*!
 * \brief Convert MAC port MII to PHY MII types.
 *
 * Converts MII type definition from #EnetMacPort_Interface to
 * #EnetPhy_Mii.
 *
 * \param macMii     MAC port MII interface type
 *
 * \return ENETPHY MII interface
 */
EnetPhy_Mii EnetUtils_macToPhyMii(const EnetMacPort_Interface *macMii);

/*!
 * \brief Copy MAC address.
 *
 * Copies 6-byte MAC address.
 *
 * \param dst        Pointer to MAC address memory being copied into
 * \param src        Pointer to MAC address memory to be copied from
 */
static inline void EnetUtils_copyMacAddr(uint8_t *dst,
                                         const uint8_t *src);

/*!
 * \brief Compare two MAC address.
 *
 * Compares two MAC address to determine if they are equal.
 *
 * \param addr1      MAC address being compared
 * \param addr2      MAC address being compared
 *
 * \return true if MAC addresses are same, false otherwise.
 */
static inline bool EnetUtils_cmpMacAddr(const uint8_t *addr1,
                                        const uint8_t *addr2);

/*!
 * \brief Clear MAC address.
 *
 * Clear 6-byte MAC address memory.
 *
 * \param addr       Pointer to MAC address memory to be cleared
 */
static inline void EnetUtils_clearMacAddr(uint8_t *addr);

/*!
 * \brief Check if address is multicast.
 *
 * Checks if MAC address is multicast.
 *
 * \param addr       MAC address
 *
 * \return Whether MAC address is multicast or not.
 */
static inline bool EnetUtils_isMcastAddr(const uint8_t *addr);

/* ========================================================================== */
/*                        Deprecated Function Declarations                    */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

#if ENET_CFG_IS_ON(ASSERT) && !defined(__KLOCWORK__)
static inline void EnetUtils_assertLocal(bool condition,
                                         const char *str,
                                         const char *fileName,
                                         int32_t lineNum)
{
    volatile static bool gEnetAssertWaitInLoop = true;

    if (!condition)
    {
        EnetUtils_printf("Assertion @ Line: %d in %s: %s\n", lineNum, fileName, str);
        while (gEnetAssertWaitInLoop)
        {
            /* Do nothing */
        }
    }
}
#endif

static inline void EnetUtils_copyMacAddr(uint8_t *dst,
                                         const uint8_t *src)
{
    memcpy(dst, src, ENET_MAC_ADDR_LEN);
}

static inline bool EnetUtils_cmpMacAddr(const uint8_t *addr1,
                                        const uint8_t *addr2)
{
    return (memcmp(addr1, addr2, ENET_MAC_ADDR_LEN) == 0U);
}

static inline void EnetUtils_clearMacAddr(uint8_t *addr)
{
    memset(addr, 0, ENET_MAC_ADDR_LEN);
}

static inline bool EnetUtils_isMcastAddr(const uint8_t *addr)
{
    return ((addr[0U] & 1U) == 1U);
}

#ifdef __cplusplus
}
#endif

#endif /* ENET_UTILS_H_ */
