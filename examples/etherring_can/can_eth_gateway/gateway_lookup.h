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

/*!
 * \file  gateway_lookup.h
 *
 * \brief This file contains the CAN ETH Gateway lookup
 */
#ifndef ETHERRING_CAN_ETH_GATEWAY_LOOKUP_H_
#define ETHERRING_CAN_ETH_GATEWAY_LOOKUP_H_
/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */

#include "ti_board_open_close.h"
#ifdef __cplusplus
extern "C" {
#endif
/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */
/* Size definitions */
#define ENETAPP_LOOKUP_TABLE_SIZE                           5U
/* ========================================================================== */
/*                                 Structures                                 */
/* ========================================================================== */
/* EnetApp_lookupEntry structure definition */
typedef struct
{
    uint32_t canID;
    uint8_t destMac[ENET_MAC_ADDR_LEN];
} EnetApp_lookupEntry;

/* Main lookup system structure that will be shared across files */
typedef struct
{
    EnetApp_lookupEntry lookupTable[ENETAPP_LOOKUP_TABLE_SIZE];  /* Main lookup table */
    EnetApp_lookupEntry tempArray[ENETAPP_LOOKUP_TABLE_SIZE];    /* Temporary array for sorting */
    bool isTableInitialized;                     /* Initialization state flag */
} EnetApp_lookupSystem;

/* ========================================================================== */
/*                           Function Declarations                            */
/* ========================================================================== */

/**
 * @brief Merges two sorted subarrays into one sorted array
 *
 * @param pArr   Pointer to the array containing both subarrays
 * @param left   Start index of left subarray
 * @param mid    End index of left subarray
 * @param right  End index of right subarray
 * @param pTemp  Pointer to temporary array for merging
 */
static void GatewayLookup_merge(EnetApp_lookupEntry* pArr, uint32_t left, uint32_t mid, uint32_t right, EnetApp_lookupEntry* pTemp);

/**
 * @brief Recursive merge sort implementation
 *
 * @param pArr   Pointer to array to sort
 * @param left   Left boundary of the segment to sort
 * @param right  Right boundary of the segment to sort
 * @param pTemp  Pointer to temporary array for merge operations
 */
static void GatewayLookup_mergeSortRecursive(EnetApp_lookupEntry* pArr, uint32_t left, uint32_t right, EnetApp_lookupEntry* pTemp);

/**
 * @brief Sorts the lookup table by CAN ID using merge sort algorithm
 *
 * @param EnetApp_lookupSystem Pointer to the lookup system containing the table to sort
 *
 * @return void
 */
void GatewayLookup_sortLookupTableByCanId(EnetApp_lookupSystem *EnetApp_lookupSystem);

/* ========================================================================== */
/*                           Function Definitions                             */
/* ========================================================================== */
/**
* Performs binary search to find a MAC address based on canID
* Runtime complexity: O(log n)
*
* @param canID     The CAN ID to look up
* @param pDestMac  Pointer to store the MAC address (must be 6 bytes)
* @return          true if found, false if not found
*/
bool GatewayLookup_findMacAddressByCanId(EnetApp_lookupSystem *EnetApp_lookupSystem, uint32_t canID, uint8_t* pDestMac)
{
    /* Ensure the table is initialized */
    if (!EnetApp_lookupSystem->isTableInitialized)
    {
        GatewayLookup_sortLookupTableByCanId(EnetApp_lookupSystem);
    }
    /* Check if destination buffer is valid */
    if (pDestMac == NULL)
    {
        return false;
    }
    /* Binary search implementation */
    uint32_t left = 0;
    uint32_t right = ENETAPP_LOOKUP_TABLE_SIZE - 1;
    while (left <= right)
    {
        uint32_t mid = left + (right - left) / 2;
        if (EnetApp_lookupSystem->lookupTable[mid].canID == canID)
        {
            /* Found the entry - copy MAC address to destination */
            for (uint32_t i = 0; i < 6; i++)
            {
                pDestMac[i] = EnetApp_lookupSystem->lookupTable[mid].destMac[i];
            }
            return true;
        }
        if (EnetApp_lookupSystem->lookupTable[mid].canID < canID)
        {
            left = mid + 1;  /* Search in right half */
        }
        else
        {
            right = mid - 1; /* Search in left half */
        }
    }
    /* CanID not found */
    return false;
}

static void GatewayLookup_merge(EnetApp_lookupEntry* pArr, uint32_t left, uint32_t mid, uint32_t right, EnetApp_lookupEntry* pTemp)
{
    uint32_t i = left;      /* Index for left subarray */
    uint32_t j = mid + 1;   /* Index for right subarray */
    uint32_t k = left;      /* Index for temporary array */
    /* Merge the two subarrays into temp array */
    while ((i <= mid) && (j <= right))
    {
        if (pArr[i].canID <= pArr[j].canID) {
            pTemp[k++] = pArr[i++];
        } else {
            pTemp[k++] = pArr[j++];
        }
    }
    /* Copy remaining elements from left subarray if any */
    while (i <= mid)
    {
        pTemp[k++] = pArr[i++];
    }
    /* Copy remaining elements from right subarray if any */
    while (j <= right)
    {
        pTemp[k++] = pArr[j++];
    }
    /* Copy back the merged elements to original array */
    for (i = left; i <= right; i++)
    {
        pArr[i] = pTemp[i];
    }
}

static void GatewayLookup_mergeSortRecursive(EnetApp_lookupEntry* pArr, uint32_t left, uint32_t right, EnetApp_lookupEntry* pTemp)
{
    if (left < right)
    {
        uint32_t  mid = left + (right - left) / 2;
        /* Sort first and second halves */
        GatewayLookup_mergeSortRecursive(pArr, left, mid, pTemp);
        GatewayLookup_mergeSortRecursive(pArr, mid + 1, right, pTemp);
        /* Merge the sorted halves */
        GatewayLookup_merge(pArr, left, mid, right, pTemp);
    }
}

void GatewayLookup_sortLookupTableByCanId(EnetApp_lookupSystem *EnetApp_lookupSystem)
{
    if (EnetApp_lookupSystem == NULL)
    {
        return;  /* Nothing to sort */
    }
    GatewayLookup_mergeSortRecursive(EnetApp_lookupSystem->lookupTable, 0, ENETAPP_LOOKUP_TABLE_SIZE - 1, EnetApp_lookupSystem->tempArray);
    EnetApp_lookupSystem->isTableInitialized = true;
}
#ifdef __cplusplus
}
#endif
#endif /* ETHERRING_CAN_ETH_GATEWAY_LOOKUP_H_ */
