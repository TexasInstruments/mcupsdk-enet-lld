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

#ifndef __QOSAPP_MISC_H__
#define __QOSAPP_MISC_H__

#define DEFAULT_VLAN_ID     (110)
#define AVTP_AAF_SUBTYPE    (2)
/* Default interface configured for the application, please change it
 * if you want to test it for the port other than mac port 1 (tilld0)
 * 0: index of the interface tilld0
*/
#define DEFAULT_INTERFACE_INDEX (0)

#define MAX_KEY_SIZE            (256)
#define MAX_LOG_LEN             (256)
#define MAX_VAL_SIZE            (64)

#define QOSAPP_MAX_STREAMS      (8)
#define QOSAPP_NUM_OF_STREAMS   (7)
#define QOSAPP_TASK_PRIORITY    (2)
#define QOSAPP_PRIORITY_MAX     (8)

#define TRAFFIC_CLASS_NODE      "/ietf-interfaces/interfaces/interface|name:%s|" \
                                "/bridge-port/traffic-class"
#define TRAFFIC_CLASS_TABLE_NODE TRAFFIC_CLASS_NODE"/traffic-class-table"
#define TRAFFIC_CLASS_DATA_NODE  TRAFFIC_CLASS_NODE"/tc-data"
#define PHYSICAL_QUEUE_MAP_NODE  TRAFFIC_CLASS_NODE"/pqueue-map"

UB_ABIT32_FIELD(cmsh_sv, 23, 0x1) // cmsh_sv_bit_field, cmsh_sv_set_bit_field

typedef struct QoSAppCommonParam
{
    /*! Name of network interface */
    char *netdev;
    /*! index is priority, value of each index is TC, -1: not used. */
    int8_t priority2TcMapping[QOSAPP_PRIORITY_MAX];
    uint8_t nTCs;                 /*! Num of traffic classes */
    uint8_t nQueues;              /*! Num of HW queue */
} QoSAppCommonParam_t;

typedef struct EnetQoSApp_AppCtx
{
    /*! An active network interface used for this app */
    char *netdev[MAX_NUMBER_ENET_DEVS];
    /*! How many network interfaces */
    int32_t netdevSize;
    /*! A delay offset for applying a schedule in microsecond unit */
    int64_t adminDelayOffset;

    /*! A pointer to a context object. */
    void *ectx;
} EnetQoSApp_AppCtx_t;

/*!
 * \brief Open a Yang DB for reading or writing.
 *
 * \param dbarg [OUT]  Keep necessary handles after openning a DB.
 * \param dbName[IN] Name of the DB. Could be null to use default DB.
 * \param mode  [IN] r: reading ; w: writing mode
 *
 * \return 0: On Sucess; -1: on Failure
 */
int  EnetQoSApp_openDB(EnetApp_dbArgs *dbarg,
                       char *dbName,
                       const char *mode);

/*!
 * \brief Close a Yang DB after use.
 *
 * \param dbarg [IN]  Hold necessary handles for closing a DB.
 */
void EnetQoSApp_closeDB(EnetApp_dbArgs *dbarg);

/*!
 * \brief Convert network interface name to portIndex.

 * \param ctx   [IN] Point to context object of the application
 * \param netdev[IN] Network interface name.
 *
 * \return -1: on error: Other than -1: port index of the interface
 */
int8_t EnetQoSApp_getPortIdx(EnetQoSApp_AppCtx_t *ctx, char *netdev);

/*!
 * \brief Set common parameters for all streams in the yang DB.

 * \param dbarg [IN] necessary handles to access a DB.
 * \param ctx   [IN] Point to context object of the application
 *
 * \return 0: On Sucess; -1: on Failure
 */
int EnetQoSApp_setCommonParam(QoSAppCommonParam_t *prm,
                              EnetApp_dbArgs *dbarg);

/*!
 * \brief Get current time.

* \return 0: On Failure; != 0: Current time in microsecond unit.
 */
uint64_t EnetQoSApp_getCurrentTimeUs(void);

#endif
