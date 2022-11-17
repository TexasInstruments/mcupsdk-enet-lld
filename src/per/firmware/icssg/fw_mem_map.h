/*
 * fw_mem_map.h
 *
 * Contains memory map for Ethernet firmware.
 * This file is used by Ethernet Switch driver.
 *
 * Copyright (C) 2020 Texas Instruments Incorporated - http://www.ti.com/
 *
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
 *
 */
#ifndef ____fw_mem_map_h
#define ____fw_mem_map_h 1

/************************* Ethernet Switch Constants *********************/
/* all sizes in bytes except otherwise mentioned */
#define NRT_TX_Q_HIGH_OFFSET                    0x0000

#define NRT_QUEUE_CONTEXT_SIZE                  (16)
#define NRT_DESC_QUEUE_CONTEXT_SIZE             (16)
#define NRT_PUSH_SIZE                           (32)
#define PACKET_DESC_SIZE                        (4)
#define BD_CUT_THROUGH_MAGIC_VALUE              (0xBB8)     /* used to indicate cut-through. This is an illegal packet size and does not overlap */
#define NRT_PRE_EMPTION_SIZE_THRESHOLD          (64)

#define R31_ERROR_BITS_MASK                     0x3180  /* Mask for extracting CRC & Min & Max error. Refer MII RT Spec */
#define MII_RT_ERROR_MINMAX_MASK                0xC  /* Mask for extracting Min/Max errors from MII RT registers. Refer MII RT Spec */
#define R31_ERROR_MIN_SHIFT                     0x5  /* Shift for extracting Min  errors from R31.b3. Refer MII RT Spec */

#define RX_B0_QNUM_SPECIAL_VALUE                0xFF    /* special pattern used to wait for IPC SPAD values from RTU0 */

/* queue numbers */
#define QUEUE_0                                 (0)
#define QUEUE_1                                 (1)
#define QUEUE_2                                 (2)
#define QUEUE_3                                 (3)
#define QUEUE_4                                 (4)
#define QUEUE_5                                 (5)
#define QUEUE_6                                 (6)
#define QUEUE_7                                 (7)

#define PORT_1_ID                               (1)
#define PORT_2_ID                               (2)
#define UNDIRECTED_PKT_ID                       (0)

/* if bucket size is changed in firmware then this too should be changed */
/* because it directly impacts FDB ageing calculation */
#define NUMBER_OF_FDB_BUCKET_ENTRIES            (4)
#define SIZE_OF_FDB                             (2048)  /* This is fixed in ICSSG */
/* ageing interval in nanoseconds. 30s */
#define DEFAULT_FDB_AGEING_INTERVAL             (0x6FC23AC00)

/* MSMC queue sizes */
#define NRT_NUM_PORT_QUEUES                     (16)                /* Number of Port Tx queues for both ports. 8 each */
#define NRT_NUM_HOST_QUEUES                     (16)                /* Number of Host Tx queues for both ports. 8 each */
#define NRT_NUM_HOST_EGRESS_QUEUES              (4)                 /* Number of Host Egress queues for both ports. 2 each */
#define NRT_PORT_QUEUE_SIZE                     (25 * 1024)         /* 25kB per port queue */
#define NRT_HOST_QUEUE_SIZE                     (6400)              /* ~6.5kB per host queue */
#define NRT_RESERVED_MEM                        (2048)

/* Descriptor Q sizes */
/*  See design doc. It's calculated as ((queue size) / 64) * 4 plus some additional descriptors for margin of safety */
#define NRT_PORT_DESC_QUEUE_SIZE                ((((NRT_PORT_QUEUE_SIZE + NRT_RESERVED_MEM) / 64) + 29) * 4)     /*  for 25kB queue size */
#define NRT_HOST_DESC_QUEUE_SIZE                ((((NRT_HOST_QUEUE_SIZE + NRT_RESERVED_MEM) / 64) + 9) * 4)   /*  for ~6kB queue size */

#define NRT_PORT_DESC_SMEM_SIZE                 0xE68       /* FIXME : Magic number? */
#define NRT_HOST_DESC_SMEM_SIZE                 0x468       /* FIXME : Magic number? */
#define NRT_SPECIAL_PD_SMEM_SIZE                0x400       /* FIXME : Magic number? */

/* TAS sizes */
#define TAS_NUM_WINDOWS                         (16)
#define TAS_NUM_QUEUES                          (8)

#define MTU_SIZE                                (2048)

#define MAX_VALID_FRAME_SIZE                    (2000)

#define MII_INTC_RX_ERROR_MASK                  (0x13)      /* Mask to extract CRC and Rx error from INCT bits 32-63 */
/* Bit 32 : PRU0_RX_ERR, 33 : PRU0_RX_ERR32, 36 : PRU0_RX_CRC */

/* default priority for packets without VLAN tag and special packet type */
#define DEFAULT_P1_PRIORITY                     (0)         /* default priority for P1 */
#define DEFAULT_P2_PRIORITY                     (0)         /* default priority for P2 */
#define DEFAULT_HOST_PRIORITY                   (0)         /* default priority for Host port */

/* ---------------------PSI packet types---------------------- */
/* They come in handy to match against R1, instead of comparing each bit field we do a single shot compare */
/* Info packet type meta data which comes in R1. It has bit 0 (sop), bit 4(lastw) set indicating it's the first and last packet of it's type */
#define INFO_PKT_META_DATA                      (0x0f000011)

/* Info packet type in R1 metadata. R1.b2 */
/* it is first and last chunk of it's type. packet type is 0 and size is f or 15 */
#define INFO_PKT_TYPE                           (0x00)

/* Info packet has the following meta data inside */
/* R2.t23 indicates if Host has inserted CRC in packet. Not used by switch */
/* R4.b2 tells if it's a directed packet. R4.b2 = 0(undirected), R4.b2 = 1(directed. P1), R4.b2 = 2(directed. P2) */

/* Control packet type meta data which comes in R1. It has bit 0 (sop), bit 4(lastw) set indicating it's the first and last packet of it's type */
#define CTRL_PKT_META_DATA                      (0x0f100011)

/* Ctrl packet type in R1 metadata. R1.b2 */
/* it is first and last chunk of it's type. packet type is 0x10 and size is 15 */
#define CTRL_PKT_TYPE                           (0x10)

/* Status packet type meta data which comes in R1. It has bit 0 (sop), bit 4(lastw) set indicating */
#define STATUS_PKT_META_DATA                    (0x0f180012)

/* Status packet type in R1 metadata. R1.b2 */
/* it is first and last chunk of it's type. packet type is 0x18 and size is 15 */
#define STATUS_PKT_TYPE                         (0x18)

/* Data packet type meta data which comes in R1. This is for the all chunks (except last) and  */
/* doesn't have lastw set. packet type is 0x14 and size is 15 */
#define DATA_FIRST_PKT_META_DATA                (0x10140000)

/* Data packet type meta data which comes in R1. This is for the last chunk and has */
/* lastw set. packet type is 0x14 and size is 0 because size determined by xout by widget */
#define DATA_LAST_PKT_META_DATA                 (0x00140010)

/* Same as above but has eop also set */
#define DATA_LAST_PKT_META_DATA_EOP             (0x00140012)

/* Data packet type in R1 metadata. R1.b2 */
#define DATA_PKT_TYPE                           (0x14)

#define NRT_PACKET_DROP_TS_ERROR_CODE           (0xFFFFFFFFFFFFFFFF)

#define DEFAULT_GUARD_BAND                      (0x4E2)     /*  1.25 microseconds */

#define FW_LINK_SPEED_1G                           (0x00)
#define FW_LINK_SPEED_100M                         (0x01)
#define FW_LINK_SPEED_100M_BIT_MASK                (0x00)
#define FW_LINK_SPEED_10M                          (0x02)
#define FW_LINK_SPEED_100M_HD                      (0x81)
#define FW_LINK_SPEED_10M_HD                       (0x82)

/*********************** Ethernet Switch Constants End *********************/

/* Memory Usage of
 * SHARED_MEMORY
 */
/*Time after which FDB entries are checked for aged out values. Value in nanoseconds*/
#define FDB_AGEING_TIMEOUT_OFFSET                          0x0014
/*default VLAN tag for Host Port*/
#define HOST_PORT_DF_VLAN_OFFSET                           0x001C
/*Same as HOST_PORT_DF_VLAN_OFFSET*/
#define EMAC_ICSSG_SWITCH_PORT0_DEFAULT_VLAN_OFFSET        HOST_PORT_DF_VLAN_OFFSET
/*default VLAN tag for P1 Port*/
#define P1_PORT_DF_VLAN_OFFSET                             0x0020
/*Same as P1_PORT_DF_VLAN_OFFSET*/
#define EMAC_ICSSG_SWITCH_PORT1_DEFAULT_VLAN_OFFSET        P1_PORT_DF_VLAN_OFFSET
/*default VLAN tag for P2 Port*/
#define P2_PORT_DF_VLAN_OFFSET                             0x0024
/*Same as P2_PORT_DF_VLAN_OFFSET*/
#define EMAC_ICSSG_SWITCH_PORT2_DEFAULT_VLAN_OFFSET        P2_PORT_DF_VLAN_OFFSET
/*VLAN-FID Table offset. 4096 VIDs. 2B per VID = 8KB = 0x2000*/
#define VLAN_STATIC_REG_TABLE_OFFSET                       0x0100
/*VLAN-FID Table offset for EMAC*/
#define EMAC_ICSSG_SWITCH_DEFAULT_VLAN_TABLE_OFFSET        VLAN_STATIC_REG_TABLE_OFFSET
/*packet descriptor Q reserved memory*/
#define PORT_DESC0_HI                                      0x2104
/*packet descriptor Q reserved memory*/
#define PORT_DESC0_LO                                      0x2F6C
/*packet descriptor Q reserved memory*/
#define PORT_DESC1_HI                                      0x3DD4
/*packet descriptor Q reserved memory*/
#define PORT_DESC1_LO                                      0x4C3C
/*packet descriptor Q reserved memory*/
#define HOST_DESC0_HI                                      0x5AA4
/*packet descriptor Q reserved memory*/
#define HOST_DESC0_LO                                      0x5F0C
/*packet descriptor Q reserved memory*/
#define HOST_DESC1_HI                                      0x6374
/*packet descriptor Q reserved memory*/
#define HOST_DESC1_LO                                      0x67DC
/*special packet descriptor Q reserved memory*/
#define HOST_SPPD0                                         0x7AAC
/*special packet descriptor Q reserved memory*/
#define HOST_SPPD1                                         0x7EAC
/*_Small_Description_*/
#define TIMESYNC_FW_WC_CYCLECOUNT_OFFSET                   0x83EC
/*IEP count hi roll over count*/
#define TIMESYNC_FW_WC_HI_ROLLOVER_COUNT_OFFSET            0x83F4
/*_Small_Description_*/
#define TIMESYNC_FW_WC_COUNT_HI_SW_OFFSET_OFFSET           0x83F8
/*Set clock descriptor*/
#define TIMESYNC_FW_WC_SETCLOCK_DESC_OFFSET                0x83FC
/*_Small_Description_*/
#define TIMESYNC_FW_WC_SYNCOUT_REDUCTION_FACTOR_OFFSET     0x843C
/*_Small_Description_*/
#define TIMESYNC_FW_WC_SYNCOUT_REDUCTION_COUNT_OFFSET      0x8440
/*_Small_Description_*/
#define TIMESYNC_FW_WC_SYNCOUT_START_TIME_CYCLECOUNT_OFFSET 0x8444
/*_Small_Description_*/
#define TIMESYNC_FW_WC_SYNCOUT_START_TIMESTAMP_OFFSET      0x844C
/*Control variable to generate SYNC1*/
#define TIMESYNC_FW_WC_ISOM_PIN_SIGNAL_EN_OFFSET           0x8454
/*SystemTime Sync0 periodicity*/
#define TIMESYNC_FW_ST_SYNCOUT_PERIOD_OFFSET               0x8458
/*pktTxDelay for P1 = link speed dependent p1 mac delay + p1 phy delay*/
#define TIMESYNC_FW_WC_PKTTXDELAY_P1_OFFSET                0x845C
/*pktTxDelay for P2 = link speed dependent p2 mac delay + p2 phy delay*/
#define TIMESYNC_FW_WC_PKTTXDELAY_P2_OFFSET                0x8460
/*Set clock operation done signal for next task*/
#define TIMESYNC_FW_SIG_PNFW_OFFSET                        0x8464
/*Set clock operation done signal for next task*/
#define TIMESYNC_FW_SIG_TIMESYNCFW_OFFSET                  0x8468

/* Memory Usage of
 * MSMC
 */

/* Memory Usage of
 * DMEM0
 */
/*New list is copied at this time*/
#define TAS_CONFIG_CHANGE_TIME                             0x000C
/*config change error counter*/
#define TAS_CONFIG_CHANGE_ERROR_COUNTER                    0x0014
/*TAS List update pending flag*/
#define TAS_CONFIG_PENDING                                 0x0018
/*TAS list update trigger flag*/
#define TAS_CONFIG_CHANGE                                  0x0019
/*List length for new TAS schedule*/
#define TAS_ADMIN_LIST_LENGTH                              0x001A
/*Currently active TAS list index*/
#define TAS_ACTIVE_LIST_INDEX                              0x001B
/*Cycle time for the new TAS schedule*/
#define TAS_ADMIN_CYCLE_TIME                               0x001C
/*Cycle counts remaining till the TAS list update*/
#define TAS_CONFIG_CHANGE_CYCLE_COUNT                      0x0020
/*Base Flow ID for sending packets to Host for Slice0*/
#define PSI_L_REGULAR_FLOW_ID_BASE_OFFSET                  0x0024
/*Same as PSI_L_REGULAR_FLOW_ID_BASE_OFFSET*/
#define EMAC_ICSSG_SWITCH_PSI_L_REGULAR_FLOW_ID_BASE_OFFSET PSI_L_REGULAR_FLOW_ID_BASE_OFFSET
/*Base Flow ID for sending mgmt and Tx TS to Host for Slice0*/
#define PSI_L_MGMT_FLOW_ID_OFFSET                          0x0026
/*Same as PSI_L_MGMT_FLOW_ID_OFFSET*/
#define EMAC_ICSSG_SWITCH_PSI_L_MGMT_FLOW_ID_BASE_OFFSET   PSI_L_MGMT_FLOW_ID_OFFSET
/*Queue number for Special packets written here*/
#define SPL_PKT_DEFAULT_PRIORITY                           0x0028
/*Express Preemptible Queue Mask*/
#define EXPRESS_PRE_EMPTIVE_Q_MASK                         0x0029
/*Port1/Port2 Default Queue number for untagged packets, only 1B is used*/
#define QUEUE_NUM_UNTAGGED                                 0x002A
/*Stores the table used for priority regeneration. 1B per PCP/Queue*/
#define PORT_Q_PRIORITY_REGEN_OFFSET                       0x002C
/*For marking packet as priority/express (this feature is disabled) or cut-through/S&F. One per slice*/
#define EXPRESS_PRE_EMPTIVE_Q_MAP                          0x0034
/*Stores the table used for priority mapping. 1B per PCP/Queue*/
#define PORT_Q_PRIORITY_MAPPING_OFFSET                     0x003C
/*Used to notify the FW of the current link speed*/
#define PORT_LINK_SPEED_OFFSET                             0x00A8
/*2k memory pointer reserved for default writes by PRU0*/
#define DEFAULT_MSMC_Q_OFFSET                              0x00AC
/*TAS gate mask for windows list0*/
#define TAS_GATE_MASK_LIST0                                0x0100
/*TAS gate mask for windows list1*/
#define TAS_GATE_MASK_LIST1                                0x0350
/*Memory to Enable/Disable Preemption on TX side*/
#define PRE_EMPTION_ENABLE_TX                              0x05A0
/*Active State of Preemption on TX side*/
#define PRE_EMPTION_ACTIVE_TX                              0x05A1
/*Memory to Enable/Disable Verify State Machine Preemption*/
#define PRE_EMPTION_ENABLE_VERIFY                          0x05A2
/*Verify Status of State Machine*/
#define PRE_EMPTION_VERIFY_STATUS                          0x05A3
/*Non Final Fragment Size supported by Link Partner*/
#define PRE_EMPTION_ADD_FRAG_SIZE_REMOTE                   0x05A4
/*Non Final Fragment Size supported by Firmware*/
#define PRE_EMPTION_ADD_FRAG_SIZE_LOCAL                    0x05A6
/*Time in ms the State machine waits for respond packet*/
#define PRE_EMPTION_VERIFY_TIME                            0x05A8
/*Memory used for R30 related management commands*/
#define MGR_R30_CMD_OFFSET                                 0x05AC
/*HW Buffer Pool0 base address*/
#define BUFFER_POOL_0_ADDR_OFFSET                          0x05BC
/*16B for Host Egress MSMC Q (Pre-emptible) context*/
#define HOST_RX_Q_PRE_CONTEXT_OFFSET                       0x0684
/*Buffer for 8 FDB entries to be added by 'Add Multiple FDB entries IOCTL*/
#define FDB_CMD_BUFFER                                     0x0894
/*TAS queue max sdu length list*/
#define TAS_QUEUE_MAX_SDU_LIST                             0x08FA
/*Used by FW to generate random number with the SEED value*/
#define HD_RAND_SEED_OFFSET                                0x0934
/*16B for Host Egress MSMC Q (Express) context*/
#define HOST_RX_Q_EXP_CONTEXT_OFFSET                       0x0940
/*DSCP enable/disable written here*/
#define DSCP_ENABLE_DISABLE_STATUS                         0x0A50
/*DSCP priority map is written here 8 dscp to 8 queues */
#define DSCP_BASED_PRI_MAP_INDEX_OFFSET                    0x0A51

/* Memory Usage of
 * DMEM1
 */
/*2k memory pointer reserved for default writes by PRU0*/
#define DEFAULT_MSMC_Q_OFFSET                              0x00AC
/*Used by FW to generate random number with the SEED value*/
#define HD_RAND_SEED_OFFSET                                0x0934

/* Memory Usage of
 * PA_STAT
 */
/*Start of 64 bits PA_STAT counters*/
#define PA_STAT_64b_START_OFFSET                           0x0000
/*Number of valid bytes sent by Rx PRU to Host on PSI. Currently disabled*/
#define NRT_HOST_RX_BYTE_COUNT_PASTATID                    0x0000
/*Number of valid bytes copied by RTU0 to Tx queues. Currently disabled*/
#define NRT_HOST_TX_BYTE_COUNT_PASTATID                    0x0002
/*Start of 32 bits PA_STAT counters*/
#define PA_STAT_32b_START_OFFSET                           0x0080
/*Number of valid packets sent by Rx PRU to Host on PSI*/
#define NRT_HOST_RX_PKT_COUNT_PASTATID                     0x0080
/*Number of valid packets copied by RTU0 to Tx queues*/
#define NRT_HOST_TX_PKT_COUNT_PASTATID                     0x0084
/*PRU diagnostic error counter which increments when RTU0 drops a locally injected packet due to port disabled or rule violation*/
#define NRT_RTU0_PACKET_DROPPED_SLICE0_PASTATID            0x0088
/*PRU diagnostic error counter which increments when RTU1 drops a locally injected packet due to port disabled or rule violation*/
#define NRT_RTU0_PACKET_DROPPED_SLICE1_PASTATID            0x008C
/*Port1 Tx Q Overflow Counters*/
#define NRT_PORT1_Q0_OVERFLOW_PASTATID                     0x0090
/*Port1 Tx Q Overflow Counters*/
#define NRT_PORT1_Q1_OVERFLOW_PASTATID                     0x0094
/*Port1 Tx Q Overflow Counters*/
#define NRT_PORT1_Q2_OVERFLOW_PASTATID                     0x0098
/*Port1 Tx Q Overflow Counters*/
#define NRT_PORT1_Q3_OVERFLOW_PASTATID                     0x009C
/*Port1 Tx Q Overflow Counters*/
#define NRT_PORT1_Q4_OVERFLOW_PASTATID                     0x00A0
/*Port1 Tx Q Overflow Counters*/
#define NRT_PORT1_Q5_OVERFLOW_PASTATID                     0x00A4
/*Port1 Tx Q Overflow Counters*/
#define NRT_PORT1_Q6_OVERFLOW_PASTATID                     0x00A8
/*Port1 Tx Q Overflow Counters*/
#define NRT_PORT1_Q7_OVERFLOW_PASTATID                     0x00AC
/*Port2 Tx Q Overflow Counters*/
#define NRT_PORT2_Q0_OVERFLOW_PASTATID                     0x00B0
/*Port2 Tx Q Overflow Counters*/
#define NRT_PORT2_Q1_OVERFLOW_PASTATID                     0x00B4
/*Port2 Tx Q Overflow Counters*/
#define NRT_PORT2_Q2_OVERFLOW_PASTATID                     0x00B8
/*Port2 Tx Q Overflow Counters*/
#define NRT_PORT2_Q3_OVERFLOW_PASTATID                     0x00BC
/*Port2 Tx Q Overflow Counters*/
#define NRT_PORT2_Q4_OVERFLOW_PASTATID                     0x00C0
/*Port2 Tx Q Overflow Counters*/
#define NRT_PORT2_Q5_OVERFLOW_PASTATID                     0x00C4
/*Port2 Tx Q Overflow Counters*/
#define NRT_PORT2_Q6_OVERFLOW_PASTATID                     0x00C8
/*Port2 Tx Q Overflow Counters*/
#define NRT_PORT2_Q7_OVERFLOW_PASTATID                     0x00CC
/*Host Tx Q Overflow Counters*/
#define NRT_HOST_Q0_OVERFLOW_PASTATID                      0x00D0
/*Host Tx Q Overflow Counters*/
#define NRT_HOST_Q1_OVERFLOW_PASTATID                      0x00D4
/*Host Tx Q Overflow Counters*/
#define NRT_HOST_Q2_OVERFLOW_PASTATID                      0x00D8
/*Host Tx Q Overflow Counters*/
#define NRT_HOST_Q3_OVERFLOW_PASTATID                      0x00DC
/*Host Tx Q Overflow Counters*/
#define NRT_HOST_Q4_OVERFLOW_PASTATID                      0x00E0
/*Host Tx Q Overflow Counters*/
#define NRT_HOST_Q5_OVERFLOW_PASTATID                      0x00E4
/*Host Tx Q Overflow Counters*/
#define NRT_HOST_Q6_OVERFLOW_PASTATID                      0x00E8
/*Host Tx Q Overflow Counters*/
#define NRT_HOST_Q7_OVERFLOW_PASTATID                      0x00EC
/*Host Egress Q (Pre-emptible) Overflow Counter*/
#define NRT_HOST_EGRESS_Q_PRE_OVERFLOW_PASTATID            0x00F0
/*Incremented if a packet is dropped at PRU0 because of a rule violation*/
#define NRT_DROPPED_PKT_SLICE0_PASTATID                    0x00F8
/*Incremented if a packet is dropped at PRU1 because of a rule violation*/
#define NRT_DROPPED_PKT_SLICE1_PASTATID                    0x00FC
/*Incremented if there was a CRC error or Min/Max frame error at PRU0*/
#define NRT_RX_ERROR_SLICE0_PASTATID                       0x0100
/*Incremented if there was a CRC error or Min/Max frame error at PRU1*/
#define NRT_RX_ERROR_SLICE1_PASTATID                       0x0104
/*RTU0 diagnostic counter increments when RTU detects Data Status invalid condition*/
#define RX_EOF_RTU_DS_INVALID_SLICE0_PASTATID              0x0108
/*RTU1 diagnostic counter increments when RTU detects Data Status invalid condition*/
#define RX_EOF_RTU_DS_INVALID_SLICE1_PASTATID              0x010C
/*Counter for packets dropped via NRT TX Port1*/
#define NRT_TX_PORT1_DROPPED_PACKET_PASTATID               0x0128
/*Counter for packets dropped via NRT TX Port2*/
#define NRT_TX_PORT2_DROPPED_PACKET_PASTATID               0x012C
/*Counter for packets with TS flag dropped via NRT TX Port1*/
#define NRT_TX_PORT1_TS_DROPPED_PACKET_PASTATID            0x0130
/*Counter for packets with TS flag dropped via NRT TX Port2*/
#define NRT_TX_PORT2_TS_DROPPED_PACKET_PASTATID            0x0134
/*PRU0 diagnostic error counter which increments when RX frame is dropped due to port is disabled*/
#define NRT_INF_PORT_DISABLED_SLICE0_PASTATID              0x0138
/*PRU1 diagnostic error counter which increments when RX frame is dropped due to port is disabled*/
#define NRT_INF_PORT_DISABLED_SLICE1_PASTATID              0x013C
/*PRU0 diagnostic error counter which increments when RX frame is dropped due to SA violation*/
#define NRT_INF_SAV_SLICE0_PASTATID                        0x0140
/*PRU1 diagnostic error counter which increments when RX frame is dropped due to SA violation*/
#define NRT_INF_SAV_SLICE1_PASTATID                        0x0144
/*PRU0 diagnostic error counter which increments when RX frame is dropped due to SA black listed*/
#define NRT_INF_SA_BL_SLICE0_PASTATID                      0x0148
/*PRU1 diagnostic error counter which increments when RX frame is dropped due to SA black listed*/
#define NRT_INF_SA_BL_SLICE1_PASTATID                      0x014C
/*PRU0 diagnostic error counter which increments when RX frame is dropped due to port blocked and not a special frame*/
#define NRT_INF_PORT_BLOCKED_SLICE0_PASTATID               0x0150
/*PRU1 diagnostic error counter which increments when RX frame is dropped due to port blocked and not a special frame*/
#define NRT_INF_PORT_BLOCKED_SLICE1_PASTATID               0x0154
/*PRU0 diagnostic error counter which increments when RX frame is dropped due to tagged*/
#define NRT_INF_AFT_DROP_TAGGED_SLICE0_PASTATID            0x0158
/*PRU1 diagnostic error counter which increments when RX frame is dropped due to tagged*/
#define NRT_INF_AFT_DROP_TAGGED_SLICE1_PASTATID            0x015C
/*PRU0 diagnostic error counter which increments when RX frame is dropped due to priority tagged*/
#define NRT_INF_AFT_DROP_PRIOTAGGED_SLICE0_PASTATID        0x0160
/*PRU1 diagnostic error counter which increments when RX frame is dropped due to priority tagged*/
#define NRT_INF_AFT_DROP_PRIOTAGGED_SLICE1_PASTATID        0x0164
/*PRU0 diagnostic error counter which increments when RX frame is dropped due to untagged*/
#define NRT_INF_AFT_DROP_NOTAG_SLICE0_PASTATID             0x0168
/*PRU1 diagnostic error counter which increments when RX frame is dropped due to untagged*/
#define NRT_INF_AFT_DROP_NOTAG_SLICE1_PASTATID             0x016C
/*PRU0 diagnostic error counter which increments when RX frame is dropped due to port not member of VLAN*/
#define NRT_INF_AFT_DROP_NOTMEMBER_SLICE0_PASTATID         0x0170
/*PRU1 diagnostic error counter which increments when RX frame is dropped due to port not member of VLAN*/
#define NRT_INF_AFT_DROP_NOTMEMBER_SLICE1_PASTATID         0x0174
/*PRU diagnostic error counter which increments when an entry couldn't be learned*/
#define NRT_FDB_NO_SPACE_TO_LEARN                          0x0178
/*PRU0 Bad fragment Error Counter*/
#define NRT_PREEMPT_BAD_FRAG_SLICE0_PASTATID               0x0180
/*PRU1 Bad fragment Error Counter*/
#define NRT_PREEMPT_BAD_FRAG_SLICE1_PASTATID               0x0184
/*PRU0 Fragment assembly Error Counter*/
#define NRT_PREEMPT_ASSEMBLY_ERROR_SLICE0_PASTATID         0x0188
/*PRU1 Fragment assembly Error Counter*/
#define NRT_PREEMPT_ASSEMBLY_ERROR_SLICE1_PASTATID         0x018C
/*PRU0 Fragment count in TX*/
#define NRT_PREEMPT_FRAG_COUNT_TX_SLICE0_PASTATID          0x0190
/*PRU1 Fragment count in TX*/
#define NRT_PREEMPT_FRAG_COUNT_TX_SLICE1_PASTATID          0x0194
/*PRU0 Assembly Completed*/
#define NRT_PREEMPT_ASSEMBLY_OK_SLICE0_PASTATID            0x0198
/*PRU1 Assembly Completed*/
#define NRT_PREEMPT_ASSEMBLY_OK_SLICE1_PASTATID            0x019C
/*PRU0 Fragments received*/
#define NRT_PREEMPT_FRAG_COUNT_RX_SLICE0_PASTATID          0x01A0
/*PRU1 Fragments received*/
#define NRT_PREEMPT_FRAG_COUNT_RX_SLICE1_PASTATID          0x01A4
/*PRU0 diagnostic error counter which increments if EOF task is scheduled without seeing RX_B1*/
#define RX_EOF_SHORT_FRAMEERR_SLICE0_PASTATID              0x01E8
/*PRU1 diagnostic error counter which increments if EOF task is scheduled without seeing RX_B1*/
#define RX_EOF_SHORT_FRAMEERR_SLICE1_PASTATID              0x01EC
/*PRU0 diagnostic counter which increments when frame if droped due to Early EOF received in B0*/
#define RX_B0_DROP_EARLY_EOF_SLICE0_PASTATID               0x0208
/*PRU1 diagnostic counter which increments when frame if droped due to Early EOF received in B0*/
#define RX_B0_DROP_EARLY_EOF_SLICE1_PASTATID               0x020C
/*Tx PRU0 diagnostic counter which increments when frame is cut off to prevent packet size > 2000B*/
#define TX_JUMBO_FRAME_CUTOFF_SLICE0_PASTATID              0x0210
/*Tx PRU1 diagnostic counter which increments when frame is cut off to prevent packet size > 2000B*/
#define TX_JUMBO_FRAME_CUTOFF_SLICE1_PASTATID              0x0214
/*Rx PRU0 diagnostic counter which increments when express frame is received in same queue as previous fragment*/
#define RX_EXPRESS_FRAG_Q_DROP_SLICE0_PASTATID             0x0238
/*Rx PRU1 diagnostic counter which increments when express frame is received in same queue as previous fragment*/
#define RX_EXPRESS_FRAG_Q_DROP_SLICE1_PASTATID             0x023C

#endif /* ____fw_mem_map_h*/
