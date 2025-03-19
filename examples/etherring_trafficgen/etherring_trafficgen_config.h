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

#ifndef ETHER_RING_LWIP_ETHERRING_CONFIG_H_
#define ETHER_RING_LWIP_ETHERRING_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================*/
/*                           Macros and Constants                             */
/*============================================================================*/
/* Number of ClassA streams enabled */
#define ENETAPP_NUM_CLASSA_STREAMS 3

/* Maximum Number of ClassA Streams supported */
#define ENETAPP_MAX_CLASSA_STREAMS 3

/* ClassA Packet Payload Length */
#define ENETAPP_CLASSA_PAYLOAD_LENGTH 500U

/* Number of nodes present in Ether-Ring */
#define ENETAPP_MAX_NODES_IN_RING 4

/* Flag to enable/disable LWIP Background traffic */
#define ENETAPP_ENABLE_TCP_BG_TRAFFIC


#ifdef __cplusplus
}
#endif
#endif /* ETHER_RING_LWIP_ETHERRING_CONFIG_H_ */
