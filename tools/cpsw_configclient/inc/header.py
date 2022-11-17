"""
 #  Copyright (c) Texas Instruments Incorporated 2020
 #
 #  Redistribution and use in source and binary forms, with or without
 #  modification, are permitted provided that the following conditions
 #  are met:
 #
 #    Redistributions of source code must retain the above copyright
 #    notice, this list of conditions and the following disclaimer.
 #
 #    Redistributions in binary form must reproduce the above copyright
 #    notice, this list of conditions and the following disclaimer in the
 #    documentation and/or other materials provided with the
 #    distribution.
 #
 #    Neither the name of Texas Instruments Incorporated nor the names of
 #    its contributors may be used to endorse or promote products derived
 #    from this software without specific prior written permission.
 #
 #  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 #  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 #  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 #  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 #  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 #  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 #  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 #  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 #  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 #  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 #  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

"""
/* ========================================================================== */
/*                            Include Packages                                */
/* ========================================================================== */
"""
import serial
from ctypes import *
import ctypes
from json import JSONEncoder

"""
/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
"""
win = "unset"
error = "unset"
global_ip = "unset"
global_port = 5555
uart_port = "unset"
port = "unset"
files = ['','']

default_port_config = '''
{"portCfg":
[
{
"learningCfg":{"noLearn":0, "noSaUpdateEn":0},
"vlanCfg":{"vidIngressCheck":0,"dropUntagged":0,"dropDualVlan":0,"dropDoubleVlan":0},
"macModeCfg":{"macOnlyCafEn":0,"macOnlyEn":0},
"pvidCfg":{"vidIngressCheck":0,"limitIPNxtHdr":0,"disallowIPFrag":0,"noLearnMask":0,"forceUntaggedEgressMask":0,"regMcastFloodMask":511,"unregMcastFloodMask":511,"vlanMemberList":511,"vlanIdInfo":{"vlanId":0,"tagType":0}}
},
{
"learningCfg":{"noLearn":0, "noSaUpdateEn":0},
"vlanCfg":{"vidIngressCheck":0,"dropUntagged":0,"dropDualVlan":0,"dropDoubleVlan":0},
"macModeCfg":{"macOnlyCafEn":0,"macOnlyEn":0},
"pvidCfg":{"vidIngressCheck":0,"limitIPNxtHdr":0,"disallowIPFrag":0,"noLearnMask":0,"forceUntaggedEgressMask":0,"regMcastFloodMask":511,"unregMcastFloodMask":511,"vlanMemberList":511,"vlanIdInfo":{"vlanId":0,"tagType":0}}
},
{
"learningCfg":{"noLearn":0, "noSaUpdateEn":0},
"vlanCfg":{"vidIngressCheck":0,"dropUntagged":0,"dropDualVlan":0,"dropDoubleVlan":0},
"macModeCfg":{"macOnlyCafEn":0,"macOnlyEn":0},
"pvidCfg":{"vidIngressCheck":0,"limitIPNxtHdr":0,"disallowIPFrag":0,"noLearnMask":0,"forceUntaggedEgressMask":0,"regMcastFloodMask":511,"unregMcastFloodMask":511,"vlanMemberList":511,"vlanIdInfo":{"vlanId":0,"tagType":0}}
},
{
"learningCfg":{"noLearn":0, "noSaUpdateEn":0},
"vlanCfg":{"vidIngressCheck":0,"dropUntagged":0,"dropDualVlan":0,"dropDoubleVlan":0},
"macModeCfg":{"macOnlyCafEn":0,"macOnlyEn":0},
"pvidCfg":{"vidIngressCheck":0,"limitIPNxtHdr":0,"disallowIPFrag":0,"noLearnMask":0,"forceUntaggedEgressMask":0,"regMcastFloodMask":511,"unregMcastFloodMask":511,"vlanMemberList":511,"vlanIdInfo":{"vlanId":0,"tagType":0}}
},
{
"learningCfg":{"noLearn":0, "noSaUpdateEn":0},
"vlanCfg":{"vidIngressCheck":0,"dropUntagged":0,"dropDualVlan":0,"dropDoubleVlan":0},
"macModeCfg":{"macOnlyCafEn":0,"macOnlyEn":0},
"pvidCfg":{"vidIngressCheck":0,"limitIPNxtHdr":0,"disallowIPFrag":0,"noLearnMask":0,"forceUntaggedEgressMask":0,"regMcastFloodMask":511,"unregMcastFloodMask":511,"vlanMemberList":511,"vlanIdInfo":{"vlanId":0,"tagType":0}}
},
{
"learningCfg":{"noLearn":0, "noSaUpdateEn":0},
"vlanCfg":{"vidIngressCheck":0,"dropUntagged":0,"dropDualVlan":0,"dropDoubleVlan":0},
"macModeCfg":{"macOnlyCafEn":0,"macOnlyEn":0},
"pvidCfg":{"vidIngressCheck":0,"limitIPNxtHdr":0,"disallowIPFrag":0,"noLearnMask":0,"forceUntaggedEgressMask":0,"regMcastFloodMask":511,"unregMcastFloodMask":511,"vlanMemberList":511,"vlanIdInfo":{"vlanId":0,"tagType":0}}
},
{
"learningCfg":{"noLearn":0, "noSaUpdateEn":0},
"vlanCfg":{"vidIngressCheck":0,"dropUntagged":0,"dropDualVlan":0,"dropDoubleVlan":0},
"macModeCfg":{"macOnlyCafEn":0,"macOnlyEn":0},
"pvidCfg":{"vidIngressCheck":0,"limitIPNxtHdr":0,"disallowIPFrag":0,"noLearnMask":0,"forceUntaggedEgressMask":0,"regMcastFloodMask":511,"unregMcastFloodMask":511,"vlanMemberList":511,"vlanIdInfo":{"vlanId":0,"tagType":0}}
},
{
"learningCfg":{"noLearn":0, "noSaUpdateEn":0},
"vlanCfg":{"vidIngressCheck":0,"dropUntagged":0,"dropDualVlan":0,"dropDoubleVlan":0},
"macModeCfg":{"macOnlyCafEn":0,"macOnlyEn":0},
"pvidCfg":{"vidIngressCheck":0,"limitIPNxtHdr":0,"disallowIPFrag":0,"noLearnMask":0,"forceUntaggedEgressMask":0,"regMcastFloodMask":511,"unregMcastFloodMask":511,"vlanMemberList":511,"vlanIdInfo":{"vlanId":0,"tagType":0}}
},
{
"learningCfg":{"noLearn":0, "noSaUpdateEn":0},
"vlanCfg":{"vidIngressCheck":0,"dropUntagged":0,"dropDualVlan":0,"dropDoubleVlan":0},
"macModeCfg":{"macOnlyCafEn":0,"macOnlyEn":0},
"pvidCfg":{"vidIngressCheck":0,"limitIPNxtHdr":0,"disallowIPFrag":0,"noLearnMask":0,"forceUntaggedEgressMask":0,"regMcastFloodMask":511,"unregMcastFloodMask":511,"vlanMemberList":511,"vlanIdInfo":{"vlanId":0,"tagType":0}}
}
]
}
'''

host_stats_list = [

    "Number of Good Frames",
    "Number of Good Broadcast Frames",
    "Number of Good Multicast Frames",
    "Reserved4",
    "Number of CRC errors received",
    "Reserved6",
    "Number of oversized frames received",
    "Reserved8",
    "Number of undersized frames received",
    "Number fo fragmented frames received",
    "Number of frames dropped by the ALE",
    "Number of overrun frames dropped by the ALE",
    "Number of received bytes in good frames",
    "Number of good frames transmitted",
    "Number of good broadcast frames transmitted",
    "Number of good multicast frames transmitted",
    "Reserved17",
    "Reserved18",
    "Reserved19",
    "Reserved20",
    "Reserved21",
    "Reserved22",
    "Reserved23",
    "Reserved24",
    "Reserved25",
    "Number of bytes in all good frames transmitted",
    "Number of 64-byte frames received and transmitted",
    "Number of frames of size 65 to 127 bytes received and trasmitted ",
    "Number of frames of size 128 to 255 bytes received and trasmitted",
    "Number of frames of size 256 to 511 bytes received and trasmitted",
    "Number of frames of size 512 to 1023 bytes received and trasmitted",
    "Number of frames of size 1024 or greater transmitted",
    "Number of bytes received and transmitted",
    "Receive bottom of FIFO drop",
    "Number of dropped frames received due to portmask",
    "Receive top of FIFO drop",
    "Number of frames dropped due to ALE rate limiting",
    "Number of dropped frames due to ALE VID ingress",
    "Number of dropped frames due to DA = SA",
    "Number of dropped frames due to ALE block mode",
    "Number of dropped frames due to ALE secure mode",
    "Number of dropped frames due to ALE authentication",
    "ALE receive unknown unicast",
    "ALE receive unknown unicast bytecount",
    "ALE receive unknown multicast",
    "ALE receive unknown multicast bytecount",
    "ALE receive unknown broadcast",
    "ALE receive unknown broadcast bytecount",
    "ALE policer matched",
    "ALE policer matched and condition red",
    "ALE policer matched and condition yellow",
    "ALE multicast source address drop",
    "ALE dual VLAN drop",
    "ALE IEEE 802.3 length error drop",
    "ALE IP next header limit drop",
    "ALE IPv4 fragment drop",
    "Reserved57",
    "Reserved58",
    "Reserved59",
    "Reserved60",
    "Reserved61",
    "Reserved62",
    "Reserved63",
    "Reserved64",
    "Reserved65",
    "Reserved66",
    "Reserved67",
    "Reserved68",
    "Reserved69",
    "Reserved70",
    "Reserved71",
    "Reserved72",
    "Reserved73",
    "Reserved74",
    "Reserved75",
    "Reserved76",
    "Reserved77",
    "Reserved78",
    "Reserved79",
    "Reserved80",
    "IET receive assembly error",
    "IET receive assembly OK",
    "IET receive SMD error",
    "IET recieve merge fragment count",
    "IET transmit merge fragment count",
    "IET transmit merge hold count",
    "Reserved87",
    "Reserved88",
    "Reserved89",
    "Reserved90",
    "Reserved91",
    "Reserved92",
    "Reserved93",
    "Reserved94",
    "Reserved95",
    "Transmit memory protect CRC error",
    "Reserved97",
    "Reserved98",
    "Reserved99",
    "Reserved100",
    "Reserved101",
    "Reserved102",
    "Reserved103",
    "Reserved104",
    "Reserved105",
    "Reserved106",
    "Reserved107",
    "Reserved108",
    "Reserved109",
    "Reserved110",
    "Reserved111",
    "Reserved112",
    "Reserved113",
    "Reserved114",
    "Reserved115",
    "Reserved116",
    "Reserved117",
    "Reserved118",
    "Reserved119",
    "Reserved120",
    "Reserved121",
    "Reserved122",
    "Reserved123",
    "Reserved124",
    "Reserved125",
    "Reserved126",
    "Reserved127",
    "Reserved128"
]

macport_stats_list = [

    "Number of Good Frames",
    "Number of Good Broadcast Frames",
    "Number of Good Multicast Frames",
    "Number of pause frames received",
    "Number of CRC errors received",
    "Number of alignment/code errors received",
    "Number of oversized frames received",
    "Number of jabber frames received",
    "Number of undersized frames received",
    "Number fo fragmented frames received",
    "Number of frames dropped by the ALE",
    "Number of overrun frames dropped by the ALE",
    "Number of received bytes in good frames",
    "Number of good frames transmitted",
    "Number of good broadcast frames transmitted",
    "Number of good multicast frames transmitted",
    "Number of pause frames transmitted",
    "Number of deferred frames transmitted",
    "Number of transmitted frames experiencing a collsion",
    "Number of transmitted frames experiencing a single collision",
    "Number of transmitted frames experiencing multiple collisions",
    "Number of transmitted frames abandoned due to excessive collisions",
    "Number of transmitted frames abandoned due to a late collision",
    "Number of receive inter-packet gap errors (10G only)",
    "Number of transmitted frames that experienced a carrier loss",
    "Number of bytes in all good frames transmitted",
    "Number of 64-byte frames received and transmitted",
    "Number of frames of size 65 to 127 bytes received and trasmitted ",
    "Number of frames of size 128 to 255 bytes received and trasmitted",
    "Number of frames of size 256 to 511 bytes received and trasmitted",
    "Number of frames of size 512 to 1023 bytes received and trasmitted",
    "Number of frames of size 1024 or greater transmitted",
    "Number of bytes received and transmitted",
    "Receive bottom of FIFO drop",
    "Number of dropped frames received due to portmask",
    "Receive top of FIFO drop",
    "Number of frames dropped due to ALE rate limiting",
    "Number of dropped frames due to ALE VID ingress",
    "Number of dropped frames due to DA = SA",
    "Number of dropped frames due to ALE block mode",
    "Number of dropped frames due to ALE secure mode",
    "Number of dropped frames due to ALE authentication",
    "ALE receive unknown unicast",
    "ALE receive unknown unicast bytecount",
    "ALE receive unknown multicast",
    "ALE receive unknown multicast bytecount",
    "ALE receive unknown broadcast",
    "ALE receive unknown broadcast bytecount",
    "ALE policer matched",
    "ALE policer matched and condition red",
    "ALE policer matched and condition yellow",
    "ALE multicast source address drop",
    "ALE dual VLAN drop",
    "ALE IEEE 802.3 length error drop",
    "ALE IP next header limit drop",
    "ALE IPv4 fragment drop",
    "Reserved57",
    "Reserved58",
    "Reserved59",
    "Reserved60",
    "Reserved61",
    "Reserved62",
    "Reserved63",
    "Reserved64",
    "Reserved65",
    "Reserved66",
    "Reserved67",
    "Reserved68",
    "Reserved69",
    "Reserved70",
    "Reserved71",
    "Reserved72",
    "Reserved73",
    "Reserved74",
    "Reserved75",
    "Reserved76",
    "Reserved77",
    "Reserved78",
    "Reserved79",
    "Reserved80",
    "IET receive assembly error",
    "IET receive assembly OK",
    "IET receive SMD error",
    "IET recieve merge fragment count",
    "IET transmit merge fragment count",
    "IET transmit merge hold count",
    "Reserved87",
    "Reserved88",
    "Reserved89",
    "Reserved90",
    "Reserved91",
    "Reserved92",
    "Reserved93",
    "Reserved94",
    "Reserved95",
    "Transmit memory protect CRC error",
    "1 Ethernet port priority packet count",
    "2 Ethernet port priority packet count",
    "3 Ethernet port priority packet count",
    "4 Ethernet port priority packet count",
    "5 Ethernet port priority packet count",
    "6 Ethernet port priority packet count",
    "7 Ethernet port priority packet count",
    "8 Ethernet port priority packet count",
    "1 Ethernet port priority packet byte count",
    "2 Ethernet port priority packet byte count",
    "3 Ethernet port priority packet byte count",
    "4 Ethernet port priority packet byte count",
    "5 Ethernet port priority packet byte count",
    "6 Ethernet port priority packet byte count",
    "7 Ethernet port priority packet byte count",
    "8 Ethernet port priority packet byte count",
    "1 Ethernet port priority packet drop count",
    "2 Ethernet port priority packet drop count",
    "3 Ethernet port priority packet drop count",
    "4 Ethernet port priority packet drop count",
    "5 Ethernet port priority packet drop count",
    "6 Ethernet port priority packet drop count",
    "7 Ethernet port priority packet drop count",
    "8 Ethernet port priority packet drop count",
    "1 Ethernet port priority packet drop byte count",
    "2 Ethernet port priority packet drop byte count",
    "3 Ethernet port priority packet drop byte count",
    "4 Ethernet port priority packet drop byte count",
    "5 Ethernet port priority packet drop byte count",
    "6 Ethernet port priority packet drop byte count",
    "7 Ethernet port priority packet drop byte count",
    "8 Ethernet port priority packet drop byte count"
]
"""
/* ========================================================================== */
/*                           Constants & Typedefs                             */
/* ========================================================================== */
"""

def BUFFER_SIZE():
    return 1024

__version__ = '1.0.0'

"""
/* ========================================================================== */
/*                            Class Declarations                              */
/* ========================================================================== */
"""

# *****************************************************************************
#   Class CDataJSONEncoder
#
#   A variation of JSONEncoder that supports serializing ctypes structures
# *****************************************************************************
class CDataJSONEncoder(JSONEncoder):
    def default(self, obj):
        if isinstance(obj, (ctypes.Array, list)):
            return [self.default(e) for e in obj]

        if isinstance(obj, ctypes._Pointer):
            return self.default(obj.contents) if obj else None

        if isinstance(obj, ctypes._SimpleCData):
            return self.default(obj.value)

        if isinstance(obj, (bool, int, float, str)):
            return obj

        if obj is None:
            return obj

        if isinstance(obj, (Structure, Union)):
            result = {}
            anonymous = getattr(obj, '_anonymous_', [])

            for key, *_ in getattr(obj, '_fields_', []):
                value = getattr(obj, key)

                # private fields don't encode
                if key.startswith('_'):
                    continue

                if key in anonymous:
                    result.update(self.default(value))
                else:
                    result[key] = self.default(value)

            return result

        return JSONEncoder.default(self, obj)


class Enet_IoctlPrms(Structure):
    _fields_ = [("inArgs",c_void_p),("inArgsSize",c_uint32),("outArgs",c_void_p),("outArgsSize",c_uint32)]

class Cpsw_RemoteConfig(Structure):
    _fields_ = [("cmd",c_uint32),("prms",Enet_IoctlPrms)]


# *****************************************************************************
#                        ENET_PER_IOCTL_GET_VERSION
# *****************************************************************************

class Cpsw_Version(Structure):
    _fields_ = [("major",c_uint32),("minor",c_uint32),("rtl",c_uint32),("id",c_uint32)]

class EnetMod_Version(Structure):
    _fields_ = [("major",c_uint32),("minor",c_uint32),("rtl",c_uint32),("id",c_uint32),("scheme",c_uint32),("bu",c_uint32)]

class Enet_Version(Structure):
    _fields_ = [("ss",Cpsw_Version),("cpsw",Cpsw_Version),("ale",Cpsw_Version),("cpts",Cpsw_Version),("mdio",EnetMod_Version)]
    

# *****************************************************************************
#                   CPSW_ALE_IOCTL_GET_BCAST_MCAST_LIMIT
# *****************************************************************************

class CpswAle_PortBcastMcastRateLimitParams(Structure):
    _fields_ = [("portNum",c_uint32),("bcastRateLimitForPortEn",c_bool),("mcastRateLimitForPortEn",c_bool),("bcastLimitNumPktsPerSec",c_uint32),("mcastLimitNumPktsPerSec",c_uint32)]

class CpswAle_GetBcastMcastRateLimitOutArgs(Structure):
    _fields_ = [("rateLimitEn", c_bool),("rateLimitAtTxPort",c_bool),("numPorts",c_uint32),("portPrms",ARRAY(CpswAle_PortBcastMcastRateLimitParams,2))]
    
# *****************************************************************************
#                       CPSW_ALE_IOCTL_LOOKUP_UCAST
# *****************************************************************************


class CpswAle_macAddrInfo(Structure):
    _fields_ = [("addr",ARRAY(c_uint8,6)),("vlanId",c_uint32)]

class Cpsw_lookupUnicastRemoteConfig(Structure):
    _fields_ = [("cmd",c_uint32),("inArgs", CpswAle_macAddrInfo)]
    
# *****************************************************************************
#                    ENET_STATS_IOCTL_GET_HOSTPORT_STATS
# *****************************************************************************

class CpswStats_PortStats(Structure):
    _fields_ = [("val",ARRAY(c_uint64,128))]

#class EnetPer_PortLinkCfg(Structure):
#    _fields_ = [("portNum",Enet_MacPort),("interface",EnetMacPort_Interface),("linkCfg",EnetMacPort_LinkCfg),("macCfg",CpswMacPort_Cfg),("phyCfg",EnetPhy_Cfg)]
#
#class Enet_MacPort(Structure):
#    _fields_ = [("portNum", Enet_MacPort)]
#    
#class Cpsw_SetHostPortTrafficShapingInArgs(Structure):
#    _fields_ = [("trafficShapingParams",CpswHostPort_SetTrafficShapingInArgs),("txBlocksRem",c_uint32)]
##
##class Cpsw_VlanCfg(Structure):
##    _fields_ = [("vlanAware",ctyoe),(,),(,),(,)]
##
