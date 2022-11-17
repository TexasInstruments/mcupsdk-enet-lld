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

cmd_list = ["display_version", "display_mcast_bcast_limits", "lookup_unicast", "hostport_stats",
 "macport_stats", "add_vlanentry", "display_cpuload", "enable_disable_mcast" ,
 "enable_sw_intervlan_routing", "enable_hw_intervlan_routing", "ip_nxt_hdr_whitelisting", "rate_limiting"]


schema_1 = {
  "title": "Command 1: Display Versions",
  "type": "object",
  "properties": {
    "cmd": 
    {
      "description": "Unique Command ID",
      "type": "integer"
    },
  },
  "required": [ "cmd" ]
}

schema_2 = {
  "title": "Command 2: Display Multicast and Broadcast Limits",
  "type": "object",
  "properties": {
    "cmd": 
    {
      "description": "Unique Command ID",
      "type": "integer"
    },
  },
  "required": [ "cmd" ]
}

schema_3 = {
  "title": "Command 3: Lookup Unicast",
  "type": "object",
  "properties": {
    "cmd": 
    {
      "description": "Unique Command ID",
      "type": "integer"
    },
    "addr": 
    {
      "description": "MAC Addr 0",
      "type": "string",
      "pattern": "^([0-9A-Fa-f]{2}[:-]){5}([0-9A-Fa-f]{2})$"
    },
    "vlanId": 
    {
      "description": "VLAN ID",
      "type": "integer"
    },

  },
  "required": [ "cmd", "addr", "vlanId" ]
}

schema_4 = {
  "title": "Command 4: Display Host Port Statistics",
  "type": "object",
  "properties": {
    "cmd": 
    {
      "description": "Unique Command ID",
      "type": "integer"
    },
  },
  "required": [ "cmd" ]
}

schema_5 = {
  "title": "Command 5: Display MAC Port Statistics",
  "type": "object",
  "properties": {
    "cmd": 
    {
      "description": "Unique Command ID",
      "type": "integer"
    },
    "portNum": 
    {
      "description": "MAC Port Number",
      "type": "integer"
    },

  },
  "required": [ "cmd" , "portNum" ]
}

schema_6 = {
  "title": "Command 6: Add VLAN Entry",
  "type": "object",
  "properties": {
    "cmd": 
    {
      "description": "Unique Command ID",
      "type": "integer"
    },
    "vlanId": 
    {
      "description": "VLAN ID",
      "type": "integer"
    },
    "vlanMemberMask": 
    {
      "description": "VLAN Member Mask",
      "type": "integer"
    },
    "isEnable": 
    {
      "description": "Enable/Disable flag",
      "type": "integer"
    },    
  },
  "required": [ "cmd", "vlanId", "vlanMemberMask", "isEnable"]
}

schema_7 = {
  "title": "Command 7: Display CPU Load",
  "type": "object",
  "properties": {
    "cmd": 
    {
      "description": "Unique Command ID",
      "type": "integer"
    },
  },
  "required": [ "cmd" ]
}

schema_8 = {
  "title": "Command 8: Enable/Disable Multicast",
  "type": "object",
  "properties": {
    "cmd": 
    {
      "description": "Unique Command ID",
      "type": "integer"
    },
    "addr": 
    {
      "description": "MAC Addr 0",
      "type": "string",
      "pattern": "^([0-9A-Fa-f]{2}[:-]){5}([0-9A-Fa-f]{2})$"
    },
    "vlanId": 
    {
      "description": "VLAN ID",
      "type": "integer"
    },
    "vlanMemberMask": 
    {
      "description": "VLAN Member Mask",
      "type": "integer"
    },
    "isEnable": 
    {
      "description": "Enable/Disable flag",
      "type": "integer"
    },                                    
  },
  "required": [ "cmd", "addr", "isEnable" ]
}

schema_9 = {
  "title": "Command 9: Enable Software InterVLAN routing",
  "type": "object",
  "properties": {
    "cmd": 
    {
      "description": "Unique Command ID",
      "type": "integer"
    },
    "src_mac_addr": 
    {
      "description": "MAC Addr 0",
      "type": "string",
      "pattern": "^([0-9A-Fa-f]{2}[:-]){5}([0-9A-Fa-f]{2})$"
    },
    "src_IpAddr": 
    {
      "description": "IP Addr 0",
      "type": "string",
      "pattern": "^(([0-9]|[1-9][0-9]|1[0-9]{2}|2[0-4][0-9]|25[0-5])\.){3}([0-9]|[1-9][0-9]|1[0-9]{2}|2[0-4][0-9]|25[0-5])$"
    },
    "ing_vlanId": 
    {
      "description": "Ingress VLAN ID",
      "type": "integer"
    },
    "ing_portNum": 
    {
      "description": "Ingress Port Num",
      "type": "integer"
    },
    
    "dst_mac_addr": 
    {
      "description": "MAC Addr 0",
      "type": "string",
      "pattern": "^([0-9A-Fa-f]{2}[:-]){5}([0-9A-Fa-f]{2})$"
    },
    "dst_IpAddr": 
    {
      "description": "IP Addr 0",
      "type": "string",
      "pattern": "^(([0-9]|[1-9][0-9]|1[0-9]{2}|2[0-4][0-9]|25[0-5])\.){3}([0-9]|[1-9][0-9]|1[0-9]{2}|2[0-4][0-9]|25[0-5])$"
    },
    "egr_vlanId": 
    {
      "description": "Egress VLAN ID",
      "type": "integer"
    },
    "egr_portNum": 
    {
      "description": "Ingress Port Num",
      "type": "integer"
    },


  },
  "required": [ "cmd", "src_mac_addr", "src_IpAddr", "ing_vlanId", "ing_portNum","dst_mac_addr", "dst_IpAddr", "egr_vlanId", "egr_portNum"]
}

schema_10 = {
  "title": "Command 10: Enable Hardware InterVLAN routing",
  "type": "object",
  "properties": {
    "cmd": 
    {
      "description": "Unique Command ID",
      "type": "integer"
    },
    "src_mac_addr": 
    {
      "description": "MAC Addr 0",
      "type": "string",
      "pattern": "^([0-9A-Fa-f]{2}[:-]){5}([0-9A-Fa-f]{2})$"
    },
    "src_IpAddr": 
    {
      "description": "IP Addr 0",
      "type": "string",
      "pattern": "^(([0-9]|[1-9][0-9]|1[0-9]{2}|2[0-4][0-9]|25[0-5])\.){3}([0-9]|[1-9][0-9]|1[0-9]{2}|2[0-4][0-9]|25[0-5])$"
    },
    "ing_vlanId": 
    {
      "description": "Ingress VLAN ID",
      "type": "integer"
    },
    "ing_portNum": 
    {
      "description": "Ingress Port Num",
      "type": "integer"
    },
    
    "dst_mac_addr": 
    {
      "description": "MAC Addr 0",
      "type": "string",
      "pattern": "^([0-9A-Fa-f]{2}[:-]){5}([0-9A-Fa-f]{2})$"
    },
    "dst_IpAddr": 
    {
      "description": "IP Addr 0",
      "type": "string",
      "pattern": "^(([0-9]|[1-9][0-9]|1[0-9]{2}|2[0-4][0-9]|25[0-5])\.){3}([0-9]|[1-9][0-9]|1[0-9]{2}|2[0-4][0-9]|25[0-5])$"
    },
    "egr_vlanId": 
    {
      "description": "Egress VLAN ID",
      "type": "integer"
    },
    "egr_portNum": 
    {
      "description": "Ingress Port Num",
      "type": "integer"
    },


  },
  "required": [ "cmd",
   "src_mac_addr", "src_IpAddr", "ing_vlanId", "ing_portNum",
   "dst_mac_addr", "dst_IpAddr", "egr_vlanId", "egr_portNum"]
}

schema_11 = {
  "title": "Command 11: IP Next Header based filtering",
  "type": "object",
  "properties": {
    "cmd": 
    {
      "description": "Unique Command ID",
      "type": "integer"
    },
    "vlanId": 
    {
      "description": "VLAN ID",
      "type": "integer"
    },
    "vlanMemberMask": 
    {
      "description": "VLAN Member Mask",
      "type": "integer"
    },
  },
  "required": [ "cmd", "vlanId", "vlanMemberMask"]
}

schema_12 = {
  "title": "Command 12: Enable Rate Limiting",
  "type": "object",
  "properties": {
    "cmd": 
    {
      "description": "Unique Command ID",
      "type": "integer"
    },
    "src_mac_addr": 
    {
      "description": "MAC Addr 0",
      "type": "string",
      "pattern": "^([0-9A-Fa-f]{2}[:-]){5}([0-9A-Fa-f]{2})$"
    },
    "ing_portNum": 
    {
      "description": "Ingress Port Num",
      "type": "integer"
    },    
    "dst_mac_addr": 
    {
      "description": "MAC Addr 0",
      "type": "string",
      "pattern": "^([0-9A-Fa-f]{2}[:-]){5}([0-9A-Fa-f]{2})$"
    },
    "egr_portNum": 
    {
      "description": "Ingress Port Num",
      "type": "integer"
    },
    "egr_portNum": 
    {
      "description": "Ingress Port Num",
      "type": "integer"
    },
    "PIR": 
    {
      "description": "Peak Information Rate in bps",
      "type": "integer"
    },
    "CIR": 
    {
      "description": "Committed Information Rate in bps",
      "type": "integer"
    },


  },
  "required": [ "cmd", "src_mac_addr", "ing_portNum",
  "dst_mac_addr", "egr_portNum",
  "PIR", "CIR"]
}

schemas = [schema_1, schema_2, schema_3, schema_4, schema_5, schema_6, schema_7, schema_8, schema_9, schema_10, schema_11, schema_12]

cmd_format_list = [
    ',{"cmd":"display_version"}',
    ',{"cmd":"display_mcast_bcast_limits"}',
    ',{"cmd":"lookup_unicast",\n "addr": ,\n"vlanId":\n}',
    ',{"cmd":"hostport_stats"}',
    ',{"cmd":"macport_stats",\n"portNum":\n}',
    ',{"cmd":"add_vlanentry",\n"vlanId":,\n"vlanMemberMask":,\n"isEnable":\n}',
    ',{"cmd":"display_cpuload"}',
    ',{"cmd":"enable_disable_mcast",\n"addr":,\n"vlanId":,\n"vlanMemberMask":,\n"isEnable":\n}',
    ',{"cmd":"enable_sw_intervlan_routing",\n"src_mac_addr":,\n"src_IpAddr":,\n"ing_vlanId":,\n"ing_portNum":,\n"dst_mac_addr":,\n"dst_IpAddr":,\n"egr_vlanId":,\n"egr_portNum":\n}',
    ',{"cmd":"enable_hw_intervlan_routing",\n"src_mac_addr":,\n"src_IpAddr":,\n"ing_vlanId":,\n"ing_portNum":,\n"dst_mac_addr":,\n"dst_IpAddr":,\n"egr_vlanId":,\n"egr_portNum":\n}',
    ',{"cmd":"ip_nxt_hdr_whitelisting",\n"vlanId":,\n"vlanMemberMask":\n}',
    ',{"cmd":"rate_limiting",\n"src_mac_addr":,\n"ing_portNum":,\n"dst_mac_addr":,\n"egr_portNum":,\n"PIR":,\n"CIR":\n}'
    
]
