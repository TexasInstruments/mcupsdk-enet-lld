#############################################################################
# Copyright 1996-2021 Synopsys, Inc.                                        #
#                                                                           #
# This Synopsys software and all associated documentation are proprietary   #
# to Synopsys, Inc. and may only be used pursuant to the terms and          #
# conditions of a written license agreement with Synopsys, Inc.             #
# All other use, reproduction, modification, or distribution of the         #
# Synopsys software or the associated documentation is strictly prohibited. #
#############################################################################
import sim
import sys
import os
import re
import sim_utils
import time

plat = "TDA5"
system = plat + "_System"
mcu = plat + "_SoC"
CPSW_NUSS = "/TDA5_System/TDA5_SoC/Main_Domain/NetworkSubsystem/CPSW_NUSS"
plat_name = "/" + system + "/" + mcu

if (sys.argv[1] == "A720_SS"):
    core_name = '/TDA5_System/TDA5_SoC/A720_SS/CLUSTER_0/cpu0'
    mem_name = plat_name + "/DRAM_0/m_memory"
    mem_start_address = int(0x80000000)
elif (sys.argv[1] == "R52P_SS_0"):
    core_name = '/TDA5_System/TDA5_SoC/R52P_SS/R52P_SS_0/CLUSTER_0/cpu0'
    mem_name = plat_name + "/RAM/m_memory"
    mem_start_address = int(0x60000000)
elif (sys.argv[1] == "R52P_SS_1"):
    core_name = '/TDA5_System/TDA5_SoC/R52P_SS/R52P_SS_1/CLUSTER_0/cpu0'
    mem_name = plat_name + "/RAM/m_memory"
    mem_start_address = int(0x60000000)
elif (sys.argv[1] == "R52P_SS_2"):
    core_name = '/TDA5_System/TDA5_SoC/R52P_SS/R52P_SS_2/CLUSTER_0/cpu0'
    mem_name = plat_name + "/RAM/m_memory"
    mem_start_address = int(0x60000000)
elif (sys.argv[1] == "R52P_SS_3"):
    core_name = '/TDA5_System/TDA5_SoC/R52P_SS/R52P_SS_3/CLUSTER_0/cpu0'
    mem_name = plat_name + "/RAM/m_memory"
    mem_start_address = int(0x60000000)
elif (sys.argv[1] == "M55_SS"):
    core_name ='/TDA5_System/TDA5_SoC/Main_Domain/Multimedia_drivers/M55_SS_0/CLUSTER_0/cpu0' 
    mem_name = plat_name + "/RAM/m_memory"
    mem_start_address = int(0x60000000)
    
    
#mem start address should be one of the memory address where variable gets created (TEST)


def send_ephy_message(eth_cmd_probe, dst_mac, src_mac):
    global eth0_cmd_probe
    #global eth1_cmd_probe
    global core_probe
    print("ETH_IO send_ephy_message called")
    packet = "0x45 0x08 0x00 0x78 0xb0 0x7a 0x40 0x00 0x40 0x06 0x41 0xb7 0x95 0xa5 0x79 0x2f 0xc4 0xc5 0x72 0xb4 0x2b 0x9f 0xa5 0xeb 0xd5 0x19 0xf0 0xb2 0x00 0x7c 0xd9 0x6e 0x59 0x02 0xef 0x24 0x5f 0x13 0x75 0xde 0xa2 0xf3 0xcb 0x33 0xae 0x37 0x93 0xf5 0xc3 0xe7 0xcd 0x28 0x25 0x5f 0xb8 0xa5 0x8b 0x9f 0xdf 0xb4 0xe9 0x97 0x9e 0xe7 0xf1 0xd6 0x1f 0xae 0x23 0xe5 0x37 0x63 0xbd 0x4e 0x8b 0x54 0x6b 0x99 0x40 0xd8 0x2b 0x5d 0x18 0x3b 0x3b 0x96 0x2b 0x7d 0xa8 0x43 0x2b 0xf3 0x39 0xf5 0xb6 0xca 0xcd 0x45 0x44 0x82 0x77 0x1a 0xf5 0x78 0xc4 0x70 0x89 0xa4 0xbf 0xcf 0x39 0xe1 0xa8 0x74 0x39 0x1d 0x79 0x9d 0xff 0x97 0x1a 0xe7 0x9e 0xe7"
#    eth0_cmd_probe.execute_command("send_message", ["ff:ff:ff:ff:ff:ff", "38:1b:b9:cc:58:e3", "false", "0", "0800", packet, "0x8A"])
#    eth0_cmd_probe.execute_command("send_message", ["ff:ff:ff:ff:ff:ff", "70:ff:76:1d:ec:f2", "false", "0", "0800", packet, "0x8A"])
#    eth0_cmd_probe.execute_command("send_message", ["70:ff:76:1d:ec:f4", "38:1b:b9:cc:58:e3", "false", "0", "0800", packet, "0x8A"])
    eth0_cmd_probe.execute_command("send_message", [dst_mac, src_mac, "false", "0", "0800", packet, "0x8A"])

def cb_stat_reg_cmd(observer, args):
    print("cb_stat_reg_cmd called")
    cpsw_cmd_probe = sim.CommandProcessorProbe(CPSW_NUSS)
    cpsw_cmd_probe.execute_command("stat_reg_cmd", ['0x3a200', '0x7FFFFFFD'])

def do_loopback():
    print("Looping back message received on enet port 1 to CPPI")
    eth0_cmd_probe.execute_command("send_packets_from_pcap", ["eth_0_rx.pcap"]);    

    
def ETH_MON_0_Rx_state(observer, args):
    ##p_is_loopback = core_probe.find_symbol_by_name("py_is_loopback",'object')
    ##p_is_loopback_mem_probe = sim.MemoryProbe(mem_name, p_is_loopback.size, p_is_loopback.start_address - mem_start_address)
    global eth0_cmd_probe
    print("ETH_MON_0_Rx_state called")
    io_state = eth0_cmd_probe.execute_command("get_current_phase")
    print(io_state)
    
    is_loopback = 0;
    if (io_state == "Receive"):
        print(io_state)
        if 0 == is_loopback:
            rx_mgs = eth0_cmd_probe.execute_command("get_message")
            send_ephy_message(None, None);
            send_ephy_message(None, None);
            send_ephy_message(None, None);
            send_ephy_message(None, None);
            send_ephy_message(None, None);
            print(rx_mgs)
        else:
            print("Doing loop-back")
            do_loopback();

def ETH_MON_1_Rx_state(observer, args):
    global eth1_cmd_probe
    print("ETH_MON_1_Rx_state called")
    io_state = eth1_cmd_probe.execute_command("get_current_phase")
    print(io_state)
    if (io_state == "Receive"):
        rx_mgs = eth1_cmd_probe.execute_command("get_message")
        send_ephy_message(eth1_cmd_probe, "70:ff:76:1d:ec:f2", "70:ff:76:1d:ec:01")
        
def ETH_MON_2_Rx_state(observer, args):
    global eth2_cmd_probe
    print("ETH_MON_2_Rx_state called")
    io_state = eth2_cmd_probe.execute_command("get_current_phase")
    print(io_state)
    if (io_state == "Receive"):
        rx_mgs = eth2_cmd_probe.execute_command("get_message")
        send_ephy_message(eth2_cmd_probe, "70:ff:76:1d:ec:f2", "70:ff:76:1d:ec:02")
        
def ETH_MON_3_Rx_state(observer, args):
    global eth3_cmd_probe
    print("ETH_MON_3_Rx_state called")
    io_state = eth3_cmd_probe.execute_command("get_current_phase")
    print(io_state)
    if (io_state == "Receive"):
        rx_mgs = eth3_cmd_probe.execute_command("get_message")
        send_ephy_message(eth3_cmd_probe, "70:ff:76:1d:ec:f2", "70:ff:76:1d:ec:03")

def ETH_MON_4_Rx_state(observer, args):
    global eth4_cmd_probe
    print("ETH_MON_4_Rx_state called")
    io_state = eth4_cmd_probe.execute_command("get_current_phase")
    print(io_state)
    if (io_state == "Receive"):
        rx_mgs = eth4_cmd_probe.execute_command("get_message")
        send_ephy_message(eth4_cmd_probe, "70:ff:76:1d:ec:f2", "70:ff:76:1d:ec:04")

def ETH_MON_5_Rx_state(observer, args):
    global eth5_cmd_probe
    print("ETH_MON_5_Rx_state called")
    io_state = eth5_cmd_probe.execute_command("get_current_phase")
    print(io_state)
    if (io_state == "Receive"):
        rx_mgs = eth5_cmd_probe.execute_command("get_message")
        send_ephy_message(eth5_cmd_probe, "70:ff:76:1d:ec:f2", "70:ff:76:1d:ec:05")

def ETH_MON_6_Rx_state(observer, args):
    global eth6_cmd_probe
    print("ETH_MON_6_Rx_state called")
    io_state = eth6_cmd_probe.execute_command("get_current_phase")
    print(io_state)
    if (io_state == "Receive"):
        rx_mgs = eth6_cmd_probe.execute_command("get_message")
        send_ephy_message(eth6_cmd_probe, "70:ff:76:1d:ec:f2", "70:ff:76:1d:ec:06")

def ETH_MON_7_Rx_state(observer, args):
    global eth7_cmd_probe
    print("ETH_MON_7_Rx_state called")
    io_state = eth7_cmd_probe.execute_command("get_current_phase")
    print(io_state)
    if (io_state == "Receive"):
        rx_mgs = eth7_cmd_probe.execute_command("get_message")
        send_ephy_message(eth7_cmd_probe, "70:ff:76:1d:ec:f2", "70:ff:76:1d:ec:07")
   
def ETH_compare_pkt(rx_mgs):
    print("EQoS -> ENET IO Stub Rx Packet not matched test failed")

 
try:
    print("#---In CPSW test Py script---#")

    if core_name != "":
       	core_probe = sim.CoreProbe(core_name)
    else:
       	raise StandardError("Core Name invalid")

    #stat_reg_cmd_probe = core_probe.find_symbol_by_name("_Z15py_stat_reg_cmdv", 'function')
    #observer_prb1 = core_probe.create_about_to_execute_instruction_observer(cb_stat_reg_cmd, stat_reg_cmd_probe.start_address, stat_reg_cmd_probe.start_address) 
    
    #func_probe1 = core_probe.find_symbol_by_name("_Z25inject_packet_from_ETH_IOv", 'function')
    #observer_prb = core_probe.create_about_to_execute_instruction_observer(send_ephy_message, func_probe1.start_address, func_probe1.start_address)
    obj0 = sim.MemoryContentObserver("/TDA5_System/CPSW_ENET/ETH_IO_1/ephy_msg_recv_trigger", 0, 0, ETH_MON_1_Rx_state, "write" )
    obj1 = sim.MemoryContentObserver("/TDA5_System/CPSW_ENET/ETH_IO_2/ephy_msg_recv_trigger", 0, 0, ETH_MON_2_Rx_state, "write" )
    obj2 = sim.MemoryContentObserver("/TDA5_System/CPSW_ENET/ETH_IO_3/ephy_msg_recv_trigger", 0, 0, ETH_MON_3_Rx_state, "write" )
    obj3 = sim.MemoryContentObserver("/TDA5_System/CPSW_ENET/ETH_IO_4/ephy_msg_recv_trigger", 0, 0, ETH_MON_4_Rx_state, "write" )
    obj4 = sim.MemoryContentObserver("/TDA5_System/CPSW_ENET/ETH_IO_5/ephy_msg_recv_trigger", 0, 0, ETH_MON_5_Rx_state, "write" )
    obj5 = sim.MemoryContentObserver("/TDA5_System/CPSW_ENET/ETH_IO_6/ephy_msg_recv_trigger", 0, 0, ETH_MON_6_Rx_state, "write" )
    obj6 = sim.MemoryContentObserver("/TDA5_System/CPSW_ENET/ETH_IO_7/ephy_msg_recv_trigger", 0, 0, ETH_MON_7_Rx_state, "write" )
    eth0_cmd_probe = sim.CommandProcessorProbe("/TDA5_System/CPSW_ENET/ETH_IO_0")
    eth1_cmd_probe = sim.CommandProcessorProbe("/TDA5_System/CPSW_ENET/ETH_IO_1")
    eth2_cmd_probe = sim.CommandProcessorProbe("/TDA5_System/CPSW_ENET/ETH_IO_2")
    eth3_cmd_probe = sim.CommandProcessorProbe("/TDA5_System/CPSW_ENET/ETH_IO_3")
    eth4_cmd_probe = sim.CommandProcessorProbe("/TDA5_System/CPSW_ENET/ETH_IO_4")
    eth5_cmd_probe = sim.CommandProcessorProbe("/TDA5_System/CPSW_ENET/ETH_IO_5")
    eth6_cmd_probe = sim.CommandProcessorProbe("/TDA5_System/CPSW_ENET/ETH_IO_6")
    eth7_cmd_probe = sim.CommandProcessorProbe("/TDA5_System/CPSW_ENET/ETH_IO_7")
 
       
except Exception as detail:
        print("Error: ",detail)

        sim.print_message("PY:- ERROR. CPSW test Py script.")

sim.suspend_script()