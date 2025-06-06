#############################################################################
# Copyright 1996-2023 Synopsys, Inc.                                        #
#                                                                           #
# This Synopsys software and all associated documentation are proprietary   #
# to Synopsys, Inc. and may only be used pursuant to the terms and          #
# conditions of a written license agreement with Synopsys, Inc.             #
# All other use, reproduction, modification, or distribution of the         #
# Synopsys software or the associated documentation is strictly prohibited. #
#############################################################################

import sim
import sys, os.path
import inspect

script_dir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
script_dir = script_dir + "/../shared/sim_shared"

if script_dir not in sys.path:
    sys.path.append(script_dir)
    
import bootcode_utils
        
core_name = [
    "AutoHSM_M55",
    "DM_M55",
    "RTSS_0_R52P_Core0",
    "RTSS_0_R52P_Core1",
    "RTSS_1_R52P_Core0",
    "RTSS_1_R52P_Core1",
    "RTSS_2_R52P_Core0",
    "RTSS_2_R52P_Core1",
    "Main_M55_0",
    "Main_M55_1",
    "Main_M55_2"
    "Main_M55_3"
    "Main_M55_4"
    "A720_Core0",
    "A720_Core1",
    "A720_Core2",
    "A720_Core3",
    "A720_Core4",
    "A720_Core5",
    "A720_Core6",
    "A720_Core7",
    "C7x_0",
    "C7x_1",
    "C7x_2",
    "C7x_3",
]

#By default Bootcode running on ROT puts all core in reset except ROT.
#User can make necessary code out of reset using configure_core_reset_state() api
#configure_core_reset_state(core_name, vector_table_addr)   
#Example
#First argument is core name, find the core name from the core_name list
#Second argument is vector table addr/Reset handler address
    

bootcode_utils.configure_core_out_of_reset("Main_M55_0", 0x78000000)       



