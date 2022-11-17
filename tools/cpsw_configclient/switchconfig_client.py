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

from PyQt5 import QtWidgets, uic, QtGui
from PyQt5.QtWidgets import QFileDialog
import sys
import getopt
import os
import time
import threading
import json
from pathlib import Path
from jsonschema import validate

"""
/* ========================================================================== */
/*                           Global Variables                                 */
/* ========================================================================== */
"""
curr_file_path = str(Path().absolute())
ui_files_path = curr_file_path + '//ui//'
src_files_path = curr_file_path + '//src//'
inc_files_path = curr_file_path + '//inc//'

"""
/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
"""
# Inserting folders to system path
sys.path.insert(0, ui_files_path)
sys.path.insert(0, src_files_path)
sys.path.insert(0, inc_files_path)

from header import *
from header import CDataJSONEncoder
from ser_des import *
from schemas import *

"""
/* ========================================================================== */
/*                           Function Declarations                            */
/* ========================================================================== */
"""


# *****************************************************************************
#   Function display_version
#
#   Handler for button "Version" in Main UI. 
#   It creates a C configuration structure with appropriate values, 
#   passes it to serializer layer, and sends the response to GUI.
# *****************************************************************************
def display_version():

    # Windows created
    global ver
    global error

    # Populate Structure
    prms = Enet_IoctlPrms() 
    config = Cpsw_RemoteConfig()
    
    prms.inArgs = None
    prms.inArgsSize = 0
    prms.outArgs = None
    prms.outArgsSize = 0
    config.cmd = 1
    config.prms = prms

    # Get Media 
    chosen_media = win.media_comboBox.currentText()
    if (chosen_media == "Network" and (not check_ip(global_ip))):
        load_window("ipError")
        return
    if (chosen_media == "UART" and (port == "unset")):
        load_window("Error")
        return

    # Serialize and Send
    status = serialize(config, 1, chosen_media, global_ip, port)
    if (status == 0):
        load_window("Error")
        return
     
    # Receive and Deserialize
    status = deserialize(chosen_media)
    if (status == 0):
        load_window("Error")
        return
    received_struct = status

    try:
        reported_error = received_struct["Error"]
    except:
        # Send back to GUI
        ver = uic.loadUi(ui_files_path + "version.ui")
        ver.versionTable.setItem(0,0,QtWidgets.QTableWidgetItem(str(hex(received_struct["cpsw"]["major"]))))
        ver.versionTable.setItem(1,0,QtWidgets.QTableWidgetItem(str(hex(received_struct["cpsw"]["minor"]))))
        ver.versionTable.setItem(2,0,QtWidgets.QTableWidgetItem(str(hex(received_struct["cpsw"]["id"]))))
        ver.versionTable.setItem(3,0,QtWidgets.QTableWidgetItem(str(hex(received_struct["cpsw"]["rtl"]))))
        ver.versionTable.setItem(0,1,QtWidgets.QTableWidgetItem(str(hex(received_struct["ale"]["major"]))))
        ver.versionTable.setItem(1,1,QtWidgets.QTableWidgetItem(str(hex(received_struct["ale"]["minor"]))))
        ver.versionTable.setItem(2,1,QtWidgets.QTableWidgetItem(str(hex(received_struct["ale"]["id"]))))
        ver.versionTable.setItem(3,1,QtWidgets.QTableWidgetItem(str(hex(received_struct["ale"]["rtl"]))))
        ver.versionTable.setItem(0,2,QtWidgets.QTableWidgetItem(str(hex(received_struct["cpts"]["major"]))))
        ver.versionTable.setItem(1,2,QtWidgets.QTableWidgetItem(str(hex(received_struct["cpts"]["minor"]))))
        ver.versionTable.setItem(2,2,QtWidgets.QTableWidgetItem(str(hex(received_struct["cpts"]["id"]))))
        ver.versionTable.setItem(3,2,QtWidgets.QTableWidgetItem(str(hex(received_struct["cpts"]["rtl"]))))
        ver.versionTable.setItem(0,3,QtWidgets.QTableWidgetItem(str(hex(received_struct["ss"]["major"]))))
        ver.versionTable.setItem(1,3,QtWidgets.QTableWidgetItem(str(hex(received_struct["ss"]["minor"]))))
        ver.versionTable.setItem(2,3,QtWidgets.QTableWidgetItem(str(hex(received_struct["ss"]["id"]))))
        ver.versionTable.setItem(3,3,QtWidgets.QTableWidgetItem(str(hex(received_struct["ss"]["rtl"]))))
        ver.versionTable.setItem(0,4,QtWidgets.QTableWidgetItem(str(hex(received_struct["mdio"]["major"]))))
        ver.versionTable.setItem(1,4,QtWidgets.QTableWidgetItem(str(hex(received_struct["mdio"]["minor"]))))
        ver.versionTable.setItem(2,4,QtWidgets.QTableWidgetItem(str(hex(received_struct["mdio"]["id"]))))
        ver.versionTable.setItem(3,4,QtWidgets.QTableWidgetItem(str(hex(received_struct["mdio"]["rtl"]))))
        ver.versionTable.setItem(4,4,QtWidgets.QTableWidgetItem(str(hex(received_struct["mdio"]["scheme"]))))
        ver.versionTable.setItem(5,4,QtWidgets.QTableWidgetItem(str(hex(received_struct["mdio"]["bu"]))))
        ver.show()
        return
    
    load_window("ioctlError")
    error.ioctl_error_textBrowser.setText(str(reported_error))
    return

# *****************************************************************************
#   Function display_bcast_mcast
#
#   Handler for "Bcast/Mcast" button in Main UI. 
#   It creates a C configuration structure, passes it to the serializer 
#   layer, and sends the response to the GUI.
# *****************************************************************************
def display_bcast_mcast():
    
    # Windows created
    global error
    global cast
    
    # Populate Stucture
    config = dict()
    config["cmd"] = 2    

    # Get Media
    chosen_media = win.media_comboBox.currentText()
    if (chosen_media == "Network" and (not check_ip(global_ip))):
        load_window("ipError")
        return
    if (chosen_media == "UART" and (port == "unset")):
        load_window("Error")
        return

    # Serialize and send
    status = serialize(config, 1, chosen_media, global_ip, port)
    if (status == 0):
        load_window("Error")
        return
     
    # Receive and deserialize
    status = deserialize(chosen_media)
    if (status == 0):
        load_window("Error")
        return
    received_struct = status
    try:
        reported_error = received_struct["Error"]
    except:
        # Send back to GUI
        cast = uic.loadUi(ui_files_path + "cast.ui") 
        cast.rateLimitEn_label.setText(str(received_struct["rateLimitEn"]))
        cast.rateLimitAtTxPort_label.setText(str(received_struct["rateLimitAtTxPort"]))
        cast.numPorts_label.setText(str(received_struct["numPorts"]))
        
        for i in range(2):
            cast.limit_table.setItem(i,0,QtWidgets.QTableWidgetItem(str(received_struct["portPrms"][i]["portNum"])))
            cast.limit_table.setItem(i,1,QtWidgets.QTableWidgetItem(str(received_struct["portPrms"][i]["bcastRateLimitForPortEn"])))
            cast.limit_table.setItem(i,2,QtWidgets.QTableWidgetItem(str(received_struct["portPrms"][i]["mcastRateLimitForPortEn"])))
            cast.limit_table.setItem(i,3,QtWidgets.QTableWidgetItem(str(received_struct["portPrms"][i]["bcastLimitNumPktsPerSec"])))
            cast.limit_table.setItem(i,4,QtWidgets.QTableWidgetItem(str(received_struct["portPrms"][i]["mcastLimitNumPktsPerSec"])))

        if (received_struct["numPorts"] == 9):
            for i in range(2,9):
                numRows = cast.limit_table.rowCount()
                cast.limit_table.insertRow(numRows)

                cast.limit_table.setItem(i,0,QtWidgets.QTableWidgetItem(str(received_struct["portPrms"][i]["portNum"])))
                cast.limit_table.setItem(i,1,QtWidgets.QTableWidgetItem(str(received_struct["portPrms"][i]["bcastRateLimitForPortEn"])))
                cast.limit_table.setItem(i,2,QtWidgets.QTableWidgetItem(str(received_struct["portPrms"][i]["mcastRateLimitForPortEn"])))
                cast.limit_table.setItem(i,3,QtWidgets.QTableWidgetItem(str(received_struct["portPrms"][i]["bcastLimitNumPktsPerSec"])))
                cast.limit_table.setItem(i,4,QtWidgets.QTableWidgetItem(str(received_struct["portPrms"][i]["mcastLimitNumPktsPerSec"])))

        cast.show()
        return
    
    load_window("ioctlError")
    error.ioctl_error_textBrowser.setText(str(reported_error))
    return

# *****************************************************************************
#   Function display_lookup_unicast
#
#   Handler for "Lookup Unicast" form in Main UI. 
#   It creates a C configuration structure based on form in GUI, passes it 
#   to the serializer layer, and sends the response to the GUI.
# *****************************************************************************
def display_lookup_unicast():
    
    # Windows created
    global error
    global lookup_unicast 
    
    # Check Input
    temp_addr = []
    temp_addr.append(win.lineEdit.text())
    temp_addr.append(win.lineEdit_2.text())
    temp_addr.append(win.lineEdit_3.text())
    temp_addr.append(win.lineEdit_4.text())
    temp_addr.append(win.lineEdit_5.text())
    temp_addr.append(win.lineEdit_6.text())
    vlanId = win.lineEdit_7.text()

    if ((not check_mac(temp_addr)) or (not check_int(vlanId))):
        load_window("inputError")
        return

    # Populate Structure
    config = dict()

    config["cmd"] = 3

    config["addr_0"] = int(temp_addr[0],16)
    config["addr_1"] = int(temp_addr[1],16)
    config["addr_2"] = int(temp_addr[2],16)
    config["addr_3"] = int(temp_addr[3],16)
    config["addr_4"] = int(temp_addr[4],16)
    config["addr_5"] = int(temp_addr[5],16)

    config["vlanId"] = int(vlanId)

    # Get Media
    chosen_media = win.media_comboBox.currentText()
    if (chosen_media == "Network" and (not check_ip(global_ip))):
        load_window("ipError")
        return
    if (chosen_media == "UART" and (port == "unset")):
        load_window("Error")
        return

    # Serialize and send
    status = serialize(config, 1, chosen_media, global_ip, port)
    if (status == 0):
        load_window("Error")
        return
    
    # Receive and deserialize
    status = deserialize(chosen_media)
    if (status == 0):
        load_window("Error")
        return
    received_struct = status
    try:
        reported_error = received_struct["Error"]
    except:
        # Send back to GUI
        lookup_unicast = uic.loadUi(ui_files_path + "lookup_unicast.ui")
        lookup_unicast.touched_label.setText(str(received_struct["touched"]))
        lookup_unicast.aleEntryIndex_label.setText(str(received_struct["aleEntryIndex"]))
        lookup_unicast.portNum_label.setText(str(received_struct["unicastInfo"]["portNum"]))
        lookup_unicast.blocked_label.setText(str(received_struct["unicastInfo"]["blocked"]))
        lookup_unicast.secure_label.setText(str(received_struct["unicastInfo"]["secure"]))
        lookup_unicast.super_label.setText(str(received_struct["unicastInfo"]["super"]))
        lookup_unicast.ageable_label.setText(str(received_struct["unicastInfo"]["ageable"]))
        lookup_unicast.trunk_label.setText(str(received_struct["unicastInfo"]["trunk"]))
        lookup_unicast.show()
        return
    
    load_window("ioctlError")
    error.ioctl_error_textBrowser.setText(str(reported_error))
    return

# *****************************************************************************
#   Function display_stats
#
#   Handler for "Show" button in Main UI's Stats Tab. 
#   It creates a C configuration structure based on selection in GUI, passes it 
#   to the serializer layer, and sends the response to the GUI.
# *****************************************************************************
def display_stats():
    
    # Windows created
    global error

    port_num = win.statsPort_comboBox.currentText()
    
    # Populate Structure
    config = dict()
    
    if (port_num == "Host"):
        config["cmd"] = 4
    else:
        config["cmd"] = 5
        config["portNum"] = int(port_num[4:]) -1
        print(config["cmd"])
    
    # Get Media
    chosen_media = win.media_comboBox.currentText()
    if (chosen_media == "Network" and (not check_ip(global_ip))):
        load_window("ipError")
        return
    if (chosen_media == "UART" and (port == "unset")):
        load_window("Error")
        return

    # Serialize and send
    status = serialize(config, 0, chosen_media, global_ip, port)
    if (status == 0):
        load_window("Error")
        return
    
    # Receive and deserialize
    status = deserialize(chosen_media)
    if (status == 0):
        load_window("Error")
        return
    received_struct = status
    try:
        reported_error = received_struct["Error"]
    except:
        count = 0
        # Send back to GUI
        win.stats_textBrowser.setText("")
        win.stats_textBrowser_2.setText("")
        for i in range(128):
            num = received_struct["val"][i]
            if (not num ==0):
                count = count + 1
                if (config["cmd"]==4):
                    if (host_stats_list[i][0:8] == "Reserved"):
                        continue
                    win.stats_textBrowser.append(host_stats_list[i] + ":")
                elif (config["cmd"] == 5):
                    if (macport_stats_list[i][0:8] == "Reserved"):
                        continue
                    win.stats_textBrowser.append(macport_stats_list[i] + ": ")
                win.stats_textBrowser_2.append(str(num))
        if (not count):
            load_window("cpswTypeError")
        return

    load_window("ioctlError")
    error.ioctl_error_textBrowser.setText(str(reported_error))
    return

# *****************************************************************************
#   Function dump_table
#
#   Handler for "Dump Table" button in Main UI's ALE Tab. 
#   It creates a C configuration structure, passes it 
#   to the serializer layer, and sends the response to the GUI.
# *****************************************************************************
def dump_table():
    
    # Windows Created
    global error

    # Populate Structure
    config = Cpsw_RemoteConfig()
    prms = Enet_IoctlPrms()
    
    prms.inArgs = None
    prms.inArgsSize = 0
    prms.outArgs = None
    prms.outArgsSize = 0
    
    config.cmd = 0
    config.prms = prms

    # Get Media 
    chosen_media = win.media_comboBox.currentText()
    if (chosen_media == "Network" and (not check_ip(global_ip))):
        load_window("ipError")
        return
    if (chosen_media == "UART" and (port == "unset")):
        load_window("Error")
        return

    # Serialize and send
    status = serialize(config, 1, chosen_media, global_ip, port)
    if (status == 0):
        load_window("Error")
        return

    # Receive and deserialize
    status = deserialize(chosen_media)
    if (status == 0):
        load_window("Error")
        return
    received_struct = status
    try:
        reported_error = received_struct["Error"]
    except:
        # Send back to GUI
        win.dumpTable_textBrowser.setText(str(received_struct))
        return

    load_window("ioctlError")
    error.ioctl_error_textBrowser.setText(str(reported_error))
    return

# *****************************************************************************
#   Function add_vlan
#
#   Handler for Add VLAN "Submit" button in Main UI's Configure Tab. 
#   It creates a C configuration structure, passes it 
#   to the serializer layer, and sends the response to the GUI.
# *****************************************************************************
def add_vlan():
    
    # Windows Created
    global error
    
    # Check Input
    vlanId = win.addVlan_vlanID_lineEdit.text()
    vlanMemberMask = win.addVlan_portMask_lineEdit.text()

    if ((not check_int(vlanId)) or (not check_int(vlanMemberMask))):
        load_window("inputError")
        return

    # Populate Structure
    config = dict()
    config["cmd"] = 6
    config["vlanId"] = int(vlanId)
    config["vlanMemberMask"] = int(vlanMemberMask)
    if (win.addVlan_enabled_comboBox.currentText() == "True"):
        config["isEnable"] = 1
    else:
        config["isEnable"] = 0

    # Get Media 
    chosen_media = win.media_comboBox.currentText()
    if (chosen_media == "Network" and (not check_ip(global_ip))):
        load_window("ipError")
        return
    if (chosen_media == "UART" and (port == "unset")):
        load_window("Error")
        return

    # Serialize and send
    status = serialize(config, 0, chosen_media, global_ip, port)
    if (status == 0):
        load_window("Error")
        return

    # Receive and deserialize
    status = deserialize(chosen_media)
    if (status == 0):
        load_window("Error")
        return
    received_struct = status
    try:
        reported_error = received_struct["Error"]
    except:
        # Send back to GUI
        win.addVlan_aleEntryIndex_textBrowser.setText(str(received_struct["aleEntryIndex"]))
        return
    
    load_window("ioctlError")
    error.ioctl_error_textBrowser.setText(str(reported_error))
    return

# *****************************************************************************
#   Function enable_disable_mcast
#
#   Handler for Enable/Disable Mcast "Submit" button in Main UI's Configure Tab. 
#   It creates a C configuration structure, passes it 
#   to the serializer layer, and sends the response to the GUI.
# *****************************************************************************
def enable_disable_mcast():

    # Windows Created
    global error
    
    # Check Input
    temp_addr = []
    temp_addr.append(win.mcast_add1_lineEdit.text())
    temp_addr.append(win.mcast_add2_lineEdit.text())
    temp_addr.append(win.mcast_add3_lineEdit.text())
    temp_addr.append(win.mcast_add4_lineEdit.text())
    temp_addr.append(win.mcast_add5_lineEdit.text())
    temp_addr.append(win.mcast_add6_lineEdit.text())
    vlanId = win.mcast_vlanID_lineEdit.text()
    vlanMemberMask = win.mcast_portMask_lineEdit.text()

    if ((not check_mac(temp_addr)) or (not check_int(vlanId)) or (not check_int(vlanMemberMask))):
        load_window("inputError")
        return

    # Populate Structure
    config = dict()
    config["cmd"] = 8
    config["addr_0"] = int(temp_addr[0],16)
    config["addr_1"] = int(temp_addr[1],16)
    config["addr_2"] = int(temp_addr[2],16)
    config["addr_3"] = int(temp_addr[3],16)
    config["addr_4"] = int(temp_addr[4],16)
    config["addr_5"] = int(temp_addr[5],16)

    config["vlanId"] = int(vlanId)
    config["vlanMemberMask"] = int(vlanMemberMask)
    if (win.mcast_enabled_comboBox.currentText() == "True"):
        config["isEnable"] = 1
    else:
        config["isEnable"] = 0

    # Get Media 
    chosen_media = win.media_comboBox.currentText()
    if (chosen_media == "Network" and (not check_ip(global_ip))):
        load_window("ipError")
        return
    if (chosen_media == "UART" and (port == "unset")):
        load_window("Error")
        return

    # Serialize and send
    status = serialize(config, 0, chosen_media, global_ip, port)
    if (status == 0):
        load_window("Error")
        return

    # Receive and deserialize
    status = deserialize(chosen_media)
    if (status == 0):
        load_window("Error")
        return
    received_struct = status
    try:
        reported_error = received_struct["Error"]
    except:
        # Send back to GUI
        win.mcast_aleEntryIndex_textBrowser.setText(str(received_struct["aleEntryIndex"]))
        return
    
    load_window("ioctlError")
    error.ioctl_error_textBrowser.setText(str(reported_error))
    return

# *****************************************************************************
#   Function get_cmdValue
#
#   Checks if entered cmd matches any of the valid commands present 
#   in the schemas file and returns the value of the command
# *****************************************************************************
def get_cmdValue(cmd):
    ret = False
    for count, item in enumerate(cmd_list, 1):
        if (item == cmd):
            ret = count
            break
    return ret

# *****************************************************************************
#   Function format_matches
#
#   Checks if temp_dict has a format matching with any of the dicts in
#   header file
# *****************************************************************************
def format_matches(temp_dict):
    cmd = temp_dict["cmd"]
    cmdValue = get_cmdValue(cmd)

    if (cmdValue == False):
        ret = False
    else:
        temp_dict["cmd"] = cmdValue
        ret = True

        try:    
            validate(instance=temp_dict, schema=schemas[cmdValue - 1])  
        except:
            ret = False
            print("Validate failed")
    return ret

# *****************************************************************************
#   Function load_file_config
#
#   Loads content from file onto screen
# *****************************************************************************
def load_config_file():
    global files
    files = QFileDialog.getOpenFileName()
    
    if (not files[0] == ''):
        win.file_config_textBrowser.setText("")
        file = open(files[0], "r")
        for line in file:
            if (line[len(line)-1] == '\n'):
                win.file_config_textBrowser.append(line[:-1]) # Ignore newline
            else:
                win.file_config_textBrowser.append(line)
        file.close()
        win.file_status_label.setText("File Open")

# *****************************************************************************
#   Function load_file_config
#
#   Loads content from file onto screen
# *****************************************************************************
def write_config_file():
    global files
    global error

    files = QFileDialog.getSaveFileName()
    
    if (files[0] == ''):
        load_window("Error")
        return

    file = open(files[0],"w")
    currText = win.file_config_textBrowser.toPlainText()
    file.write(currText)
    file.close()

# *****************************************************************************
#   Function cli_send_config_file
#
#   Sends the config contained in the container to serializer and puts the
#   output on the screen
# *****************************************************************************
def cli_send_config_file(cfgfile):

    global error

    text = ''
    file = open(cfgfile, "r")
    for line in file:
        text = text + line
    file.close()

    if text[0] == ',':
        text = text[1:len(text)]

    json_input = '[' + text + ']'
    json_config_list = json.loads(json_input)
    
    format_match = True
    for i in range(len(json_config_list)):
        if (not format_matches(json_config_list[i])):
            format_match = False
        else :
            print(json_config_list[i]["cmd"])

    # Get Media
    if (chosen_media == "Network" and (not check_ip(global_ip))):
        print("Ip Error")
        return
    if (chosen_media == "UART" and (port == "unset")):
        print("UART Error")
        return
    
    # Serialize and Deserialize one-by-one
    for i in range(len(json_config_list)):
        status = serialize(json_config_list[i], 0, chosen_media, global_ip, port)
        if (status == 0):
            print("Error")
            return
        try:
            status = deserialize(chosen_media)
        except Exception as e:
            if (status == 0):
                print("Error")
                return
        else :
            print("Configuration updated successfully")      
    get_cpuload()

# *****************************************************************************
#   Function send_config_file
#
#   Sends the config contained in the container to serializer and puts the
#   output on the screen
# *****************************************************************************
def send_config_file():

    global error

    is_ok = parser()
    if not is_ok:
        load_window("syntaxError")
        return

    text = win.file_config_textBrowser.toPlainText()
    print(text)
    if text[0] == ',':
        text = text[1:len(text)]

    json_input = '[' + text + ']'
    json_config_list = json.loads(json_input)
    
    format_match = True
    for i in range(len(json_config_list)):
        if (not format_matches(json_config_list[i])):
            format_match = False
        else :
            print(json_config_list[i]["cmd"])

    # Get Media
    chosen_media = win.media_comboBox.currentText()
    if (chosen_media == "Network" and (not check_ip(global_ip))):
        load_window("ipError")
        return
    if (chosen_media == "UART" and (port == "unset")):
        load_window("Error")
        return
    
    # Serialize and Deserialize one-by-one
    for i in range(len(json_config_list)):
        status = serialize(json_config_list[i], 0, chosen_media, global_ip, port)
        if (status == 0):
            load_window("Error")
            return
        status = deserialize(chosen_media)
        if (status == 0):
            load_window("Error")
            return
        win.file_output_textBrowser.append(str(status))
        
# *****************************************************************************
#   Function parser
#
#   Parses the text on window for errors in json format
# *****************************************************************************
def parser():
    
    global error

    text = win.file_config_textBrowser.toPlainText()

    if len(text)==0:
        return 0 
    else:
        if text[0] == ',':
            text = text[1:len(text)]
        
    json_input = '[' + text + ']'
    try:
        json_config_list = json.loads(json_input)
    except:
        print("json.loads Failed")
        return 0

    format_match = True
    for i in range(len(json_config_list)):
        if (not format_matches(json_config_list[i])):
            format_match = False
    if (format_match):
        return 1
    else:
        return 0

# *****************************************************************************
#   Function get_ip
#
#   Sets value of global_ip based on entered text in Main UI
# *****************************************************************************
def get_ip():

    # Variable
    global global_ip
    
    # Window opened
    global error
    
    # Check and assign IP
    entered_ip = win.IP_lineEdit.text()
    if (check_ip(entered_ip)):
        global_ip = entered_ip
    else:
        load_window("ipError")
        return

    # media = win.media_comboBox.currentText()
    
    if (global_ip=="unset"):
        win.ip_top_label.setText('0.0.0.0')
    else:
        win.ip_top_label.setText(global_ip)
    # win.media_top_label.setText(media)
    win.ip_submit_label.setText("IP Set.")
    get_cpuload()

# *****************************************************************************
#   Function get_cpu load
#
#   Gets the value of CPU load
# *****************************************************************************
def get_cpuload():

    global chosen_media
    global fifo
    # Get CPU Load from Target
    # Populate Structure
    config = dict()
    config["cmd"] = 7

    if cli == False:
        # Get Media
        chosen_media = win.media_comboBox.currentText()
        if (chosen_media == "Network" and (not check_ip(global_ip))):
            load_window("ipError")
            return
        if (chosen_media == "UART" and (port == "unset")):
            load_window("Error")
            return

    # Serialize and send
    status = serialize(config, 0, chosen_media, global_ip, port)

    # Receive and deserialize
    status = deserialize(chosen_media)

    received_struct = status

    if(received_struct != 0):
        try:
            reported_error = received_struct["Error"]
        except:
            if cli == False:
                win.progressBar.setValue(int(received_struct["load"]))
            else :
                if pipe_flag == True:
                    try:
                        fifo = open(fifo_path, "w")
                    except Exception as e:
                        print (e)   
                    fifo.write(str(received_struct["load"]))
                else:
                    print("R5 Load: ", received_struct["load"])

    if (close_flag == False):            
        threading.Timer(1, get_cpuload).start()
    
# Print CPU Load


# *****************************************************************************
#   Function get_uart
#
#   Sets value of port based on entered text in Main UI
# *****************************************************************************
def get_uart():

    # Variable
    global uart_port
    global port

    # Window opened
    global error
    
    # Check UART connection
    entered_uart = win.UART_lineEdit.text()
    try:
        port = serial.Serial(entered_uart, 115200, timeout = 1)
        uart_port = entered_uart
        print("PORT IS: ", port)
    except:
        load_window("Error")
        return

    win.UART_top_label.setText(uart_port)
    win.UART_submit_label.setText("UART Set.")

# *****************************************************************************
#   Function enable UART console
#
# *****************************************************************************
def enable_uart_console():

    uart_thread = threading.Thread(target = uart_print)
    uart_thread.start()
    
# *****************************************************************************
#   Function check_int
#
#   Checks if val is integer or not
# *****************************************************************************
def check_int(val):
    try:
        int(val)
    except:
        return False
    return True

# *****************************************************************************
#   Function check_mac
#
#   Checks value of addr based on Mac Format
# *****************************************************************************
def check_mac(addr):
    ret = True
    for i in range(6):
        if addr[i] == "":
            return False
        try:
            num = int(addr[i])
        except:
            ret = ret and (addr[i].lower()<='ff' and addr[i].lower()>='aa')
    return ret

# *****************************************************************************
#   Function check_ip
#
#   Checks value of global_ip based on IPv4 formatting
# *****************************************************************************
def check_ip(ip):
    length = len(ip)
    if (length<7 or length>15):
        return 0
    
    dot_index = []
    for i in range(length):
        if ip[i] == '.':
            dot_index.append(i)
    numDots = len(dot_index)
    if (not numDots==3):
        return 0

    for i in range(numDots+1):
        if (i == 0):
            start = 0
            end = dot_index[0]
        elif (i == 1):
            start = dot_index[0] + 1
            end = dot_index[1]
        elif (i == 2):
            start = dot_index[1] + 1
            end = dot_index[2]
        elif (i == 3):
            start = dot_index[2] + 1
            end = length

        try:
            number = int(ip[start:end])
            if (number<0 or number>255):
                return 0
        except:
            return 0
    return 1

# *****************************************************************************
#   Function uart_print

#   Prints to UART tab of Main UI from UART port
# *****************************************************************************
def uart_print():
    while(True):
        rcv = port.read(1)
        if (rcv):
            try:
                rcv = rcv.decode('utf-8')
                pass_flag = 1
            except:
                print ("string is not UTF-8")
                pass_flag = 0                
            if not rcv == '':
                if pass_flag == 1:
                    win.uart_textBrowser.insertPlainText(rcv)

        
# *****************************************************************************
#   Function init_cpsw
#
#   Creates a python dictionary based on CPSW Init values entered in Main UI, 
#   and passes the structure to the serialization layer 
# *****************************************************************************
def init_cpsw():
    
    # Windows created
    global error
    global port

    # Initialize empty dictionary
    # CpswInit_dict = json.loads(win.portCfg_initPrms_textBrowser.toPlainText())
    CpswInit_dict = dict()

    # Populate dictionary
    CpswInit_dict["ale_modeFlag"] = int(win.ale_modeFlag_lineEdit.text())

    CpswInit_dict["ale_macAddr_1"] = int(win.ale_macAddr_lineEdit_1.text(),16)

    CpswInit_dict["ale_macAddr_2"] = int(win.ale_macAddr_lineEdit_2.text(),16)

    CpswInit_dict["ale_macAddr_3"] = int(win.ale_macAddr_lineEdit_3.text(),16)

    CpswInit_dict["ale_macAddr_4"] = int(win.ale_macAddr_lineEdit_4.text(),16)

    CpswInit_dict["ale_macAddr_5"] = int(win.ale_macAddr_lineEdit_5.text(),16)

    CpswInit_dict["ale_macAddr_6"] = int(win.ale_macAddr_lineEdit_6.text(),16)

    if (win.ale_policingEn_comboBox.currentText() == "True"):
        CpswInit_dict["ale_policingEn"] = 1
    else:
        CpswInit_dict["ale_policingEn"] = 0

    if (win.ale_yellowDropEn_comboBox.currentText() == "True"):
        CpswInit_dict["ale_yellowDropEn"] = 1
    else:
        CpswInit_dict["ale_yellowDropEn"] = 0

    if (win.ale_redDropEn_comboBox.currentText() == "True"):
        CpswInit_dict["ale_redDropEn"] = 1
    else:
        CpswInit_dict["ale_redDropEn"] = 0

    CpswInit_dict["ale_yellowThreshold"] = int(win.ale_yellowThreshold_lineEdit.text())

    CpswInit_dict["ale_noMatchMode"] = int(win.ale_noMatchMode_lineEdit.text())

    CpswInit_dict["ale_peakRate"] = int(win.ale_peakRate_lineEdit.text())

    CpswInit_dict["ale_commitRate"] = int(win.ale_commitRate_lineEdit.text())

    if (win.ale_autoAgingEn_comboBox.currentText() == "True"):
        CpswInit_dict["ale_autoAgingEn"] = 1
    else:
        CpswInit_dict["ale_autoAgingEn"] = 0

    CpswInit_dict["ale_agingPeriod"] = int(win.ale_agingPeriod_lineEdit.text())

    if (win.ale_vlanAware_comboBox.currentText() == "True"):
        CpswInit_dict["ale_vlanAware"] = 1
    else:
        CpswInit_dict["ale_vlanAware"] = 0

    if (win.cpsw_vlanAware_comboBox.currentText() == "True"):
        CpswInit_dict["cpsw_vlanAware"] = 1
    else:
        CpswInit_dict["cpsw_vlanAware"] = 0

    if (win.ale_vlanAutoLearn_comboBox.currentText() == "True"):
        CpswInit_dict["ale_vlanAutoLearn"] = 1
    else:
        CpswInit_dict["ale_vlanAutoLearn"] = 0

    if (win.ale_unknownVlanNoLearn_comboBox.currentText() == "True"):
        CpswInit_dict["ale_unknownVlanNoLearn"] = 1
    else:
        CpswInit_dict["ale_unknownVlanNoLearn"] = 0

    CpswInit_dict["ale_UnknownForceUntaggedEgrMask"] = int(win.ale_UnknownForceUntaggedEgrMask_lineEdit.text())

    CpswInit_dict["ale_UnknownRegMcastFloodMask"] = int(win.ale_UnknownRegMcastFloodMask_lineEdit.text())

    CpswInit_dict["ale_UnknownURegMcastFloodMask"] = int(win.ale_UnknownURegMcastFloodMask_lineEdit.text())

    CpswInit_dict["ale_UnknownVlanMemberListMask"] = int(win.ale_UnknownVlanMemberListMask_lineEdit.text())

    if (win.nwSecCfg_hostOUINoMatchDeny_comboBox.currentText() == "True"):
        CpswInit_dict["nwSecCfg_hostOUINoMatchDeny"] = 1
    else:
        CpswInit_dict["nwSecCfg_hostOUINoMatchDeny"] = 0

    if (win.nwSecCfg_enableVID0Mode_comboBox.currentText() == "True"):
        CpswInit_dict["nwSecCfg_enableVID0Mode"] = 1
    else:
        CpswInit_dict["nwSecCfg_enableVID0Mode"] = 0

    if (win.nwSecCfg_disableSourceMcastDrop_comboBox.currentText() == "True"):
        CpswInit_dict["nwSecCfg_disableSourceMcastDrop"] = 1
    else:
        CpswInit_dict["nwSecCfg_disableSourceMcastDrop"] = 0

    if (win.nwSecCfg_enableBadLenPackDrop_comboBox.currentText() == "True"):
        CpswInit_dict["nwSecCfg_enableBadLenPackDrop"] = 1
    else:
        CpswInit_dict["nwSecCfg_enableBadLenPackDrop"] = 0

    if (win.nwSecCfg_dfltNoFragEn_comboBox.currentText() == "True"):
        CpswInit_dict["nwSecCfg_dfltNoFragEn"] = 1
    else:
        CpswInit_dict["nwSecCfg_dfltNoFragEn"] = 0

    if (win.nwSecCfg_enableDefaultNxtHeaderWL_comboBox.currentText() == "True"):
        CpswInit_dict["nwSecCfg_enableDefaultNxtHeaderWL"] = 1
    else:
        CpswInit_dict["nwSecCfg_enableDefaultNxtHeaderWL"] = 0

    CpswInit_dict["nwSecCfg_ipNxtHdrWLCount"] = int(win.nwSecCfg_ipNxtHdrWLCount_lineEdit.text())

    CpswInit_dict["nwSecCfg_ipNxtHdrWL_1"] = int(win.nwSecCfg_ipNxtHdrWL_lineEdit_1.text())

    CpswInit_dict["nwSecCfg_ipNxtHdrWL_2"] = int(win.nwSecCfg_ipNxtHdrWL_lineEdit_2.text())

    CpswInit_dict["nwSecCfg_ipNxtHdrWL_3"] = int(win.nwSecCfg_ipNxtHdrWL_lineEdit_3.text())

    CpswInit_dict["nwSecCfg_ipNxtHdrWL_4"] = int(win.nwSecCfg_ipNxtHdrWL_lineEdit_4.text())

    if (win.nwSecCfg_authModeEn_comboBox.currentText() == "True"):
        CpswInit_dict["nwSecCfg_authModeEn"] = 1
    else:
        CpswInit_dict["nwSecCfg_authModeEn"] = 0

    CpswInit_dict["nwSecCfg_MACAuthDisableMask"] = int(win.nwSecCfg_MACAuthDisableMask_lineEdit.text())

    if (win.vlanAware_comboBox.currentText() == "True"):
        CpswInit_dict["vlanAware"] = 1
    else:
        CpswInit_dict["vlanAware"] = 0

    if (win.crcRemove_comboBox.currentText() == "True"):
        CpswInit_dict["removeCrc"] = 1
    else:
        CpswInit_dict["removeCrc"] = 0

    if (win.padShortPackets_comboBox.currentText() == "True"):
        CpswInit_dict["padShortPacket"] = 1
    else:
        CpswInit_dict["padShortPacket"] = 0

    if (win.passCRCErrors_comboBox.currentText() == "True"):
        CpswInit_dict["passCrcErrors"] = 1
    else:
        CpswInit_dict["passCrcErrors"] = 0

    
    # Serialize and send
    status = serialize(CpswInit_dict, 0, "UART", global_ip, port)
    if (status == 0):
        load_window("Error")
        return
    
    win.target_init_label.setText("Initialization Done.")

# *****************************************************************************
#   Function disable_ip_edit
#
#   Disables the editing of IP when UART is the selected media
# *****************************************************************************
def disable_ip_edit():
    curr = win.media_comboBox.currentText()
    if (curr == "UART"):
        win.IP_lineEdit.setEnabled(False)
        win.setIP_button.setEnabled(False)
    elif (curr == "Network"):
        win.IP_lineEdit.setEnabled(True)
        win.setIP_button.setEnabled(True)

    win.media_top_label.setText(curr)

# *****************************************************************************
#   Function cmd_printer
#
#   Temporary function to read from cmd
# *****************************************************************************
def cmd_printer():
    pipe = os.popen("")
    while True:
        line = pipe.readline()
        if (line == ""):
            break
        win.output_textBrowser.append(line)

# *****************************************************************************
#   Function disable_test_edits
#
#   It disables some text fields in Main UI's Test Tab based on what test
#   option is chosen
# *****************************************************************************
def disable_test_edits():
    curr = win.test_comboBox.currentText()

    if (not curr == "TCP Send/Receive TLS"):
        win.serverCert_lineEdit.setEnabled(False)
        win.clientCert_lineEdit.setEnabled(False)
        win.clientKey_lineEdit.setEnabled(False)
    else:
        win.serverCert_lineEdit.setEnabled(True)
        win.clientCert_lineEdit.setEnabled(True)
        win.clientKey_lineEdit.setEnabled(True)
        
# *****************************************************************************
#   Function demo_start
#
#   UI menu shows a drop down menu with list of available demos.
#   After selecting the required demo, if the start button is pressed,
#   this function is called and the selected demo is enabled.
# *****************************************************************************
def demo_start():

    config = dict()
    # Check for selected demo
    demo_name = win.demo_comboBox.currentText()
    
    if (demo_name == "Inter VLAN Routing - SW"):
        config["cmd"] = 9
    if (demo_name == "Inter VLAN Routing - HW"):
        config["cmd"] = 10
        
    # Get Media
    chosen_media = win.media_comboBox.currentText()
    if (chosen_media == "Network" and (not check_ip(global_ip))):
        load_window("ipError")
        return
    if (chosen_media == "UART" and (port == "unset")):
        load_window("Error")
        return

    # Serialize and send
    status = serialize(config, 0, chosen_media, global_ip, port)
    if (status == 0):
        load_window("Error")
        return        
        
# *****************************************************************************
#   Function demo_stop
#
#   UI menu shows a drop down menu with list of available demos.
#   After selecting the required demo, if the stop button is pressed,
#   this function is called and the selected demo is disabled.
# *****************************************************************************
#def demo_stop():



# *****************************************************************************
#   Function testing
#
#   It runs a specified testing python script from command line 
# *****************************************************************************
def testing():
    curr = win.test_comboBox.currentText()

    if (curr == "TCP Send/Receive"):
        ip = win.IP_lineEdit_2.text()
        port = win.port_lineEdit.text()
        test_id = win.id_lineEdit.text()
        length = win.length_lineEdit.text()
        sleep = win.sleep_lineEdit.text()
        number = win.number_lineEdit.text()

        COMMAND = "python tcpSendReceive.py "+ip+" "+port+" "+test_id+" -l"+length+" -s"+sleep+" -n"+number
        COMMAND = "dir"
        # print(COMMAND)
        # printing = threading.Thread(target=cmd_printer)
        # printing.start()
        output = os.popen(COMMAND).read()
        win.output_textBrowser.setText(output)
    
    elif (curr == "TCP Send/Receive TLS"):
        ip = win.IP_lineEdit_2.text()
        port = win.port_lineEdit.text()
        test_id = win.id_lineEdit.text()
        length = win.length_lineEdit.text()
        sleep = win.sleep_lineEdit.text()
        number = win.number_lineEdit.text()
        server_cert = win.serverCert_lineEdit.text()
        client_cert = win.clientCert_lineEdit.text()
        client_key = win.clientKey_lineEdit.text()
        
        COMMAND = "python tcpSendReceiveTLS.py "+ip+" "+port+" "+test_id+" "+server_cert+" "+client_cert+" "+client_key+" -l"+length+" -s"+sleep+" -n"+number
        # print(COMMAND)
        output = os.popen(COMMAND).read()
        win.output_textBrowser.setText(output)

    elif (curr == "UDP Send/Receive"):
        ip = win.IP_lineEdit_2.text()
        port = win.port_lineEdit.text()
        test_id = win.id_lineEdit.text()
        length = win.length_lineEdit.text()
        sleep = win.sleep_lineEdit.text()
        number = win.number_lineEdit.text()
        
        COMMAND = "python udpSendReceive.py "+ip+" "+port+" "+test_id+" -l"+length+" -s"+sleep+" -n"+number
        output = os.popen(COMMAND).read()
        win.output_textBrowser.setText(output)

# *****************************************************************************
#   Function clear_stats
#
#   Handler for 'Clear' button in the Main UI's Stats Tab.
#   It clears the text containers.
# *****************************************************************************
def clear_stats():
    win.stats_textBrowser.setText("")
    win.stats_textBrowser_2.setText("")

# *****************************************************************************
#   Function clear_aleDump
#
#   Handler for 'Clear' button in the Main UI's ALE Tab.
#   It clears the text containers.
# *****************************************************************************
def clear_aleDump():
    win.dumpTable_textBrowser.setText("")

# *****************************************************************************
#   Function clear_config
#
#   Handler for 'Clear' button in the Main UI's File Tab.
#   It clears the text container.
# *****************************************************************************
def clear_config():
    global files 

    win.file_config_textBrowser.setText("")
    win.file_output_textBrowser.setText("")

    files = ['','']
    win.file_status_label.setText("No File Open")

# *****************************************************************************
#   Function add_template
#
#   Adds a structure template to populate on screen in File Tab of Main UI
# *****************************************************************************
def add_template():
    choice = win.file_template_comboBox.currentText()

    if (choice=="Version"):
        win.file_config_textBrowser.append(cmd_format_list[0])
    elif (choice=="Bcast/Mcast"):
        win.file_config_textBrowser.append(cmd_format_list[1])
    elif (choice=="Lookup Unicast"):
        win.file_config_textBrowser.append(cmd_format_list[2])
    elif (choice=="Host Stats"):
        win.file_config_textBrowser.append(cmd_format_list[3])
    elif (choice=="Mac Stats"):
        win.file_config_textBrowser.append(cmd_format_list[4])
    elif (choice=="Add VLAN"):
        win.file_config_textBrowser.append(cmd_format_list[5])
    elif (choice=="CPU Load"):
        win.file_config_textBrowser.append(cmd_format_list[6])
    elif (choice=="E/D Mcast"):
        win.file_config_textBrowser.append(cmd_format_list[7])
    elif (choice=="SW Inter VLAN Routing"):
        win.file_config_textBrowser.append(cmd_format_list[8])
    elif (choice=="HW Inter VLAN Routing"):
        win.file_config_textBrowser.append(cmd_format_list[9])    
    elif (choice=="IP Next Header Whitelisting"):
        win.file_config_textBrowser.append(cmd_format_list[10])    
    elif (choice=="Rate Limiting"):
        win.file_config_textBrowser.append(cmd_format_list[11])    

# *****************************************************************************
#   Function load_window
#
#   Displays a window win_name.ui
# *****************************************************************************
def load_window(win_name):
    global error

    error = uic.loadUi(ui_files_path + win_name + ".ui")
    error.show()
    return

# *****************************************************************************
#   Function init_fill_default
#
#   Fills default init time configuration values
# *****************************************************************************
def init_fill_default():
    win.ale_modeFlag_lineEdit.setText("5")
    win.ale_macAddr_lineEdit_1.setText("40")
    win.ale_macAddr_lineEdit_2.setText("ab")
    win.ale_macAddr_lineEdit_3.setText("cd")
    win.ale_macAddr_lineEdit_4.setText("ef")
    win.ale_macAddr_lineEdit_5.setText("fe")
    win.ale_macAddr_lineEdit_6.setText("dc")
    win.ale_policingEn_comboBox.setCurrentIndex(0)
    win.ale_yellowDropEn_comboBox.setCurrentIndex(0)
    win.ale_redDropEn_comboBox.setCurrentIndex(0)
    win.ale_yellowThreshold_lineEdit.setText("0")
    win.ale_noMatchMode_lineEdit.setText("0")
    win.ale_peakRate_lineEdit.setText("0")
    win.ale_commitRate_lineEdit.setText("0")
    win.ale_autoAgingEn_comboBox.setCurrentIndex(0)
    win.ale_agingPeriod_lineEdit.setText("1000")
    win.ale_vlanAware_comboBox.setCurrentIndex(0)
    win.cpsw_vlanAware_comboBox.setCurrentIndex(0)
    win.ale_vlanAutoLearn_comboBox.setCurrentIndex(0)
    win.ale_unknownVlanNoLearn_comboBox.setCurrentIndex(0)
    win.ale_UnknownForceUntaggedEgrMask_lineEdit.setText("0")
    win.ale_UnknownRegMcastFloodMask_lineEdit.setText("0")
    win.ale_UnknownURegMcastFloodMask_lineEdit.setText("0")
    win.ale_UnknownVlanMemberListMask_lineEdit.setText("511")
    win.nwSecCfg_hostOUINoMatchDeny_comboBox.setCurrentIndex(0)
    win.nwSecCfg_enableVID0Mode_comboBox.setCurrentIndex(0)
    win.nwSecCfg_disableSourceMcastDrop_comboBox.setCurrentIndex(0)
    win.nwSecCfg_enableBadLenPackDrop_comboBox.setCurrentIndex(0)
    win.nwSecCfg_dfltNoFragEn_comboBox.setCurrentIndex(0)
    win.nwSecCfg_enableDefaultNxtHeaderWL_comboBox.setCurrentIndex(0)
    win.nwSecCfg_ipNxtHdrWLCount_lineEdit.setText("0")
    win.nwSecCfg_ipNxtHdrWL_lineEdit_1.setText("0")
    win.nwSecCfg_ipNxtHdrWL_lineEdit_2.setText("0")
    win.nwSecCfg_ipNxtHdrWL_lineEdit_3.setText("0")
    win.nwSecCfg_ipNxtHdrWL_lineEdit_4.setText("0")
    win.nwSecCfg_authModeEn_comboBox.setCurrentIndex(0)
    win.nwSecCfg_MACAuthDisableMask_lineEdit.setText("0")
    win.portCfg_initPrms_textBrowser.setText(default_port_config)
    win.crcRemove_comboBox.setCurrentIndex(0)
    win.padShortPackets_comboBox.setCurrentIndex(0)
    win.passCRCErrors_comboBox.setCurrentIndex(0)
    win.vlanAware_comboBox.setCurrentIndex(0)

# *****************************************************************************
#   Function main
#
#   Initializes the GUI
# *****************************************************************************
if __name__ == "__main__":

    global global_ip
    global close_flag
    global pipe_flag
    global win
    global cli
    global chosen_media
    global fifo
    global fifo_path


    cli = False
    close_flag = False
    pipe_flag = False
    cfgfile =''
    # Create App
    app = QtWidgets.QApplication([])
    # Load Main Window
    win = uic.loadUi(ui_files_path + "main.ui") #specify the location of your .ui file

    # Connect components to functions
    win.pushButton.clicked.connect(display_version)
    win.pushButton_2.clicked.connect(display_bcast_mcast)
    win.lookupUnicast_button.clicked.connect(display_lookup_unicast)
    win.setIP_button.clicked.connect(get_ip)
    win.setUART_button.clicked.connect(get_uart)
    win.init_submit_pushButton.clicked.connect(init_cpsw)
    win.media_comboBox.currentIndexChanged.connect(disable_ip_edit)
    win.startTest_pushButton.clicked.connect(testing)
    win.test_comboBox.currentIndexChanged.connect(disable_test_edits)
    win.showStats_pushButton.clicked.connect(display_stats)
    win.clearStats_pushButton.clicked.connect(clear_stats)
    win.addVlan_submit_pushButton.clicked.connect(add_vlan)
    win.mcast_submit_pushButton.clicked.connect(enable_disable_mcast)
    win.file_sendConfig_pushButton.clicked.connect(send_config_file)
    win.file_clearConfig_pushButton.clicked.connect(clear_config)
    win.file_loadConfig_pushButton.clicked.connect(load_config_file)
    win.file_template_pushButton.clicked.connect(add_template)
    win.file_writeConfig_pushButton.clicked.connect(write_config_file)
    win.init_fillDefault_pushButton.clicked.connect(init_fill_default)
    win.uart_console_pushButton.clicked.connect(enable_uart_console)


    try:
      opts, args = getopt.getopt(sys.argv[1:],"ha:f:p:",["address=","configfile=", "pipe="])
    except getopt.GetoptError:
        print ("switchconfig_client.py -a <IP Address> -f <config_file> -p <pipe_name>")
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print ("switchconfig_client.py -a <IP Address> -f <config_file>")
            sys.exit()
        elif opt in ("-a", "--address"):
            global_ip = arg
            chosen_media = "Network" 
        elif opt in ("-f", "--configfile"):
            cli = True
            cfgfile = arg
        elif opt in ("-p", "--pipe"):
            fifo_path = arg
            pipe_flag = True
            try:
                os.mkfifo(fifo_path)
            except:
                pass

    if len(opts) == 0:
        # Show Main Window
        win.show()

        # Change App Icon and Execute App
        try:
            myappid = 'ti.configClient.1'
            ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID(myappid)
        except:
            pass
        app.setWindowIcon(QtGui.QIcon(ui_files_path + 'icon.png'))
        status = app.exec()
        if status == 0:
            close_flag = True
    else:
        cli_send_config_file(cfgfile)

    sys.exit(0)