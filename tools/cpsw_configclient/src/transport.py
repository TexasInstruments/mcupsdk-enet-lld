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
/*                           Include Packages                                 */
/* ========================================================================== */
"""
import socket
import time
from xmodem import XMODEM
from xmodem import XMODEM1k
from time import sleep

"""
/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
"""
from switchconfig_client import *
from header import *
from header import CDataJSONEncoder

sock = 0

# *****************************************************************************
#   Function create_socket
#
#   Creates and returns a socket connected to ip and global_port for 
#   network tasks
# *****************************************************************************
def create_socket(ip):
    if (ip == "unset"):
        return 0
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(10)
    try:
        s.connect((ip, global_port))
    except:
        return 0
    return s

# *****************************************************************************
#   Function network_send
#
#   Send the serialised structure over the network to the IP "ip"
# *****************************************************************************
def network_send(serialised_config, ip):
    global sock
    if (sock == 0 or sock == "unset"):
        sock = create_socket(ip)
    if (sock == 0):
        print("Error in creating socket")
        return 0
    else:
        sock.sendall(serialised_config)
        print("COMMAND SENT.")
        return 1

# *****************************************************************************
#   Function network_receive
#
#   Receives a buffer of size BUFFER_SIZE() from Network on current global
#   socket "sock" which was assigned to inside network_send()
# *****************************************************************************
def network_receive():
    global sock
    if (sock == 0 or sock == "unset"):
        return 0
    received_buff = sock.recv(BUFFER_SIZE())
    print("OUTARGS RECEIVED.")
    return received_buff

# *****************************************************************************
#   Function uart_send
#
#   Sends the serialised config on UART
# *****************************************************************************
def uart_send(serialised_config, port):
    print("LENGTH OF BUFF IS : ",len(serialised_config))
    print(serialised_config)
    # *************************************************************************
    #   Function getc
    #
    #   Helper function needed by XMODEM.
    #   Reads "size" number of bytes from port
    # *************************************************************************
    def getc(size, timeout=1):
        return port.read(size) or None

    # *****************************************************************************
    #   Function putc
    #
    #   Helper function needed by XMODEM.
    #   Writes data to port
    # *****************************************************************************
    def putc(data, timeout=1):
        port.write(data)  # note that this ignores the timeout
        sleep(0.1)
    
    modem = XMODEM1k(getc, putc)
    f = open("temp.txt", "wb")
    f.write(serialised_config)
    f.close()
    f = open("temp.txt", "rb")
    
    status_flag = "False"
    while status_flag == "False" :
        status_flag = modem.send(f, retry= 16, timeout=5)
        print(status_flag)
        sleep(0.1)
    
    f.close()
    return 1

# *****************************************************************************
#   Function uart_receive
#
#   TBA
# *****************************************************************************