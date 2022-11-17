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

import json

"""
/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
"""
from header import *
from header import CDataJSONEncoder
from transport import *

"""
/* ========================================================================== */
/*                           Function Declarations                            */
/* ========================================================================== */
"""

# *****************************************************************************
#   Function serialize
#
#   Receives a structure from GUI, creates a json byte stream based on the 
#   media chosen as well as whether or not the structure is of C type. It 
#   then sends the byte stream over the selected media.
# *****************************************************************************
def serialize(config, isCtypes, media, ip, port):
    if (isCtypes):
        if (media == "Network"):
            serialised_config = json.dumps(config, cls=CDataJSONEncoder)
            buff = bytes(serialised_config, 'utf-8')
            buff = buff + b'0'*(BUFFER_SIZE()-len(buff))
            
        elif (media == "UART"):
            serialised_config = json.dumps(config, cls=CDataJSONEncoder)
            buff = bytes(serialised_config, 'utf-8')
            
    else:
        if (media == "Network"):
            serialised_config = json.dumps(config)
            buff = bytes(serialised_config, 'utf-8')
            buff = buff + b'0'*(BUFFER_SIZE()-len(buff))
            
        elif (media == "UART"):
            serialised_config = json.dumps(config)
            buff = bytes(serialised_config, 'utf-8')
            
    if (media == "Network"):
        return network_send(buff, ip)
    elif (media == "UART"):
        return uart_send(buff, port)

# *****************************************************************************
#   Function deserialize
#
#   Receives a bytestream on specified media and creates a structure 
#   from it by cleaning trailing garbage and loading cleaned string into 
#   a dictionary called received_struct
# *****************************************************************************
def deserialize(media):

    if (media == "Network"):
        buff = network_receive()
        if (buff == 0):
            return 0

    if (media == "UART"):
        return 0
    
    json_string = buff.decode("utf-8", "ignore")
        
    clean_json_string = ""
    count = 0
    i = 0
    while(1):
        if (json_string[i] == "{"):
            count = count + 1
        elif (json_string[i] == "}"):
            count = count -1
        
        clean_json_string = clean_json_string + json_string[i]
        i = i+1
        if (count == 0):
            break

    received_struct = json.loads(clean_json_string)
    return received_struct
