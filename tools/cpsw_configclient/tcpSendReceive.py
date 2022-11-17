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

# Echo client program
import socket
import getopt
import sys
import time
import signal
import math

BUFFSIZE = 1024             # buffer length
SLEEPTIME = 0               # sleep time in ms, defualt to no sleep time
ECHOCOUNT = 100             # number of tcp echoes to perform before printing

s = None
COUNT = 0
ARGV = sys.argv[1:]
ARGC = len(ARGV)
START = time.time()

COMMAND = "tcpSendReceive.py <IPv4 or IPv6 addr> <port> <id> -l[length]" + \
          "-s[sleep in mS] -n[number of transmits per report]"

if sys.version_info.major < 3:
    sys.stderr.write("Error: tcpSendReceive.py requires Python 3\n")
    sys.exit(1)

def handler(signum, frame):
    print("closing connection...")
    s.close()
    sys.exit()

signal.signal(signal.SIGINT, handler)

# Parse options
if ((ARGC < 3) or (ARGC > 6)):
    print("usage: " + COMMAND)
    sys.exit()
try:
    opts, args = getopt.getopt(ARGV[3:], "hl:s:n:", ["len=", "sleep="])
except getopt.GetoptError:
    print(COMMAND)
    sys.exit()
for opt, arg in opts:
    if opt == "-h":
        print("HELP: " + COMMAND)
        sys.exit()
    elif opt in ("-l", "--len"):
        BUFFSIZE = int(arg)
    elif opt in ("-s", "--sleep"):
        SLEEPTIME = int(arg)
    elif opt in ("-n", "--number"):
        ECHOCOUNT = int(arg)
    else:
        print("Valid options are -l[length] and -s[sleep in mS]" +
              " -n[number of transmits per report]")

buffer = bytearray(BUFFSIZE)

# open socket
for res in socket.getaddrinfo(ARGV[0], ARGV[1], socket.AF_UNSPEC,
                              socket.SOCK_STREAM):
    af, socktype, proto, canonname, sa = res
    try:
        s = socket.socket(af, socktype, proto)
    except socket.error as msg:
        s = None
        continue
    try:
        s.connect(sa)
    except socket.error as msg:
        s.close()
        s = None
        continue
    break
if s is None:
    print("could not open socket")
    sys.exit()

print("Starting test with a " + str(SLEEPTIME) +
      " mSec delay between transmits and reporting on every " + str(ECHOCOUNT)
      + " transmit(s)")

# main program loop
i = 0
while True:
    # Set the first and last char with a incrementing value
    i = (i + 1) % 256
    buffer[0] = i
    firstByte = buffer[0]
    # number of bytes used to store ~i
    sizeNegI = math.ceil((~i).bit_length() / 8)
    buffer[-1] = (~i).to_bytes(sizeNegI + 1, byteorder='big', signed=True)[-1]
    lastByte = buffer[-1]

    # send tcp payload
    try:
        s.sendall(buffer)
    except socket.error as msg:
        print("[id " + ARGV[2] + "] stopping test. sendall failed")
        print(msg)
        s.close()
        sys.exit()

    # receive tcp payload
    totalBytesRead = 0
    while totalBytesRead < len(buffer):
        try:
            data = s.recv(len(buffer) - totalBytesRead)
        except socket.error as msg:
            print("[id " + ARGV[2] + "] stopping test. recv returned " + msg)
            s.close()
            sys.exit()
        buffer[totalBytesRead:totalBytesRead + len(data)] = data
        totalBytesRead = totalBytesRead + len(data)

    COUNT = COUNT + 1
    if COUNT % ECHOCOUNT == 0:
        deltaTime = time.time() - START
        print("[id " + ARGV[2] + "] COUNT = " + str(COUNT) +
              ", time = " + "%.2f" % deltaTime + " seconds")

    # SLEEPTIME < 10000 may sleep for significantly longer than expected
    time.sleep(SLEEPTIME / 1000)

    if (buffer[0] != firstByte) or (buffer[-1] != lastByte):
        print("mismatch buffer[0] = " + str(buffer[0]) +
              ", (char)i = " + str(firstByte))
        print("mismatch buffer[BUFFSIZE - 1] = " +
              str(buffer[-1]) + ", (char)~i = " + str(lastByte))
