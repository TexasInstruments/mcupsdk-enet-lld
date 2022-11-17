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

import os
import sys
import argparse
import requests
from requests.packages.urllib3.exceptions import InsecureRequestWarning

parser = argparse.ArgumentParser()
parser.add_argument("ipaddr", help="The IP address of the device")
parser.add_argument("-s", "--secure", help="Enable secure (https) requests",
                    action="store_true")
args = parser.parse_args()

# IP passed must match IP assigned to target
os.environ["no_proxy"] = args.ipaddr

if args.secure:
    base = 'https://' + args.ipaddr + ':443'
    print("Making secure requests")
else:
    base = 'http://' + args.ipaddr + ':80'

# Print out a response
def dump(r):
    print(str(r.status_code) + " - " + str(r.headers))
    try:
        print("(raw text)")
        print(r.text)
    except:
        print("(exception - raw text)")
        print(r.text)
    print("\n")

# Print out a response if it is not as expected
def dumpOnErr(r, status_code, response = "", headers = None):
    if (headers == None):
        # provide a good default based on response string
        headers = {'Content-Type': 'text/plain',
                   'Content-Length': str(len(response))}

    if (r.status_code != status_code):
        print("Error, expected " + str(status_code))
        dump(r)
    elif (r.text != response):
        print("Error, expected " + response)
        dump(r)
    elif (r.headers != headers):
        print("Error in headers, expected " + str(headers))
        dump(r)

try:
    # Begin requests
    s = requests.Session()
    # Due to the nature of the dummy certificate provided, we will not verify it
    s.verify = False
    requests.packages.urllib3.disable_warnings(InsecureRequestWarning)
    s.Connection = 'close'

    # Execute a get request for the '/home.html' page
    r = s.get(base + '/home.html')
    resp = "/get 'home.html': This is the resource requested."
    dumpOnErr(r, 200, resp)

    # Execute a get request for the '/login.html' page
    r = s.get(base + '/login.html')
    resp = "/get 'login.html': This is the login page."
    dumpOnErr(r, 200, resp);

    # Post some values to the server
    nameStr = "Tex"
    activityStr = "sleeping"

    payload = {"name":nameStr}
    r = s.post(base + '/db.html', params=payload)
    resp = "Data string successfully posted to server."
    dumpOnErr(r, 200, resp)
    dump(r)

    payload = {"function":activityStr}
    r = s.post(base + '/db.html', params=payload)
    resp = "Data string successfully posted to server."
    dumpOnErr(r, 200, resp)
    dump(r)

    # Retrieve the values just stored in the server
    payload = {"name":"val"}
    r = s.get(base + '/db.html', params = payload)
    resp = nameStr
    dumpOnErr(r, 200, resp)
    dump(r)

    payload = {"function":"val"}
    r = s.get(base + '/db.html', params = payload)
    resp = activityStr
    dumpOnErr(r, 200, resp)
    dump(r)

    # Execute a request for the put-specific URLHandler
    r = s.put(base + '/index.html')
    resp = "URLHandler #2 : /put This is the body of the resource you requested."
    dumpOnErr(r, 200, resp)
    dump(r)

    # Attempt a request the server is not currently configured to handle
    r = s.patch(base + '/db.html')
    resp = "This method is not supported at this URL."
    dumpOnErr(r, 405, resp)
    dump(r)

    print("Testing complete")
    exit(0)
except Exception as e:
    print("Caught exception - ")
    print(e)
    exit(1)
