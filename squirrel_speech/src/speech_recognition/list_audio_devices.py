#!/usr/bin/env python

from __future__ import print_function


from std_msgs.msg import String

import sys
import pyaudio



# obtain device count ----------------------------------------------------------
p = pyaudio.PyAudio(); 
count = p.get_device_count(); 

print("Available devices: {}".format(count))

p = pyaudio.PyAudio()
max_apis = p.get_host_api_count()
max_devs = p.get_device_count()

print ("\nPortAudio System Info:\n======================")
print ("Version:" +     str(pyaudio.get_portaudio_version()))
print ("Version Text: " + str(pyaudio.get_portaudio_version_text()))
print ("Number of Host APIs: " + str(max_apis))
print ("Number of Devices  : " + str(max_devs))


print ("\nHost APIs:\n==========")

for i in range(max_apis):
    apiinfo = p.get_host_api_info_by_index(i)
    for k in apiinfo.items():
        print (str(k) +" : ")
    print( "--------------------------")

print( "\nDevices:\n========")

for i in range(max_devs):
    devinfo = p.get_device_info_by_index(i)
    # print out device parameters
    print( "--------------------------")
    for k in devinfo.items():
        name, value = k

        # if host API, then get friendly name
        
        if name == 'hostApi':
            value = str(value) + \
                    " (%s)" % p.get_host_api_info_by_index(k[1])['name']
        print (str(name) + " : " + str(value))
print( "--------------------------")
p.terminate() 

