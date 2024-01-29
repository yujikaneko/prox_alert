#!/usr/bin/env python3

from bluepy.btle import Scanner, DefaultDelegate

def hex_to_ascii(hex_string):
    bytes_obj = bytes.fromhex(hex_string)
    return bytes_obj.decode("ascii", "ignore").rstrip('\x00')

class ScanDelegate(DefaultDelegate):
    def __init__(self):
        DefaultDelegate.__init__(self)

    def handleDiscovery(self, dev, isNewDev, isNewData):
        for (adtype, desc, value) in dev.getScanData():
            if desc == "Manufacturer":
                if value[0:4] == "ffff":
                    print(hex_to_ascii(value[4:]), end='') 
                    break

scanner = Scanner().withDelegate(ScanDelegate())
scanner.scan(2.5)

