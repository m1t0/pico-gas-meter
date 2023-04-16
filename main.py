#
# Copyright 2022,2023 Michael Büttner
#

# This file is part of pico-gas-meter.
# 
# pico-gas-meter is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# pico-gas-meter is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with pico-gas-meter.  If not, see <http://www.gnu.org/licenses/>.
# 
# Diese Datei ist Teil von pico-gas-meter.
# 
# pico-gas-meter ist Freie Software: Sie können es unter den Bedingungen
# der GNU General Public License, wie von der Free Software Foundation,
# Version 3 der Lizenz oder (nach Ihrer Wahl) jeder neueren
# veröffentlichten Version, weiter verteilen und/oder modifizieren.
# 
# pico-gas-meter wird in der Hoffnung, dass es nützlich sein wird, aber
# OHNE JEDE GEWÄHRLEISTUNG, bereitgestellt; sogar ohne die implizite
# Gewährleistung der MARKTFÄHIGKEIT oder EIGNUNG FÜR EINEN BESTIMMTEN ZWECK.
# Siehe die GNU General Public License für weitere Details.
# 
# Sie sollten eine Kopie der GNU General Public License zusammen mit diesem
# Programm erhalten haben. Wenn nicht, siehe <https://www.gnu.org/licenses/>.


#
# /!\ Warning: This is program code that has not been reviewed for code
#              quality, security, and readability. It is the first prototype
#              that was executable and was released as it was.
# 


# https://datasheets.raspberrypi.com


# https://github.com/dhylands/rshell
# rshell
#   help
#   boards
#   repl
#   ls /pyboard
#   cp main.py /pyboard/main.py
# Useful control commands:
# CTRL-C -- interrupt a running program
# CTRL-D -- on a blank line, do a soft reset of the board
# CTRL-E -- on a blank line, enter paste mode

#
#  from machine import Pin, Timer
#  led = Pin("LED", machine.Pin.OUT)
#  tim = Timer()
#  def tick(timer):
#      global led
#      led.toggle()
# 
# 
#  tim.init(freq=2.5, mode=Timer.PERIODIC, callback=tick)

# TODO: split code into modules and files: nethelper.py, bmm150.py, bst_spi.py, quarry.py
# TODO: send zaehlerstand: 0 on startup, use ntptime
# TODO: do ntplookup and if suceeded add 'ntpdate' to json
# TODO: exclude wlan_auth.py w/ ssid and pass
# TODO: exclude iface.py w/ ntp-server, ip, bcast, 
# TODO: add threshold to protocol


##################################################################################
# https://datasheets.raspberrypi.com/picow/connecting-to-the-internet-with-pico-w.pdf
# https://docs.micropython.org/en/latest/library/network.WLAN.html
auth_ssid='MyWLAN'
auth_pass='12345678901234567890'

import ubinascii
import time
# https://docs.micropython.org/en/latest/library/json.html
import json


# nic.hostname("PIP00-0397-2411")
# nic.config(pm = 0xa11140)	# power mode, more responsive
##############################################################################
def network_up(nic_rp2, authssid, authpass):
    if not nic_rp2.active():
        nic_rp2.active(True)

    nic.config(pm = 0xa11140)	# power mode, more responsive

    if not nic_rp2.isconnected():
        nic.connect(authssid, authpass)
        while not nic_rp2.isconnected() and nic_rp2.status() <= 0:
            machine.idle()

def network_down(nic_rp2):
    nic.disconnect()
    nic_rp2.active(False)

def print_nic_status(nic_rp2):
    time.sleep(1)
    print("nic: status: ", nic_rp2.status())

#   -3 -> CYW43_LINK_BADAUTH
#   -2 -> CYW43_LINK_NONET
#   -1 -> CYW43_LINK_FAIL
#    0 -> CYW43_LINK_DOWN
#    1 -> CYW43_LINK_JOIN
#    2 -> CYW43_LINK_NOIP
#    3 -> CYW43_LINK_UP

import network
rp2.country('DE')
nic = network.WLAN(network.STA_IF)

network_up(nic, auth_ssid, auth_pass)
print_nic_status(nic)

##############################################################################
(ip,nm,gw,ns) =nic.ifconfig()

# .config("mac") need nic.active(True), which loads the firmware
# mac=nic.config('mac') # MAC address (bytes)
mac = ubinascii.hexlify(network.WLAN().config('mac'),':').decode()
# 28:cd:c1:05:66:6a # mac_release
# 28:cd:c1:04:19:b6 # mac_debug
ssid = nic.config('ssid') # WiFi access point name (string)
channel = nic.config('channel') # WiFi channel (integer)
## hidden = nic.config('hidden') # Whether SSID is hidden (boolean)
security = nic.config('security') # Security protocol supported (enumeration, see module constants)
## key = nic.config('key') # Access key (string)
## hostname = nic.config('hostname') # The hostname that will be sent to DHCP (STA interfaces) and mDNS (if supported, both STA and AP)
# reconnects = nic.config('reconnects') # Number of reconnect attempts to make (integer, 0=none, -1=unlimited)
txpower = nic.config('txpower') # Maximum transmit power in dBm (integer or float)


### print(ip, bc, gw, ns, mac)

### print("mac", mac)
### print("ssid", ssid)
### print("channel", channel)
### ## print("hidden", hidden)
### print("security", security)
### ## print("key", key)
### ## print("hostname", hostname)
### ## print("reconnects", reconnects)
### print("txpower", txpower)

bc='192.168.178.255' # FIXME: calc broadcast from ip+nm
ntp="fritz.box" # FIXME resolve "fritz.box"
ntp="192.168.178.1" # FIXME resolve "fritz.box"

if nic.isconnected():
    updown="UP"
else:
    updown="DOWN"

print("wlan: flags=<", updown, ",BROADCAST,RUNNING,MULTICAST>  mtu 1500")
print("       inet", ip, " netmask", nm, " broadcast", bc)
print("       ether", mac, " txqueuelen 1000 (WiFi)")
print("       wifi", ssid, " channel", channel, " security", security, " txpower", txpower)
print("       gw", gw, " dns", ns, " ntp", ntp)
# print("       RX packets ???  bytes ??? (??? MB)")
# print("       RX errors ?  dropped ?  overruns ?  frame ?")
# print("       TX packets ?  bytes ? (? MB)")
# print("       TX errors ?  dropped ? overruns ?  carrier ?  collisions ?")

##################################################################################
# at startup, initialize the hw clock w/ ntp
import ntptime
import time

#if needed, overwrite default time server
# ntptime.host = "1.europe.pool.ntp.org"
# ntptime.host = ntp

def ntpsettime():
    # print("ntp:", ntptime.host)
    try:
        # print("ntp: Local time before synchronization：%s" %str(time.localtime()))
        #make sure to have internet connection
        ntptime.settime()
        # print("ntp: Local time after synchronization：%s" %str(time.localtime()))
    except Exception as e:
        print("ntp: ", e)
        print("ntp: Error syncing time")

ntpsettime()

##################################################################################
# nic.disconnect();
##################################################################################



from machine import SPI, Pin
import time
import math
import utime
led = Pin("LED", machine.Pin.OUT)
led.value(0)
time.sleep_ms(100)
led.value(1)
time.sleep_ms(100)
led.value(0)
time.sleep_ms(100)
led.value(1)

##### Assign chip select (CS) pin (and start it high)
cs = Pin(17, Pin.OUT)
spi = SPI(0,
           baudrate=400000,
           polarity=1,
           phase=1,
           bits=8,
           firstbit=SPI.MSB,
           sck=Pin(18),
           mosi=Pin(19),
           miso=Pin(16))

def reg_write(reg, data):    
    # Write 1 byte to the specified device and register.
    # Construct message (set ~W bit low, MB bit low)
    msg = bytearray()
    msg.append(reg & 0x7F)
    msg.append(data)
    print(hex(reg), hex(data))
    cs.value(0)
    spi.write(msg)
    cs.value(1)

     
def reg_read(reg, nbytes=1):
    # Read 1 byte from specified register.
    # Construct message
    msg = bytearray()
    msg.append((reg & 0x7F) | 0x80 ) # 0x80 for read
    msg += bytearray([0x00] * nbytes)
    # Send out SPI message and read
    cs.value(0)
    # spi.write(msg)
    # data = spi.read(nbytes)
    data=bytearray(len(msg))
    spi.write_readinto(msg, data)
    cs.value(1)
    # print(hex(msg[0]), hex(data[1]))
    return data[1:]

def encodedata(d):
    # x
    data_x_lsb=(d[0]>>3) & 0x1F
    data_x_msb=d[1]<<5
    data_x=data_x_msb | data_x_lsb
    # y
    data_y_lsb=(d[2]>>3) & 0x1F
    data_y_msb=d[3]<<5
    data_y=data_y_msb | data_y_lsb
    # z
    data_z_lsb=(d[4]>>1) & 0x7F
    data_z_msb=d[5]<<7
    data_z=data_z_msb | data_z_lsb
    # rhall
    data_rhall_lsb=(d[6]>>3) & 0x1F
    data_rhall_msb=d[7]<<5
    data_rhall=data_rhall_msb | data_rhall_lsb
    return (data_x, data_y, data_z, data_rhall)


BMM150_CHIP_ID=0x40
BMM150_DATA_X=0x42
BMM150_POWER_MODE=0x4B	# write 0x01 for sleep mode write 0x00 for suspend mode, 0x82 for soft reset
BMM150_POWER_MODE_POWER_CONTROL_0=0<<0
BMM150_POWER_MODE_POWER_CONTROL_1=1<<0
BMM150_OP_MODE=0x4C	# write 0x00 for "10Hz" ODR and "Normal Mode"

BMM150_OP_MODE_NORMAL=0<<1
BMM150_OP_MODE_FORCED=1<<1
BMM150_OP_MODE_SLEEP=3<<1

BMM150_OP_MODE_ODR_10Hz=0<<3
BMM150_OP_MODE_ODR_2Hz=1<<3
BMM150_OP_MODE_ODR_6Hz=2<<3
BMM150_OP_MODE_ODR_8Hz=3<<3
BMM150_OP_MODE_ODR_15Hz=4<<3
BMM150_OP_MODE_ODR_20Hz=5<<3
BMM150_OP_MODE_ODR_25Hz=6<<3
BMM150_OP_MODE_ODR_30Hz=7<<3


def bmm150_init():
    reg_write(BMM150_POWER_MODE, 0x82)
    reg_write(BMM150_POWER_MODE, 0x01)
    reg_write(BMM150_OP_MODE, 0x00)
    print_chip_id()
####time.sleep_ms(10)
####for i in range(0,99):
####    #while( (reg_read(0x48)[0] & 0x01) == 0 ):
####    #	time.sleep_ms(50)
####    #	print(".")
####    d=reg_read(BMM150_DATA_X, 8)
####    print(encodedata(d),d)


def print_chip_id():
    print("CHIP_ID " + hex(reg_read(BMM150_CHIP_ID, 1)[0]))

def suspend_mode():
    reg_write(BMM150_POWER_MODE, BMM150_POWER_MODE_POWER_CONTROL_0)	# if we are in sleep or normal mode
    print("Power Mode: SUSPEND")

def sleep_mode():
    # Power control = '1'
    reg_write(BMM150_POWER_MODE, BMM150_POWER_MODE_POWER_CONTROL_1)	# if we are in suspend mode
    reg_write(BMM150_OP_MODE, BMM150_OP_MODE_SLEEP | BMM150_OP_MODE_ODR_10Hz) # if we are in active mode
    print("Power Mode: SLEEP")
    print_chip_id()

def active_mode():
    reg_write(BMM150_OP_MODE, BMM150_OP_MODE_NORMAL | BMM150_OP_MODE_ODR_30Hz) # if we are in active mode
    print("Power Mode: ACTIVE")


def vector_length(x_axis,y_axis,z_axis):
    x=x_axis * x_axis
    y=y_axis * y_axis
    z=z_axis * z_axis
    return math.sqrt(x+y+z)



def calc_state(vlen, vmin, vmax, state):
    # check for new min and max values
    if vlen>vmax:
        vmax=vlen
    if vlen<vmin:
        vmin=vlen
    th=(vmax-vmin)/2 + vmin
    count=0
    # check for 1->0 transition
    if state == 1:
        if vlen < th*0.9:
            state=0
    # check for 0->1 transition
    if state == 0:
        if vlen > th*1.1:
            state=1
            # state flips to 1, so send one 1
            count=1
    return count, vmin, vmax, state


bmm150_init()
reg_write(BMM150_POWER_MODE, 0x82)
reg_read(BMM150_DATA_X, 9)
suspend_mode()
sleep_mode()
active_mode()

# socket
import socket
s=socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1) 

# IPv4
dst_host='192.168.178.202'
print("dst_host", dst_host)
if mac == '28:cd:c1:05:66:6a':
    dst_port=4488 # the real counter
else:
    dst_port=8844 # debug/dummy device
dst_sockaddr=socket.getaddrinfo(dst_host, dst_port)[0][-1]


def run():
    gascounter=0
    now=utime.time()
    last=now - 1
    c=0
    mi=20000
    ma=20001
    st=0
    while(True):
        xx,yy,zz,hh=[0,0,0,0]
        # use the average of 10 data samples
        for m in range(0,10):
            while( (reg_read(0x48)[0] & 0x01) == 0 ):
                time.sleep_ms(10)
            d=reg_read(BMM150_DATA_X, 8)
            x,y,z,rhall=encodedata(d)
            xx+=x
            yy+=y
            zz+=z
            hh+=rhall
        l=vector_length(xx/10.0,yy/10.0,zz/10.0)
        c,mi,ma,st=calc_state(l,mi,ma,st)
        # print(utime.time(),";",xx/10.0,";",yy/10.0,";",zz/10.0,";",hh/10.0,";",l,";",mi,";",ma,";",st,";",c,";")
        if c==1:
            oldnow=utime.time()
            try:
                ntpsettime()
            except:
                pass
            now=utime.time()
            gascounter+=1
            verbrauch=10/float(now - last) # delta_gaszaehler / delta_zeit (1imp /0.01m³ = 10 l)) 
            # logstring=json.dumps({ 'time': now, 'min': mi, 'length': l, 'max': ma, 'zaehlerstand': gascounter, 'verbrauch': verbrauch, 'version': 2, 'oldtime': oldnow, 'debug': True })
            logstring=json.dumps({ 'time': now, 'min': mi, 'length': l, 'max': ma, 'zaehlerstand': gascounter, 'verbrauch': verbrauch, 'version': 2, 'oldtime': oldnow, 'mac': mac })
            udp_dump(dst_sockaddr, logstring)
            print(logstring)
            last=now
        led.value(st)

def run2():
    gascounter=0
    now=utime.time()
    last=now - 1
    c=0
    mi=20000
    ma=20001
    st=0
    while(True):
        xx,yy,zz,hh=[0,0,0,0]
#       for m in range(0,10):
#           while( (reg_read(0x48)[0] & 0x01) == 0 ):
#               time.sleep_ms(10)
#           d=reg_read(BMM150_DATA_X, 8)
#           x,y,z,rhall=encodedata(d)
#           xx+=x
#           yy+=y
#           zz+=z
#           hh+=rhall
        l=vector_length(xx/10.0,yy/10.0,zz/10.0)
        c,mi,ma,st=calc_state(l,mi,ma,st)
        print(utime.time(),";",xx/10.0,";",yy/10.0,";",zz/10.0,";",hh/10.0,";",l,";",mi,";",ma,";",st,";",c,";")
        if c==1:
            oldnow=utime.time()
            try:
                ntpsettime()
            except:
                pass
            now=utime.time()
            gascounter+=1
            verbrauch=10/float(now - last) # delta_gaszaehler / delta_zeit (1imp /0.01m³ = 10 l)) 
            logstring=json.dumps({ 'time': now, 'min': mi, 'length': l, 'max': ma, 'zaehlerstand': gascounter, 'verbrauch': verbrauch, 'version': 2 })
            # logstring=str(now) + ";" + str(mi) + ";" + str(l) + ";" + ";" + ";" + str(gascounter) + ";" + str(verbrauch) +";"
            # print(utime.time(), ";",gascounter,";",verbrauch,";")
            udp_dump(dst_sockaddr, logstring)
            print(logstring)
            last=now
        led.value(st)



def nic_reconnect():
    nic.connect(auth_ssid, auth_pass)
    while not nic.isconnected() and nic.status() >= 0:
        print("nic: Waiting to connect")
        time.sleep(1)

    if nic.isconnected():
        print("nic: OK")
    else:
        print("nic: ERROR: no network available")

def udp_dump(sockaddr, data):
    if not nic.isconnected():
        print("nic: ERROR: no network available (reconnect)")
        nic_reconnect()

    network_up(nic, auth_ssid, auth_pass)
    # socket
    import socket
    s=socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1) 
    s.sendto(data + "\n",sockaddr)
    s.close()
    # network_down(nic)

if mac == '28:cd:c1:05:66:6a': # mac_release
    run()
else:
    run()

####
##################################################################################
####>>> import utime
####>>> utime.time()
####1665532955
####
##################################################################################
####
####https://datasheets.raspberrypi.com/picow/connecting-to-the-internet-with-pico-w.pdf
####
####
####>>> from machine import Pin
####>>> led = Pin("LED", machine.Pin.OUT)
####>>> led.off()
####>>> led.on()
####>>> led.off()
####
####
####
####https://www.elektronik-kompendium.de/sites/raspberry-pi/2708121.htm
####
####>>> time.time()
####1609460319
####
##### 1609460319 entspricht Friday, 1. January 2021 00:18:39 (GMT)
####
####
####
####Raspberry Pi Pico: Taster entprellen
####    https://www.elektronik-kompendium.de/sites/raspberry-pi/2612181.htm
####
####
####Registermap
####-----------
####0x40	Chip-ID (=0x32)
####0x41	reserved
####0x42	X
####0x43	
####0x44	Y
####0x45	
####0x46	Z
####0x47
####0x48	HALL
####0x49
####0x4A
####0x4B	PowerControl Bit 0x1 (set to one to enalbe reading of ChipID)
####0x71
