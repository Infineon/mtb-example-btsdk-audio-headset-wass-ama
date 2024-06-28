#!/usr/bin/python -tt
#
# Copyright 2016-2024, Cypress Semiconductor Corporation (an Infineon company) or
# an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
#
# This software, including source code, documentation and related
# materials ("Software") is owned by Cypress Semiconductor Corporation
# or one of its affiliates ("Cypress") and is protected by and subject to
# worldwide patent protection (United States and foreign),
# United States copyright laws and international treaty provisions.
# Therefore, you may use this Software only as provided in the license
# agreement accompanying the software package from which you
# obtained this Software ("EULA").
# If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
# non-transferable license to copy, modify, and compile the Software
# source code solely for use in connection with Cypress's
# integrated circuit products.  Any reproduction, modification, translation,
# compilation, or representation of this Software except as specified
# above is prohibited without the express written permission of Cypress.
#
# Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
# EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
# reserves the right to make changes to the Software without notice. Cypress
# does not assume any liability arising out of the application or use of the
# Software or any product or circuit described in the Software. Cypress does
# not authorize its products for use in any products where a malfunction or
# failure of the Cypress product may reasonably be expected to result in
# significant property damage, injury or death ("High Risk Product"). By
# including Cypress's product in a High Risk Product, the manufacturer
# of such system or application assumes all risk of such use and in doing
# so agrees to indemnify Cypress against all liability.
#
# Test Program to test the Audio Insert feature
# This program sends a WICED HCI command to test this feature
# This program has been designed to run under Cygwin.
# Cygwin use /dev/ttySx instead of COMy to access Com Ports.
# Note that 'x' is 'y-1' => COM20 = /dev/ttyS19

# To execute this test script (COM18, 3000000bps, wait for result)
# ./ps-switch.py -d /dev/ttyS17 -b 3000000 -w
# Return value
#   0: success
#   1: reject
#   2: ready too long
#   others: fail

# The following Python modules are required
# To install the 'serial' package, you can use $python -m pip install pyserial
import serial
import time
import sys
import enum
import struct

class WicedResult(enum.IntEnum):
    SUCCESS = 0x00
    NO_EVENTS = 0x07
    NOT_AVAILABLE = 0x1D
    ERROR = 0x28

class SwitchResult(enum.IntEnum):
    WICED_BT_LRAC_SWITCH_SUCCESS                                = 0x00
    WICED_BT_LRAC_SWITCH_NOT_READY                              = 0x01
    WICED_BT_LRAC_SWITCH_READY_TOO_LONG                         = 0x02
    WICED_BT_LRAC_SWITCH_BUSY                                   = 0x03
    WICED_BT_LRAC_SWITCH_FORCE_ABORT                            = 0x04
    # General Error
    WICED_BT_LRAC_SWITCH_NO_MEMORY                              = 0x10
    # HCI
    WICED_BT_LRAC_SWITCH_DO_EXECUTE_FAIL                        = 0x20
    WICED_BT_LRAC_SWITCH_EXECUTE_FAIL                           = 0x21
    WICED_BT_LRAC_SWITCH_DO_START_FAIL                          = 0x22
    WICED_BT_LRAC_SWITCH_START_FAIL                             = 0x23
    WICED_BT_LRAC_SWITCH_DO_ASSOCIATE_FAIL                      = 0x24
    WICED_BT_LRAC_SWITCH_DO_GET_ACL_EAVESDROPPING_PARAM_FAIL    = 0x25
    WICED_BT_LRAC_SWITCH_ACL_EAVESDROPPING_FAIL                 = 0x26
    WICED_BT_LRAC_SWITCH_DO_PAUSE_FAIL                          = 0x27
    WICED_BT_LRAC_SWITCH_PAUSE_FAIL                             = 0x28
    WICED_BT_LRAC_SWITCH_DO_UNPAUSE_FAIL                        = 0x29
    WICED_BT_LRAC_SWITCH_DO_PARAM_GET_FAIL                      = 0x2A
    WICED_BT_LRAC_SWITCH_PARAM_GET_FAIL                         = 0x2B
    WICED_BT_LRAC_SWITCH_DO_FINALIZE_FAIL                       = 0x2C
    WICED_BT_LRAC_SWITCH_FINALIZE_FAIL                          = 0x2D
    # Process
    WICED_BT_LRAC_SWITCH_DATA_APPLY_FAIL                        = 0x40
    WICED_BT_LRAC_SWITCH_DATA_COLLECT_FAIL                      = 0x41
    # L2CAP Ready
    WICED_BT_LRAC_SWITCH_GET_L2CAP_READY_FAIL                   = 0x50
    WICED_BT_LRAC_SWITCH_DATA_APPLY_L2CAP_READY_FAIL            = 0x51
    WICED_BT_LRAC_SWITCH_REQ_L2CAP_READY_FAIL                   = 0x52
    WICED_BT_LRAC_SWITCH_RSP_L2CAP_READY_FAIL                   = 0x53
    # CTRL OPCODE
    WICED_BT_LRAC_SWITCH_SEND_SWITCH_REQ_FAIL                   = 0x60
    WICED_BT_LRAC_SWITCH_SEND_SWITCH_RSP_FAIL                   = 0x61
    WICED_BT_LRAC_SWITCH_SEND_DATA_RSP_DATA_TOO_BIG             = 0x62
    WICED_BT_LRAC_SWITCH_SEND_DATA_RSP_SEND_FAIL                = 0x63
    WICED_BT_LRAC_SWITCH_SEND_ACL_START_FAIL                    = 0x64
    WICED_BT_LRAC_SWITCH_RCV_ACL_START_RSP_FAIL                 = 0x65
    WICED_BT_LRAC_SWITCH_SEND_ACL_STOP_FAIL                     = 0x66
    WICED_BT_LRAC_SWITCH_RCV_ACL_STOP_RSP_FAIL                  = 0x67
    WICED_BT_LRAC_SWITCH_RCV_RSP_FAIL                           = 0x68
    WICED_BT_LRAC_SWITCH_ACL_WAIT_CTRL_TIMEOUT                  = 0x69
    # OTHER
    WICED_BT_LRAC_SWITCH_USER_ABORT                             = 0x70
    # TIMEOUT
    WICED_BT_LRAC_SWITCH_TIMEOUT                                = 0xFF


ser=None

# Open Serial Com Port
def serial_port_open(COMport,BaudRate=115200):
    global ser

    if ser!=None:
        ser.close()
        ser=None
    COMportOpen=True
    print 'Opening', COMport, 'at', BaudRate, 'bps'
    try:
        ser = serial.Serial(port=COMport,
                            baudrate=BaudRate,
                            parity=serial.PARITY_NONE,
                            stopbits=serial.STOPBITS_ONE,
                            bytesize=serial.EIGHTBITS,
                            rtscts=True,
                            timeout=1)
    except serial.SerialException:
        COMportOpen=False

    return COMportOpen

# Read data from Serial Port
def serial_port_read(response_len=0,Timeout=1):
    response_finished=False
    start_time=time.time()
    response=''

    while (len(response)<response_len):
        response=response+ser.read(ser.inWaiting())
        if ((time.time()-start_time)>Timeout):
            return response
    print('UART RX<<%s'%' '.join('{:02X}'.format(ord(c)) for c in response))
    return response

# Send Audio Insert Test command
def ps_switch(prevent_glitch):
    # Flush RX (1 second)
    serial_port_read(1024,1)
    # Send command
    message='\x19\x23\xD0\x01\x00'+chr(prevent_glitch)
    print('UART TX>>%s'%' '.join('{:02X}'.format(ord(c)) for c in message))
    ser.write(message)
    # Wait event
    response = serial_port_read(6,1)
    event_id, event_group, payload_length, status = struct.unpack("<BBHB", response[1:6])
    if (event_id != 0xFF or event_group != 0xD0):
        return WicedResult.NO_EVENTS
    return status

def ps_switch_wait():
    response = serial_port_read(9,3)
    if (len(response) < 9):
        print("Wait timeout")
        return SwitchResult.WICED_BT_LRAC_SWITCH_TIMEOUT

    (event_id,
            event_group,
            payload_length,
            status,
            local_abort,
            fatal_error) = struct.unpack("<BBHHBB", response[1:9])
    if (event_id != 0x23 or event_group != 0xD0):
        print("Unexpected event")
        return SwitchResult.WICED_BT_LRAC_SWITCH_TIMEOUT
    print("Status:0x{:02X} local_abort:{:d} fatal_error:{:d}".
            format(status, local_abort, fatal_error))
    return status;

# Wait 1 second to receive the, optional, Device Started event
def device_started():
    response=serial_port_read(5,1)
    return response=='\x19\x05\x00\x00\x00'

# Check the parameters (passed on the Command Line)
def check_parameter(param):
    try:
        sys.argv.index(param)
        return True
    except:
        return False

# Exit
def exit(error, comPortOpen):

    if comPortOpen:
        ser.close()

    if error:
        sys.exit(1)
    else:
        sys.exit(0)

# Some default values
success = False
serialport = '/dev/ttyS1'
baudrate=3000000
wait_response = False
prevent_glitch = 0

# Main function

# Check parameters
if check_parameter('-b'):
    baudrate=sys.argv[sys.argv.index('-b')+1]

if check_parameter('-d'):
    serialport=sys.argv[sys.argv.index('-d')+1]

if check_parameter('-w'):
    wait_response = True

if check_parameter('-p'):
    prevent_glitch = 1

#open Com Port
comPortOpen = serial_port_open(serialport,baudrate)
if comPortOpen:
    # Wait to receive the, optional, Device Started event
    if device_started():
        print('device_started(optional): received')
    else:
        print('device_started(optional): not received')

    # Do PS-SWITCH
    ret = ps_switch(prevent_glitch)
    if ret == WicedResult.SUCCESS:
        if not wait_response:
            print('>> ps_switch req: Success')
            sys.exit(0)
    elif ret == WicedResult.NOT_AVAILABLE:
        print('>> ps_switch req: Reject, {}'. format(WicedResult(ret).name))
        sys.exit(1)
    else:
        print(">> ps_switch req: Fail, {1}(0x{0:02X})". format(ret, WicedResult(ret).name))
        sys.exit(255)

    # Wait PS-SWITCH response
    if wait_response:
        ret = ps_switch_wait()
        if ret == SwitchResult.WICED_BT_LRAC_SWITCH_SUCCESS:
            print('>> ps_switch: Success')
            sys.exit(0)
        elif ret == SwitchResult.WICED_BT_LRAC_SWITCH_NOT_READY:
            print('>> ps_switch: Reject, {}'. format(SwitchResult(ret).name))
            sys.exit(1)
        elif ret == SwitchResult.WICED_BT_LRAC_SWITCH_BUSY:
            print('>> ps_switch: Busy, {}'. format(SwitchResult(ret).name))
            sys.exit(1)
        elif ret == SwitchResult.WICED_BT_LRAC_SWITCH_READY_TOO_LONG:
            print('>> ps_switch: Ready too long, {}'. format(SwitchResult(ret).name))
            sys.exit(2)
        elif ret == SwitchResult.WICED_BT_LRAC_SWITCH_FORCE_ABORT:
            print('>> ps_switch: Force abort, {}'. format(SwitchResult(ret).name))
            sys.exit(3)
        else:
            print(">> ps_switch: Fail, {1}(0x{0:02X})". format(ret, SwitchResult(ret).name))
            sys.exit(255)

else:
    print "Err: Couldn't open serial port", serialport
    exit(True, comPortOpen)
