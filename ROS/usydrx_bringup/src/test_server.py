#!/usr/bin/env python3
# This is written using Python3
# Based on server code provided by Bill Porter
# Written by Travis Moscicki
# Last edit 11/9

#####################################
###############Purpose###############
##This script will print statements## 
##showcasing teams interaction with## 
########the RobotX 2022 Server#######
#####################################

import os
import operator
import socket
import random
import re
import socketserver
import sys
import threading
import time
import pytz
from datetime import datetime, date
from functools import reduce

local_tz = pytz.timezone('Australia/Sydney')

def utc_to_local(utc_dt):
    local_dt = utc_dt.replace(tzinfo=pytz.utc).astimezone(local_tz)
    return local_tz.normalize(local_dt) # .normalize might be unnecessary

def aslocaltimestr(utc_dt):
    return utc_to_local(utc_dt).strftime('%Y-%m-%d %H:%M:%S.%f %Z%z')

def calcchecksum(nmea_str):
    # this returns a 2 digit hexadecimal string to use as a checksum.
    sum = hex(reduce(operator.xor, map(ord, nmea_str), 0))[2:].upper()
    if len(sum) == 2:
        return sum
    else:
        return '0' + sum

def readNMEA(nmea_str):
    # parse NMEA string into dict of fields.
    # the data will be split by commas and accessible by index.
    NMEApattern = re.compile('''
        ^[^$]*\$?
        (?P<nmea_str>
            (?P<talker>\w{2})
            (?P<sentence_type>\w{3}),
            (?P<data>[^*]+)
        )(?:\\*(?P<checksum>[A-F0-9]{2}))
        [\\\r\\\n]*
        ''', re.X | re.IGNORECASE)
    # print('nmea_str is:')
    # print(nmea_str)
    match = NMEApattern.match(nmea_str)
    # print('match is:')
    # print(match)
    if not match:
        raise ValueError('Could not parse data:', nmea_str)

    # print('before group match')
    nmea_dict = {}
    nmea_str = match.group('nmea_str')
    nmea_dict['talker'] = match.group('talker').upper()
    nmea_dict['sentence_type'] = match.group('sentence_type').upper()
    nmea_dict['data'] = match.group('data').split(',')
    checksum = match.group('checksum')
    # check the checksum to ensure matching data.
    # print('before checksum match')
    nmea_str = match.group('nmea_str')
    if checksum != calcchecksum(nmea_str):
        raise ValueError('Checksum does not match: %s != %s.' %
        (checksum, calcchecksum(nmea_str)))
    # print('readNMEA completed successfully')
    return nmea_dict

def parse_enc(string):
    if string=='P':
        return 'Platypus'
    elif string=='C':
        return 'Crocodile'
    elif string=='T':
        return 'Turtle'
    else:
        return '<invalid/Bad animal reported>'

############################################################################################
## This class will hold all team information
class Team():
    # Team class public member data
    name = ''
    date = 0
    hbdate = 0
    time = 0
    hbtime = 0
    lat = 0.0
    NS = 'N'
    lon = 0.0
    EW = 'E'
    mode = '1'
    status = '1'
    entrance = '0'
    exit = '0'
    LBactive = 'N'
    LBpattern = ''
    CODpattern = ''
    DOKcolor = ''
    DOKshape = ''
    DELcolor = ''
    DELshape = ''
    #Heartbeat message parsing and story log
    def HRB(self, message, file):
        print('Heartbeat received')
        self.hbdate = message[0]
        self.hbtime = message[1]
        self.lat = message[2]
        self.NS = message[3]
        self.lon = message[4]
        self.EW = message[5]
        self.name = message[6]
        #write out to story log on mode change
        if self.mode != message[7]:
            if message[7] == '2':
                #write out to story log
                try:
                    print(self.name + ' Vehicle in Auto\n')
                    print(str(aslocaltimestr(datetime.utcnow()))+'\n'+self.name+' vehicle is autonomous. Run started.\n')
                except:
                    print('no story log file created.')
            elif message[7] == '1':
                print(self.name + 'vehicle is in manual mode. run ended.\n')
                try:
                    print(str(aslocaltimestr(datetime.utcnow()))+'\n'+self.name+' vehicle is in manual mode. run ended.\n')
                except:
                    print('no story log file created.')
        self.mode = message[7]
        self.status = message[8]

    #Gate message parsing and story log
    def GAT(self, message, file):
        print('Entrance and Exit Gates received')
        self.GAT_date = message[0]
        self.GAT_time = message[1]
        self.GAT_name = message[2]
        self.GAT_entrance = message[3]
        self.GAT_exit = message[4]
        #write out to story log
        try:
            print(str(aslocaltimestr(datetime.utcnow()))+'\n'+self.GAT_name+' reports\n')
            print('Reported entrance gate: '+self.GAT_entrance+'\n')
            print('Reported exit gate: '+self.GAT_exit+'\n')
        except:
            print('no story log file created.')

    def FOL(self, message, file):
        print('Follow the Path received')
        self.FOL_date = message[0]
        self.FOL_time = message[1]
        self.FOL_name = message[2]
        self.FOL_finished= message[3]
        #write out to story log
        try:
            print(str(aslocaltimestr(datetime.utcnow()))+'\n'+self.FOL_name+' reports\n')
            if self.FOL_finished=='1':
                print('Follow the path task in progress'+'\n')
            elif self.FOL_finished=='2':
                print('Follow the path task completed'+'\n')
            else:
                print('Error in Follow the path task reporting'+'\n')
        except:
            print('no story log file created.')


    #Wildlife encounter report and react
    def ENC(self, message, file):
        print('Wildlife encounter received')
        self.ENC_date = message[0]
        self.ENC_time = message[1]
        self.ENC_name = message[2]
        self.ENC_numDetected = message[3]
        self.ENC_first = message[4]
        self.ENC_second = message[5]
        self.ENC_third = message[6]

        theFirst=parse_enc(self.ENC_first)
        theSecond=parse_enc(self.ENC_second)
        theThird=parse_enc(self.ENC_third)

        try:
            print(str(aslocaltimestr(datetime.utcnow()))+'\n'+self.ENC_name+' reports ')
            print('Wildlife Encounter objects detected: '+self.ENC_numDetected+'.\n')
            print('Wildlife Encounter first object: '+ theFirst +'.\n')
            print('Wildlife Encounter second object: '+ theSecond +'.\n')
            print('Wildlife Encounter third object: '+ theThird +'.\n')
        except:
            print('no story log file created.')

    #Scan the code message parsing and story log
    def COD(self, message, file):
        print('Code received')
        self.COD_date = message[0]
        self.COD_time = message[1]
        self.COD_name = message[2]
        self.COD_pattern = message[3]
        try:
            print(str(aslocaltimestr(datetime.utcnow()))+'\n'+self.COD_name+' reports ')
            print('Detected pattern for Scan the Code is '+self.COD_pattern+'.\n')
        except:
            print('no story log file created.')

    #Docking message parsing and story log
    def DOK(self, message, file):
        print('Dock received')
        self.DOK_date = message[0]
        self.DOK_time = message[1]
        self.DOK_name = message[2]
        self.DOK_color = message[3]
        self.DOK_status= message[4]
        #convert color to readable text
        if self.DOK_color == 'R':
            color = 'Red'
        elif self.DOK_color == 'B':
            color = 'Blue'
        elif self.DOK_color == 'G':
            color = 'Green'
        else:
            color = '<invalid/no color reported>'
        try:
            print(str(aslocaltimestr(datetime.utcnow()))+'\n'+self.DOK_name+' reports ')
            print('Attempting to dock in ' + color + ' colored bay.\n')
        except:
            print('no story log file created.')
        #convert AMS status to readable text
        if self.DOK_status == '1':
            status = 'Docking'
        elif self.DOK_status == '2':
            status = 'Complete'
        else:
            status = '<invalid/no AMS status reported>'
        try:
            print(str(aslocaltimestr(datetime.utcnow()))+'\n'+self.DOK_name+' reports ')
            print('AMS has status of ' + status + ' in Detect and Dock.\n')
        except:
            print('no story log file created.')

    #Find and Fling message parsing and story log
    def FLG(self, message, file):
        print('Find and Fling received')
        self.FLG_date = message[0]
        self.FLG_time = message[1]
        self.FLG_name = message[2]
        self.FLG_color = message[3]
        self.FLG_status= message[4]
        #convert color to readable text
        if self.FLG_color == 'R':
            color = 'Red'
        elif self.FLG_color == 'B':
            color = 'Blue'
        elif self.FLG_color == 'G':
            color = 'Green'
        else:
            color = '<invalid/no color reported>'
        #write out to story log
        try:
            print(str(aslocaltimestr(datetime.utcnow()))+'\n'+self.FLG_name+' reports ')
            print('The color being targeted in Find and Fling is: ' + color + '\n')
        except:
            print('no story log file created.')

        #convert AMS status to readable text
        if self.FLG_status == '1':
            status = 'Scanning'
        elif self.FLG_status == '2':
            status = 'Flinging'
        else:
            status = '<invalid/no AMS status reported>'
        #write out to story log
        try:
            print(str(aslocaltimestr(datetime.utcnow()))+'\n'+self.FLG_name+' reports ')
            print('The AMS status in Find and Fling is: ' + status + '\n')
        except:
            print('no story log file created.')

    #UAV Replenishment message parsing and story log
    def UAV(self, message, file):
        print('Find and Fling received')
        self.UAV_date = message[0]
        self.UAV_time = message[1]
        self.UAV_name = message[2]
        self.UAV_uavStatus= message[3]
        self.UAV_itemStatus= message[4]
        #convert UAV Status to readable text
        if self.UAV_uavStatus== '1':
            theUavStatus = 'Stowed'
        elif self.UAV_uavStatus== '2':
            theUavStatus = 'Deployed'
        elif self.UAV_uavStatus== '3':
            theUavStatus = 'Faulted'
        else:
            theUavStatus= '<invalid/no UAV Status reported>'
        #write out to story log
        try:
            print(str(aslocaltimestr(datetime.utcnow()))+'\n'+self.UAV_name+' reports ')
            print('The UAV Status in UAV Replenishment is: ' + theUavStatus + '\n')
        except:
            print('no story log file created.')
        #convert Item Status to readable text
        if self.UAV_itemStatus== '0':
            theItemStatus = 'Not Picked Up'
        elif self.UAV_itemStatus== '1':
            theItemStatus = 'Picked Up'
        elif self.UAV_itemStatus== '2':
            theItemStatus = 'Delivered'
        else:
            theUavStatus= '<invalid/no item Status reported>'
        #write out to story log
        try:
            print(str(aslocaltimestr(datetime.utcnow()))+'\n'+self.UAV_name+' reports ')
            print('The Item Status in UAV Replenishment is: ' + theItemStatus + '\n')
        except:
            print('no story log file created.')

    #UAV Replenishment message parsing and story log
    def SAR(self, message, file):
        print('UAV Search and report received')
        self.SAR_date = message[0]
        self.SAR_time = message[1]
        self.SAR_FIRSTobjectReported = message[2]
        self.SAR_FIRSTobjectLatitude = message[3]
        self.SAR_FIRSTnsIndicator = message[4]
        self.SAR_FIRSTobjectLongitude = message[5]
        self.SAR_FIRSTewIndicator = message[6]
        self.SAR_SECONDobjectReported = message[7]
        self.SAR_SECONDobjectLatitude = message[8]
        self.SAR_SECONDnsIndicator = message[9]
        self.SAR_SECONDobjectLongitude = message[10]
        self.SAR_SECONDewIndicator = message[11]
        self.SAR_name = message[12]
        self.SAR_uavStatus= message[13]

        if self.SAR_uavStatus== '1':
            theUavStatus = 'Manual'
        elif self.SAR_uavStatus== '2':
            theUavStatus = 'Autonomous'
        elif self.SAR_uavStatus== '3':
            theUavStatus = 'Faulted'
        else:
            theUavStatus= '<invalid/no UAV Status reported>'
        #write out to story log
        try:
            print(str(aslocaltimestr(datetime.utcnow()))+'\n'+self.SAR_name+' reports ')
            print('In task UAV Replenishment')
            print('The first report object is: ' + self.SAR_FIRSTobjectReported)
            print('The Lat is: ' + self.SAR_FIRSTobjectLatitude + ' ' + self.SAR_FIRSTnsIndicator)
            print('The Long is: ' + self.SAR_FIRSTobjectLongitude + ' ' + self.SAR_FIRSTewIndicator)
            print('The second report object is: ' + self.SAR_SECONDobjectReported)
            print('The Lat is: ' + self.SAR_SECONDobjectLatitude + ' ' + self.SAR_SECONDnsIndicator)
            print('The Long is: ' + self.SAR_SECONDobjectLongitude + ' ' + self.SAR_SECONDewIndicator)
            print('UAV Status is: ' + theUavStatus + '\n')
        except Exception as e:
            print(e.args)
            print('no story log file created.')

#########################################################################################
## TCP Handler function
class MyTCPHandler(socketserver.StreamRequestHandler):
    # read TCP message. Pass it through parser.
    def handle(self):
        #create an object for the team. This object can be used by GUI
        self.team = Team()
        print('Getting first message from TCP')
        #This next line is directly from the socketserver.StreamRequestHandler example
        #read in the first message to get team name
        self.data = self.rfile.readline().strip().decode('utf-8')
        # print('self.data is: '+ self.data.decode('utf-8'))
        self.message = readNMEA(self.data)
        # print(message)
        getattr(self.team, self.message['sentence_type'])(self.message['data'], 0)
        print('Team '+self.team.name+' connected.')

        try:
            print('**********BEGINNING OF NEW LOG**********')
        except Exception as e:
            print(e.args)
            print('ERROR. NMEA message not logged.')
        while True:
            # this pulls off one line of data from the TCP message,
            # which should be the entire message.
            try:
                self.data = self.rfile.readline().strip().decode('utf-8')
                #this will check to see if the string is empty
                if(not self.data):
                    # print('waiting for good message')
                    time.sleep(0.1)
                else:
                    self.message = readNMEA(self.data)
                    self.myWrite()
            except KeyboardInterrupt:
                print('Team '+self.team.name+' disconnected.')
                break

    def myWrite(self):
        if self.message['talker'] == 'RX':
            try:
                #calls parsing function within 'team' object by sentence type
                # getattr(self.team, self.message['sentence_type'])(self.message['data'], self.STORY_LOG)
                getattr(self.team, self.message['sentence_type'])(self.message['data'], 0)
                self.name = self.team.name
            except Exception as e:
                print(e.args)
            #write out to raw log file
        else:
            raise ValueError('Invalid talker, got', message['talker'])

class ThreadedTCPServer(socketserver.ThreadingMixIn, socketserver.TCPServer): pass


###########################################################################
## End of functions, beginning of main

if __name__ == '__main__':
    # start the threads to listen to the boats.
    # this breaks off another thread for each TCP connection.
    HOST, PORT = '', 12345
    server = ThreadedTCPServer((HOST, PORT), MyTCPHandler)
    server_thread = threading.Thread(target = server.serve_forever)
    server_thread.daemon = True
    server_thread.start()
    print('Server Address:', socket.gethostbyname(socket.gethostname()))
    print('Connect to server on port:', PORT)

    while True:
        try:
            time.sleep(0.2)
        except KeyboardInterrupt:
            print('closing logs and exiting')
            break
