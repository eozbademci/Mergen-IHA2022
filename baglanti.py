
from dronekit import connect, VehicleMode, mavutil, LocationGlobalRelative, Command
from time import sleep
from pymavlink import mavutil
import json

from math import radians
import math
import numpy as np
iha= connect("/dev/ttyACM0", baud=57600)
threshold=40
screen=[640,480]
speed=1
kirmizikontrol = False
sualdi = False
subirakti = False
direk1_b = False
direk1_c = False
iha.channels.overrides = {'6':950} #tamamen kapalı

su = {'lat' : 38.7896682, 'lon' : 30.4827552}

inis = {'lat' : 38.7899578, 'lon' : 30.4823107} 

direk1b =  {'lat':38.7893405, 'lon' : 30.4829675}
direk1c =  {'lat':38.7894827, 'lon' : 30.4834154}

direk2b =  {'lat':38.7903932, 'lon' : 30.4826201}
direk2c =  {'lat':38.7904935, 'lon' : 30.4830842}


direk1bkonum = LocationGlobalRelative(direk1b['lat'], direk1b['lon'], 10)
direk1ckonum = LocationGlobalRelative(direk1c['lat'], direk1c['lon'], 10)
def get_distance_metres(aLocation1, aLocation2):
    
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def takeoff(irtifa):
        
        
        print("İha Arm Edilebilir")
        iha.mode=VehicleMode("GUIDED")
        sleep(1)
        print(str(iha.mode)+"moduna alindi.")
        iha.armed=True
        while iha.armed is not True:
            print("İha arm ediliyor...")
            sleep(1)
            
        print("İha arm edildi.")
        iha.simple_takeoff(irtifa)
        while True:
            a = iha.location.global_relative_frame.alt
            while not type(a)== float:
                a = iha.location.global_relative_frame.alt
            #if isintance(a,float):
            if iha.location.global_relative_frame.alt > irtifa * 0.96:
                sleep(1)
                break
            sleep(0.2)


def goto_position_target_relative_ned(north, east, down):
    
    msg = iha.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, # frame
        0b0000111111111000, # type_mask (only positions enabled)
        north, east, down,
        0, 0, 0, # x, y z velocity in m/s  (not used)
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    # send command to vehicle
    iha.send_mavlink(msg)

def velocity(velocity_x, velocity_y, velocity_z):
    msg = iha.message_factory.set_position_target_local_ned_encode(
    0, # time_boot_ms (not used)
    0, 0, # target system, target component
    mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
    0b0000111111000111, # type_mask (only speeds enabled)
    0, 0, 0, # x, y, z positions (not used)
    velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
    0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
    0, 0)

def condition_yaw(heading, relative=False):
    
    if relative:
        is_relative = 1 #yaw relative to direction of travel
    else:
        is_relative = 0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = iha.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    iha.send_mavlink(msg)

def konum_al():
    return iha.location.global_relative_frame

def ortala():
    alcalma = False
    while True:
        
        with open('/home/pi/Desktop/Test/data.json') as f:
            data = json.load(f)

        #GORUNTU_ISLEME_ 1
        print(data["x"], data["y"], data["tespit"])
        if int(data["x"] < 290):
            goto_position_target_relative_ned(0, 0.5, 0)
            
        elif int(data["x"] > 330):
            goto_position_target_relative_ned(0, -0.5, 0)

        elif int(data["y"] < 210 ):
            goto_position_target_relative_ned(-0.5, 0, 0)
        elif int(data["y"] > 250 ):
            goto_position_target_relative_ned(0.5, 0, 0)
        else: break

        
            
        sleep(2)
    
    
    
    
        

def gorev_ekle():
    global komut
    komut = iha.commands
    komut.clear()
    sleep(1)
    
    #TAKEOFF
    komut.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 10))
    
    #DIREK1
    komut.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, direk1b['lat'], direk1b['lon'], 10))
    komut.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, direk1c['lat'], direk1c['lon'], 10))
    
    #DIREK2
    komut.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, direk2c['lat'], direk2c['lon'], 10))
    komut.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, direk2b['lat'], direk2b['lon'], 10))
      
    #DIREK2
    komut.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, direk2c['lat'], direk2c['lon'], 10))
    komut.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, direk2b['lat'], direk2b['lon'], 10))
    
    #INIS
    komut.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, inis['lat'], inis['lon'], 10))
   


    #RTL
    komut.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, inis['lat'], inis['lon'], 10))
    
    komut.upload()
    print("Komutlar Yukleniyor...")



    
takeoff(10)
sleep(1)

gorev_ekle()
komut.next = 0
iha.mode = VehicleMode("AUTO")

while True:
    next_waypoint = komut.next
    print(f"Siradaki komut= {next_waypoint}")
    sleep(0.1)
    if kirmizikontrol == False:
        if next_waypoint == 3:
            sleep(3)
            iha.mode = VehicleMode("GUIDED")        
            point1 = LocationGlobalRelative(direk2c['lat'], direk2c['lon'], 10)
            iha.simple_goto(point1)
            iha.groundspeed = 2
            data = {
                'camera' : True,
                'red' : True
            }    
            j = json.dumps(data)
            with open("/home/pi/Desktop/Test/datagoruntu.json","w") as f:
                f.write(j)
            
            
            with open('/home/pi/Desktop/Test/data.json') as s:
                oku = json.load(s)
            while not oku['tespit']:
                with open('/home/pi/Desktop/Test/data.json') as s:
                    oku = json.load(s)
                print("Kirmizi alan araniyor")
                sleep(0.1)
            if kirmizikontrol == False:
                print("TESPIT EDILDI!!!!!...")
                ortala()            
                red_loc=konum_al()
                print(red_loc)
                iha.groundspeed = 4
                iha.mode = VehicleMode("AUTO")
                kirmizikontrol = True
                
    
    #SU ALMA
    
    elif next_waypoint == 5:
        if sualdi == False:
            iha.mode = VehicleMode("GUIDED")            
            sukonum = LocationGlobalRelative(su['lat'], su['lon'], 10)
            iha.simple_goto(sukonum)
            while sualdi == False:
                #print(get_distance_metres(konum_al(),sukonum))
                sleep(0.2)
                if get_distance_metres(konum_al(),sukonum) < 1:
                    data = {
                    'camera' : True,
                    'red' : False
                    }    
                    j = json.dumps(data)
                    with open("/home/pi/Desktop/Test/datagoruntu.json","w") as f:
                        f.write(j)
                    sleep(1)
                    ortala()
                    sleep(0.5)
                    goto_position_target_relative_ned(0, 0, 5)
                    sleep(7)
                    goto_position_target_relative_ned(0, 0, 2.5)                     
                    sleep(10)
                    iha.channels.overrides = {'6':1350} # 1 kademe acik
                    sleep(2)
                    goto_position_target_relative_ned(0, 0, -2.5)
                    sleep(5)
                    goto_position_target_relative_ned(0, 0, -5)
                    sleep(10)
                    sualdi = True

                    #iha.mode = VehicleMode("AUTO")

                    
                    data = {
                    'camera' : True,
                    'red' : True
                     }    
                    j = json.dumps(data)
                    with open("/home/pi/Desktop/Test/datagoruntu.json","w") as f:
                        f.write(j)
        
        elif direk1_b == False:
            iha.groundspeed = 1
            iha.simple_goto(direk1bkonum)            
            sleep(0.2)
            while direk1_b == False:
                
                if get_distance_metres(konum_al(),direk1bkonum) < 2:
                    print("girdi")
                    sleep(1)                
                    direk1_b = True
                    break
                sleep(0.1)

        elif direk1_c == False:
            
            iha.simple_goto(direk1ckonum)            
            sleep(0.2)
            while direk1_c == False:
                if get_distance_metres(konum_al(),direk1ckonum) < 2:
                    print(get_distance_metres(konum_al(),direk1ckonum))
                    sleep(1)                    
                    direk1_c = True
                    break
                sleep(0.1)
        elif subirakti == False:            
            iha.simple_goto(red_loc)
            #print(get_distance_metres(konum_al(),red_loc))
            while subirakti == False:
                sleep(1)
                if get_distance_metres(konum_al(),red_loc) < 2:
                    sleep(5)
                    ortala()
                    sleep(0.5)
                    goto_position_target_relative_ned(0, 0, 5)
                    sleep(5)
                    iha.channels.overrides = {'6':1900} # 2 kademe acik
                    sleep(1)
                    goto_position_target_relative_ned(0, 0, -5)
                    sleep(3)
                    subirakti = True
                    iha.groundspeed = 8
                    iha.mode = VehicleMode("AUTO")
    
        
    
    elif next_waypoint == 8:        
        print("Gorev bitti...")
        break

