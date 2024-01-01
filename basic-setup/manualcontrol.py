# 4tronix Mars Rover Manual Control
#
# Based on the test software provided by 4tronix
#
# This program requires the file roverlib.py with all basic 
# control functions of the separate hardware components. 

import roverlib as rover
import math,time

debugflag=False # triggers debug print statements

print ("Tests the motors by using keys to control")
print ("w is straight forward, z is straight reverse")
print ("a is spinleft,         s is spinright")
print ("< is turn more left,   > is turn more right")
print ("arrow down is slow down, arrow up is speed up")
print (". to go straight")
print ("Press space bar to stop")
print ("Press Ctrl-C to end")
print

rover.initRover()

driveType='Straight'
direction=1
powerpct = 0
radiusCm = 200

rover.setAllLeds(rover.RED)
driveMode='manual'

# main loop
if driveMode=='manual':
    try:
        rover.startObstacleDetection()
        time.sleep(3)
        while True:
            keyp = rover.readkey()      
            if keyp == 'w' :
                # direction forward
                direction=1
                powerpct=direction*math.fabs(powerpct)
                rover.changeDrive(driveType,powerpct,radiusCm)
                if debugflag: print ('Forward power', powerpct)
            elif keyp == 'z' :
                # direction reverse
                direction=-1
                powerpct=direction*math.fabs(powerpct)
                rover.changeDrive(driveType,powerpct,radiusCm)
                if debugflag: print ('Reverse power', powerpct)
            elif ord(keyp) == 16:  # arrow up
                # go faster, positive or negative, unless maximum
                powerpct=direction*min(math.fabs(powerpct)+20,100)
                rover.changeDrive(driveType,powerpct,radiusCm)    
                if debugflag: print ('powerpct+', powerpct)
            elif ord(keyp) == 17: # arrow down
                # go slower, positive or negative, unless 0
                powerpct=direction*max(math.fabs(powerpct)-20,0)
                rover.changeDrive(driveType,powerpct,radiusCm)
                if debugflag: print ('powerpct-', powerpct)
            elif keyp == 's' :
                radiusCm = 0
                driveType='Spin'
                direction=1
                powerpct=direction*math.fabs(powerpct)
                rover.changeDrive(driveType,powerpct,radiusCm)
                if debugflag: print ('Spin Right', powerpct)
            elif keyp == 'a' :
                radiusCm = 0
                driveType='Spin'
                direction=-1
                powerpct=direction*math.fabs(powerpct)
                rover.changeDrive(driveType,powerpct,radiusCm)
                if debugflag: print ('Spin Left', powerpct)
            elif ord(keyp) == 19: # left arrow
                # more to the left, positive larger or negative smaller, smaller is sharper turn
                # radiusCm 200 or larger is considered straight drive
                if (driveType!='Spin'):
                    if radiusCm == 200:
                        radiusCm = -200
                    if radiusCm < -20 or radiusCm >= 20:
                        radiusCm = radiusCm + 10
                    if (radiusCm==200 or radiusCm==-200):
                        drivetype='Straight'
                        if debugflag: print('Straight')
                        rover.changeDrive(driveType,powerpct,radiusCm)
                    else:
                        driveType='Arc'    
                        if debugflag: print('Arc')
                        rover.changeDrive(driveType,powerpct,radiusCm)
                if debugflag: print ('radiusCm+', radiusCm)
            elif ord(keyp) == 18: # right arrow
                # more to the right, positive smaller or negative larger, smaller is sharper turn
                # radiusCm 200 or larger is considered straight drive
                if (driveType!='Spin'):
                    if radiusCm == -200:
                        radiusCm = 200
                    if radiusCm <= -20 or radiusCm > 20:
                        radiusCm = radiusCm - 10
                    if (radiusCm==200 or radiusCm==-200):
                        driveType='Straight'
                        if debugflag: print('Straight')
                        rover.changeDrive(driveType,powerpct,radiusCm)
                    else:
                        driveType='Arc'
                        if debugflag: print('Arc')    
                        rover.changeDrive(driveType,powerpct,radiusCm)
                if debugflag: print ('radiusCm+', radiusCm)
            elif keyp == '.':
                radiusCm = 200
                driveType='Straight'
                rover.changeDrive(driveType,powerpct,radiusCm)
                if debugflag: print ('Straight')
            elif keyp == ' ':
                powerpct = 0
                rover.changeDrive(driveType,powerpct,radiusCm)
                if debugflag: print ('Stop')
            elif ord(keyp) == 3:
                break

            #rover.matchLedsToDrive(driveType,direction,powerpct,radiusCm)
            #rover.matchMastToDrive(driveType,direction,radiusCm)
        
    except KeyboardInterrupt:
        pass

    finally:
        rover.cleanupRover()
    
