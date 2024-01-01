# Calibrates and saves Servo Offsets on MARS Rover
#
# extended from the 4tronix original to not only calibrate midpoint
# but also calibrate right and left 90' edge position 
# for more accurate servo positioning

import roverlib as rover

print ("Calibrate the servos on M.A.R.S. Rover")
print (" ")
print ("Select Servo to calibrate with '1', '2', '3', '4' or '5'")
print ("To be done without motors and wheels attached (to be able to reach full 90')")
print (" ")
print ("First use 'm' key to select midpoint position (straight)")
print ("Use the left and right arrow keys to fine tune the servo")
print (" ")
print ("Then use 'r' for right 90' edge or 'l' for left 90' edge")
print ("and use left and right arrow keys to fine tune the edge position")
print (" ")
print ("Press 's' to save the calibration data in the EEROM")
print ("Press Ctrl-C to exit without saving")
print ()

rover.initRover()

print ('Existing servo offsets')
print (rover.servoOffsets)
print ()

# default servo dutycycle values, reference for calibration values
midvalue=375
rightvalue=575
leftvalue=175

try:
    # start with midpoint of servo front left
    servo = rover.servoFrontLeft        # front left
    # first 16 memory positions are midpoint offsets, then 16 right offsets, then 16 left offsets
    positionoffset=0            

    value=midvalue+rover.servoOffsets[servo+positionoffset]
    rover.writePWM(servo,value) 
    print ('Servo: Front Left: Offset:', rover.servoOffsets[servo+positionoffset], sep='')
    
    rover.setAllLeds(rover.RED)
    rover.setLed(rover.ledFrontLeft, rover.GREEN)
    
    while True:
        key = rover.readkey()
        if key == '1':
            servo = rover.servoFrontLeft
            print ('Servo: Front Left: Offset:', rover.servoOffsets[servo+positionoffset], sep='')
            rover.setAllLeds(rover.RED)
            rover.setLed(rover.ledFrontLeft, rover.GREEN)
        elif key == '2':
            servo = rover.servoRearLeft
            print ('Servo: Rear Left: Offset:', rover.servoOffsets[servo+positionoffset], sep='')
            rover.setAllLeds(rover.RED)
            rover.setLed(rover.ledRearLeft, rover.GREEN)
        elif key == '3':
            servo = rover.servoFrontRight
            print ('Servo: Front Right: Offset:', rover.servoOffsets[servo+positionoffset], sep='')
            rover.setAllLeds(rover.RED)
            rover.setLed(rover.ledFrontRight, rover.GREEN)
        elif key == '4':
            servo = 13
            print ('Servo: Rear Right: Offset:', rover.servoOffsets[servo+positionoffset], sep='')
            rover.setAllLeds(rover.RED)
            rover.setLed(rover.ledRearRight, rover.GREEN)
        elif key == '5':
            servo = 0
            print ('Servo: Mast (is upside down) : Offset:', rover.servoOffsets[servo+positionoffset], sep='')
            rover.setAllLeds(rover.RED)
            rover.setLed(rover.ledFrontLeft, rover.BLUE)
            rover.setLed(rover.ledFrontRight, rover.BLUE)
        elif key == 'm':
            positionoffset=0
            print ('Servo ',servo, 'position midpoint')
        elif key == 'r':
            positionoffset=16
            print ('Servo ',servo, 'position right')
        elif key == 'l':
            positionoffset=32
            print ('Servo ',servo, 'position left')
        elif key == 'x' or key == '.':
            rover.stopServos()
            print ('Servo ', servo, ': Stop',sep='')
        elif ord(key) == 19: # left arrow
            rover.servoOffsets[servo+positionoffset] -= 1
        elif ord(key) == 18: # right arrow
            rover.servoOffsets[servo+positionoffset] += 1
        elif key == 's':
            print ('Saving servo offsets')
            rover.saveServoOffsets()
            break
        elif ord(key) == 3:
            break

        if positionoffset == 0:
            value=midvalue+rover.servoOffsets[servo+positionoffset]
        elif positionoffset == 16:
            value=rightvalue+rover.servoOffsets[servo+positionoffset]
        elif positionoffset == 32:
            value=leftvalue+rover.servoOffsets[servo+positionoffset]
        rover.writePWM(servo,value)
        print ('Servo ', servo,': Value ',value,': Offset:', rover.servoOffsets[servo+positionoffset], sep='')

except KeyboardInterrupt:
    print()

finally:
    rover.loadServoOffsets()
    print ()
    print ('New servo offsets')
    print (rover.servoOffsets)
    print ()
    rover.cleanupRover()
