# python 3
# This file contains all configurations and controls for the 4tronix MARS Rover in the basic 
# configuration as bought from 4tronix. 
# This includes:
#   6 motors, with separate left and right motors control (so no individual control of each motor, but per side)
#   4 servo's to set the corner wheel angles, controlled individually
#   1 servo to set the mast angle
#   1 distance sensor at the top of the mast
#   4 leds, controlled individually
# 
# This library can then be used in more high level programs to control the behaviour/logic
# 
# Modifications from base 4tronix setup:
# 1) The pin for the IR distance sensor has been changed from 23 to 5, see below
# 2) 48 instead of 16 EEROM memory locations used for servo calibration data
#

import RPi.GPIO as GPIO, sys, time, os, smbus, tty, termios, math, threading
from rpi_ws281x import *

# Global variables and configuration data
debugFlag=False      # used to print debug info to screen 
bus = smbus.SMBus(1) # I2C communication bus setup

maxRPM=80               # motor theoretical max rounds per minute at 100 percent power
wheelSize=0.042*math.pi # 0.042 cm wheel diameter 
driveType='Straight'

#======================================================================
# define SERVO channel numbers for servo front/rear/left/right and mast
servoFrontLeft=9
servoFrontRight=15
servoRearLeft=11
servoRearRight=13
servoMast=0

#======================================================================
# define MOTOR PIN connections
# Note all motors on one side are connected to the same PIN
# and are therefore driven at the same power setting
#
# Pins 16, 19 Left Motor Driver
# Pins 12, 13 Right Motor Driver
LeftMotorPin1 = 16
LeftMotorPin2 = 19
RightMotorPin1 = 12
RightMotorPin2 = 13
leftDirection=0
rightDirection=0

#======================================================================
# define LEDs
ledRearLeft=0
ledFrontLeft=1
ledFrontRight=2
ledRearRight=3
numLeds=4
defaultbrightness=20
leds=None

#======================================================================
# Define Sonar IR Pin (same pin for both Ping and Echo)
sonarPin = 24    # original 4tronix config is on pin 23, now moved to 5 to give more space in rotation of the mast

#======================================================================
# EEROM is used for permanent storage of data
# first 48 bytes are used for calibration offsets (4tronix base program used 16 for midpoint calibration only)
# remaining of the 1024 bytes are available for other purposes
EEROMbaseAddress = 0x50       # I2C base address for EEROM
servoOffsets = [0]*48         # python list of 48 positions to hold calibration data retrieved from EEROM

#======================================================================
PCAbaseAddress = 0x40    # Fixed base I2C Address of the PCA9685 chip (for SERVO control)
PCAbaseOffset = 6
I2C_INITIALIZED = False

#======================================================================
# one-time initialisation and cleanup functions
def initRover():
    global running,obstacleDetected,stopSonar,stopSensors    
    running=True
    obstacleDetected=False
    stopBlink=False
    stopSonar=False
    stopSensors=False
    print ("Initialising electronics ...")
    initI2C()
    loadServoOffsets()
    setWheelServosNeutral()
    setMast(0)
    defineStandardColors()
    initLEDs(numLeds,defaultbrightness)   
    initMotors()
    print ("Initialisation complete")

# cleanupRover(). Sets all motors and LEDs off and sets GPIO to standard values
def cleanupRover():
    global running,stopSonar,stopBlink,stopSensors
    print ('çleaning up')
    running = False
    stopMotors()
    setWheelServosSmooth(0,0,0,0)
    setMast(0)
    stopSonar=True
    stopBlink=True
    stopSensors=True
    if (leds != None):
        clearLeds()
    time.sleep(1.5)
    GPIO.cleanup()
   
#======================================================================
# Motor functions
#
# initMotors: initialize Pulse With Modulation on Motor Pins and define motor and wheel characteristics
def initMotors():
    global LeftMotorsForward,LeftMotorsReverse,RightMotorsForward,RightMotorsReverse,leftDirection,rightDirection

    # initialize GPIO
    GPIO.setwarnings(False)
    # use physical pin numbering
    GPIO.setmode(GPIO.BCM)

    # use pwm for motor outputs, set pwm frequency at 50, start with dutycycle (=power) 0 
    GPIO.setup(LeftMotorPin1, GPIO.OUT)
    LeftMotorsForward = GPIO.PWM(LeftMotorPin1, 50)
    LeftMotorsForward.start(0)

    GPIO.setup(LeftMotorPin2, GPIO.OUT)
    LeftMotorsReverse = GPIO.PWM(LeftMotorPin2, 50)
    LeftMotorsReverse.start(0)

    GPIO.setup(RightMotorPin1, GPIO.OUT)
    RightMotorsForward = GPIO.PWM(RightMotorPin1, 50)
    RightMotorsForward.start(0)

    GPIO.setup(RightMotorPin2, GPIO.OUT)
    RightMotorsReverse = GPIO.PWM(RightMotorPin2, 50)
    RightMotorsReverse.start(0)
    leftDirection=0
    rightDirection=0


# changeDrive : core function called for any change in drive mode

def changeDrive(driveType,powerpct,radiusCm):
    if not(obstacleDetected and powerpct>0):    # implements requested mode except if obstacle detected or power = 0
        if (driveType=='Straight'):
            setWheelServosSmooth(0,0,0,0)
            setMotors(powerpct)  
        elif (driveType=='Arc'):
            Ackermandrive(driveType,powerpct,radiusCm)
        elif (driveType=='Spin'):
            Ackermandrive(driveType,powerpct,radiusCm)
         
def setMotors(powerpct):
    brakeMotorsIfNeeded(powerpct,powerpct)
    setLeftMotor(powerpct)
    setRightMotor(powerpct)
        
def stopMotors():
    setLeftMotor(0)
    setRightMotor(0)
    
def brakeMotorsIfNeeded(leftPowerpct,rightPowerpct):
    if ((leftDirection == 1 and leftPowerpct<0) or (leftDirection== -1 and leftPowerpct>0) or
        (rightDirection == 1 and rightPowerpct<0) or (rightDirection== -1 and rightPowerpct>0)):
        stopMotors()
        time.sleep(0.2)

def setLeftMotor(leftPowerpct):
    global leftDirection
    if (leftPowerpct<0):
        LeftMotorsForward.ChangeDutyCycle(0)
        LeftMotorsReverse.ChangeDutyCycle(-1*leftPowerpct)
        leftDirection = -1
    elif (leftPowerpct>0):
        LeftMotorsForward.ChangeDutyCycle(leftPowerpct)
        LeftMotorsReverse.ChangeDutyCycle(0)
        leftDirection = 1
    elif (leftPowerpct==0):
        LeftMotorsForward.ChangeDutyCycle(0)
        LeftMotorsReverse.ChangeDutyCycle(0)
        leftDirection = 0
    
def setRightMotor(rightPowerpct):
    global rightDirection
    if (rightPowerpct<0):
        RightMotorsForward.ChangeDutyCycle(0)
        RightMotorsReverse.ChangeDutyCycle(-1*rightPowerpct)
        rightDirection = -1
    elif (rightPowerpct>0):
        RightMotorsForward.ChangeDutyCycle(rightPowerpct)
        RightMotorsReverse.ChangeDutyCycle(0)
        rightDirection = 1
    elif (rightPowerpct==0):
        RightMotorsForward.ChangeDutyCycle(0)
        RightMotorsReverse.ChangeDutyCycle(0)
        rightDirection = 0 

# End of Motor Functions
#======================================================================


#======================================================================
# Servos
#======================================================================
def setServo(Servo, Degrees):
    # to calculate PWM value for Degrees, taking into account calibration values
    # accepts Degrees from -90 to +90
    validDegrees=False
    if Degrees>=-90 and Degrees<=90:
        validDegrees=True
        # first get calibration values for Servo
        midOffset=servoOffsets[Servo]       # offset relative to standard PWMvalue 375 for 0' mid position
        rightOffset=servoOffsets[Servo+16]  # offset relative to standard PWMvalue 575 for 90' right position
        leftOffset=servoOffsets[Servo+32]   # offset relative to standard PWMvalue 175 for 90' left position

        # calculate PWM value for either right or left quadrant movement
        if Degrees>= 0:
            PWMvalue=int((375+midOffset) + (Degrees/90 * (575+rightOffset-375-midOffset)))
        else:
            PWMvalue=int((375+midOffset) + (Degrees/90 * (375+midOffset-175-leftOffset)))
        writePWM(Servo, PWMvalue)
    return validDegrees

def setWheelServosNeutral():
    # set all wheel servo's at 0 degrees
    global currentFLdegrees,currentFRdegrees,currentRLdegrees,currentRRdegrees
    setServo(servoFrontLeft,0)
    setServo(servoFrontRight,0)
    setServo(servoRearLeft,0)
    setServo(servoRearRight,0)
    currentFLdegrees=0
    currentFRdegrees=0
    currentRLdegrees=0
    currentRRdegrees=0
    
def setWheelServosSmooth(FLdegrees,FRdegrees,RLdegrees,RRdegrees):
    # move to specified degrees in small steps to get a smooth behaviour
    global currentFLdegrees,currentFRdegrees,currentRLdegrees,currentRRdegrees
    FLdiff=FLdegrees-currentFLdegrees
    FRdiff=FRdegrees-currentFRdegrees
    RLdiff=RLdegrees-currentRLdegrees
    RRdiff=RRdegrees-currentRRdegrees
    maxDiff=max(math.fabs(FLdiff),math.fabs(FRdiff),math.fabs(RLdiff),math.fabs(RRdiff))
    if maxDiff>2:
        FLdiff=FLdiff/maxDiff
        FRdiff=FRdiff/maxDiff
        RLdiff=RLdiff/maxDiff
        RRdiff=RRdiff/maxDiff
        for step in range(math.trunc(maxDiff/2)):
            setServo(servoFrontLeft,currentFLdegrees+step*2*FLdiff)
            setServo(servoFrontRight,currentFRdegrees+step*2*FRdiff)
            setServo(servoRearLeft,currentRLdegrees+step*2*RLdiff)
            setServo(servoRearRight,currentRRdegrees+step*2*RRdiff)
            time.sleep(0.01)
        currentFLdegrees=FLdegrees
        currentFRdegrees=FRdegrees
        currentRLdegrees=RLdegrees
        currentRRdegrees=RRdegrees
    setServo(servoFrontLeft,currentFLdegrees)
    setServo(servoFrontRight,currentFRdegrees)
    setServo(servoRearLeft,currentRLdegrees)
    setServo(servoRearRight,currentRRdegrees)

def setServosNeutral():
    for i in range(16):
        setServo(i,0)

def stopServos():
    for i in range(16):
        stopPWM(i)

# Load all servo Offsets
def loadServoOffsets():
    for idx in range(48):
        servoOffsets[idx] = rdEEROM(idx)

# Save all servo Offsets
def saveServoOffsets():
    for idx in range(48):
        wrEEROM(idx, servoOffsets[idx])

def matchMastToDrive(driveType,direction,radiusCm):
    # mast direction follows drive direction
    if driveType=='Spin':
        mastDegrees=45
        setServo(servoMast,-1*direction*mastDegrees)
    elif driveType=='Straight':
        mastDegrees=0
        setServo(servoMast,mastDegrees)
    else:
        mastDegrees=((math.fabs(radiusCm)-200)/8)
        if radiusCm>0:
            setServo(servoMast,mastDegrees)
        else:
            setServo(servoMast,-1*mastDegrees)  

def setMast(mastDegrees):
    if (mastDegrees>-90 and mastDegrees<90):
        setServo(servoMast,mastDegrees)
           
# End of Servo Functions
#======================================================================

#======================================================================
# PCA9685 Library for PWM channel setting, used by higher level Servo methods
#======================================================================
# The PCA9685 is a 16 channel servo PWM driver controlled via I2C.
#
# These methods just send the requested PWM value to the servo channel.
# The calculation of the correct PWM value must be done by the calling program.
# The calling program needs to take into account the characteristics of the servo,
# such as calibration values to reach -90', 0' and +90' angles.
#======================================================================

def initI2C():
    global I2C_INITIALIZED
    try:
        bus.write_byte_data(PCAbaseAddress, 1, 0x04)         # set Mode2 outputs to push-pull
        mode1 = bus.read_byte_data(PCAbaseAddress, 0)        # get current Mode1 register
        mode1 &= 0x7f                             # ignore the reset bit
        mode1 |= 0x10                             # set Sleep bit
        bus.write_byte_data(PCAbaseAddress, 0, mode1)        # sleep
        bus.write_byte_data(PCAbaseAddress, 254, 101)        # set prescaler
        mode1 &= 0xef                             # clear Sleep bit
        bus.write_byte_data(PCAbaseAddress, 0, mode1 | 0x80) # wake up
        time.sleep(0.005)
        I2C_INITIALIZED = True
    except:
        print ("Error: No I2C Detected")


def writePWM(channel, PWMvalue):
    # to set the channel at a given PWM dutycyle value
    if I2C_INITIALIZED:
        if (channel>=0) and (channel<=16):
            start = 0
            stop = PWMvalue
            bus.write_byte_data(PCAbaseAddress, PCAbaseOffset + channel*4 + 0, start & 0xff)
            bus.write_byte_data(PCAbaseAddress, PCAbaseOffset + channel*4 + 1, start >> 8)
            bus.write_byte_data(PCAbaseAddress, PCAbaseOffset + channel*4 + 2, stop & 0xff)
            bus.write_byte_data(PCAbaseAddress, PCAbaseOffset + channel*4 + 3, stop >> 8)
    else:
        print ("Error: I2C for PWM channel not initialized")

def stopPWM(channel):
    if I2C_INITIALIZED:
        if (channel>=0) and (channel<=16):
            bus.write_byte_data(PCAbaseAddress, PCAbaseOffset + channel*4 + 0, 0)
            bus.write_byte_data(PCAbaseAddress, PCAbaseOffset + channel*4 + 1, 0)
            bus.write_byte_data(PCAbaseAddress, PCAbaseOffset + channel*4 + 2, 0)
            bus.write_byte_data(PCAbaseAddress, PCAbaseOffset + channel*4 + 3, 0)   
    else:
        print ("Error: I2C for PWM channel not initialized")
#======================================================================
# end of PCA9685 
#======================================================================

#======================================================================
# EEROM Functions
# adapted from original: now uses 48 bits to store 3 offsets per servo
#
# First 48 bytes are used for servo offsets (signed bytes)
#     first 16 for midpoint offsets
#     next 16 for 90' right offsets
#     last 16 for 90' left offsets
# these values were obtained from calibration

# Low level read function. Reads data from actual Address
def rdEEROM(Address):
    bus.write_i2c_block_data(EEROMbaseAddress, Address >> 8, [Address & 0xff])
    return ((bus.read_byte(EEROMbaseAddress) + 0x80) & 0xff) - 0x80  # sign extend

# Low level write function. Writes Data to actual Address
def wrEEROM(Address, Data):
    bus.write_i2c_block_data(EEROMbaseAddress, Address >> 8, [Address & 0xff, Data])
    time.sleep(0.01)

# General Read Function. Ignores first 48 bytes since those are reserved for offsets
def readEEROM(Address):
    return rdEEROM(Address + 48)

# General Write Function. Ignores first 48 bytes sicne those are reserved for offsets
def writeEEROM(Address, Data):
    wrEEROM(Address + 48, Data)
            
# End of EEROM Functions
#======================================================================

# Initialise LEDs
def initLEDs(numLeds,brightness):
    global leds, stopBlink
    leds = Adafruit_NeoPixel(numLeds, 18, 800000, 5, False, brightness)
    leds.begin()
    stopBlink=False

#======================================================================
# RGB LED Functions
#
def setAllLeds(color):
    for i in range(numLeds):
        setLed(i, color)   

def setLed(ledID, color):
    if (ledID <= numLeds):
        leds.setPixelColor(ledID, color)
        leds.show()

def showLeds():
    leds.show()

def clearLeds():
    for i in range(numLeds):
        setLed(i, 0)

def fromRGB(red, green, blue):
        return ((int(red)<<16) + (int(green)<<8) + blue)

def toRGB(color):
        return (((color & 0xff0000) >> 16), ((color & 0x00ff00) >> 8), (color & 0x0000ff))

def defineStandardColors():
     #define standard LED colors
    global RED,ORANGE,GREEN,BLUE,BLACK,WHITE
    RED   =fromRGB(255,0,0)
    ORANGE=fromRGB(255,255,0)
    GREEN =fromRGB(0,255,0)
    BLUE  =fromRGB(0,0,255)
    BLACK =fromRGB(0,0,0)
    WHITE =fromRGB(255,255,255)

def matchLedsToDrive(driveType,direction,powerpct,radiusCm):
    global stopBlink
    if powerpct==0:
        setAllLeds(WHITE)
    elif driveType=='Spin':
        setAllLeds(BLUE)
    elif direction==1:
        setAllLeds(GREEN)
    elif direction==-1:
        setAllLeds(GREEN)          
        setLed(ledRearLeft,WHITE)
        setLed(ledRearRight,WHITE) 
    if powerpct!=0:
        if radiusCm>0 and radiusCm<200:
            # start right blink unless already blinking
            alreadyBlinking=not(threading.activeCount()==2) # without blinking 2 threads are active: main and obstacle detection
            if not(alreadyBlinking):
                blinkLedNoWait(ledFrontRight,ORANGE,4,direction)
                blinkLedNoWait(ledRearRight,ORANGE,4,direction)
        elif radiusCm<0 and radiusCm>-200:
            # start left blink unless already blinking
            alreadyBlinking=not(threading.activeCount()==2) # without blinking 2 threads are active: main and obstacle detection
            if not(alreadyBlinking):
                blinkLedNoWait(ledFrontLeft,ORANGE,4,direction)
                blinkLedNoWait(ledRearLeft,ORANGE,4,direction)
        elif (radiusCm==0 or radiusCm==200 or radiusCm==-200):
            # stop any active blinking, send stop signal to blink threads
            stopBlink=True
          
def blinkLed(LedID, color, blinkPerSecond, direction):
    # blink the LEDs, called from blinkLedNoWait in a separate thread
    ledStatus='on'
    setLed(LedID,color)
    while not stopBlink:
        time.sleep(1/(blinkPerSecond*2))
        if ledStatus=='on':
            ledStatus='off'
            setLed(LedID, BLACK)
        else:
            ledStatus='on'    
            setLed(LedID, color)
    if direction==1:
        setLed(LedID,GREEN)
    elif direction==-1:
        if (LedID==ledFrontRight or LedID==ledFrontLeft):
            setLed(LedID,GREEN)
        else:           
            setLed(LedID,WHITE)

def blinkLedNoWait(LedID, color, blinkPerSecond, direction):
    # launch a separate thread to start LED blinking
    global stopBlink
    stopBlink=False
    thread=threading.Thread(target=blinkLed, args=(LedID,color,blinkPerSecond,direction))    
    thread.start()   
#
# End of RGB LED Functions
#======================================================================


#======================================================================
# UltraSonic Functions
#
def getSonarDistance(): # default to front sensor
    # getSonarDistance(). Returns the distance in cm to the nearest reflecting object. 0 == no object
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(sonarPin, GPIO.OUT)
    # Send 10us pulse to trigger
    GPIO.output(sonarPin, True)
    time.sleep(0.00001)
    GPIO.output(sonarPin, False)
    start = time.time()
    count=time.time()
    GPIO.setup(sonarPin,GPIO.IN)
    while GPIO.input(sonarPin)==0 and time.time()-count<0.1:
        start = time.time()
    count=time.time()
    stop=count
    while GPIO.input(sonarPin)==1 and time.time()-count<0.1:
        stop = time.time()
    # Calculate pulse length
    elapsed = stop-start
    # Distance pulse travelled in that time is time
    # multiplied by the speed of sound 34000(cm/s) divided by 2
    distance = elapsed * 17000
    return distance

def detectObstacle():
    # detect obstacle (=close than 20 cm), called from startObstacleDetection in a separate thread
    global obstacleDetected
    while not stopSonar:
        sonarDistance=getSonarDistance()
        if sonarDistance<=20:
            obstacleDetected=True
            stopMotors()
            setLed(ledFrontLeft,RED)
            setLed(ledFrontRight,RED)
        else:
            obstacleDetected=False
        time.sleep(1)

def startObstacleDetection():
    # launch obstacle detection in a separate thread
    global stopSonar,obstacleDetected
    stopSonar=False
    obstacleDetected=False
    thread=threading.Thread(target=detectObstacle,)    
    thread.start()   

# End of UltraSonic Functions
#======================================================================

#======================================================================
# keyboard functions, so robot can be controlled using keyboard
#        
# read single char from the standard keyboard    
def readchar():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    if ch == '0x03':
        raise KeyboardInterrupt
    return ch

# read char from the standard keyboard, with check for arrow key
def readkey(getchar_fn=None):
    getchar = getchar_fn or readchar
    c1 = getchar()
    if ord(c1) != 0x1b:  # the escape key
        return c1
    c2 = getchar()
    if ord(c2) != 0x5b:  # the [ key ?
        return c1
    c3 = getchar()
    return chr(0x10 + ord(c3) - 65)  # 16=Up, 17=Down, 18=Right, 19=Left arrows

# End of single character reading
#======================================================================

def Ackermandrive(driveType,powerpercent,turningRadiusCm):
    # Function to calculate power and angles for each wheel in line with Ackermann steering geometry
    # This geometry avoids wheel slippage and reduces wear (check wikipedia)
    # 
    # Unfortunately in the 4tronix rover power cannot be controlled per wheel, but only per side.

    # powerpercent is between -100 (reverse) and +100 (forward) percent

    turningRadiusM=turningRadiusCm/100
   
    # turningRadius is the radius of the turning circle, measured from center of circle to center of rover, in meters
    # turningRadius positive is turn right, negative is turn left
    # use positive turningRadius in calculations (turning to right) and apply correction to sign afterwards if not

    # assume Front Left wheel is farthest away from center of driving circle (=gets max powerpercent) and calculate other powerpercentages relative from this
    # if this is not the case, adapt the clauclations to set another wheel to 100% of requested powerpercent and adapt each last line on formula blocks below

    # physical measurements of MARS Rover, in meters
    # defined for each servo and wheel even though setup is symmetrical left/right

    # 1: for wheel Front Left (with servo)
    distFLws=0.026 # distance center of wheel to center of servo
    distFLxs=0.056 # distance center Rover to center servo (width)
    distFLys=0.08  # distance center Rover to center servo (length)

    # 2: for wheel Middle Left (without servo)
    distMLxs=0.085 # distance center Rover to center wheel (width)

    # 3: for wheel rear Left (with servo)
    distRLws=0.026 # distance center of wheel to center of servo
    distRLxs=0.058 # distance center Rover to center servo (width)
    distRLys=0.076 # distance center Rover to center servo (length)

    # 4: for wheel Front Right (with servo)
    distFRws=0.026 # distance center of wheel to center of servo
    distFRxs=0.056 # distance center Rover to center servo (width)
    distFRys=0.08  # distance center Rover to center servo (length)

    # 5: for wheel Middle Right (without servo)
    distMRxs=0.085 # distance center Rover to center servo (width)

    # 6: for wheel rear Right (with servo)
    distRRws=0.026 # distance center of wheel to center of servo
    distRRxs=0.058 # distance center Rover to center servo (width)
    distRRys=0.076 # distance center Rover to center servo (length)

    # wheel dimension and motor/gearbox rpm 
    # wheelsize=0.042 #diameter
    # wheelcircumphere=math.pi*wheelSize
    # maxRPMMotor=80
    
    # calculate individual motor powerpercent and servo angle setting

    # note plus signs in below formulas !

    FLangle=math.atan(distFLys/(math.fabs(turningRadiusM)+distFLxs))         # angle from center of driving circle (depending on turningRadiusM) to center of wheel
    distFLxws=math.cos(FLangle)*distFLws                             # width center of wheel to center of servo, depending on angle
    distFLyws=math.sin(FLangle)*distFLws                             # length center of wheel to center of servo, depending on angle
    FLturningRadiusM=math.hypot((math.fabs(turningRadiusM)+distFLxs+distFLxws),(distFLys+distFLyws))    # turningRadiusM from center of driving circle to center of wheel
    FLdegrees=math.degrees(FLangle)                          # wheel angle in degrees (turn wheel right)
    FLpowerpercent=powerpercent                                            # set Front Left powerpercent to requested powerpercent

    MLturningRadiusM=math.fabs(turningRadiusM)+distMLxs                          # turningRadiusM from center of driving circle to center of wheel
    MLpowerpercent=MLturningRadiusM/FLturningRadiusM*powerpercent                          # set Middle Left powerpercent relative to Front Left powerpercent

    RLangle=math.atan(distRLys/(math.fabs(turningRadiusM)+distRLxs))         # angle from center of driving circle (depending on turningRadiusM) to center of wheel
    distRLxws=math.cos(RLangle)*distRLws                             # width center of wheel to center of servo, depending on angle
    distRLyws=math.sin(RLangle)*distRLws                             # length center of wheel to center of servo, depending on angle
    RLturningRadiusM=math.hypot((math.fabs(turningRadiusM)+distRLxs+distRLxws),(distRLys+distRLyws))    # turningRadiusM from center of driving circle to center of wheel
    RLdegrees=-1*math.degrees(RLangle)                       # wheel angle in degrees (turn wheel left) note the minus sign !
    RLpowerpercent=RLturningRadiusM/FLturningRadiusM*powerpercent                          # set rear Left powerpercent relative to Front Left powerpercent

    # note minus signs in below formulas !

    FRangle=math.atan(distFRys/(math.fabs(turningRadiusM)-distFRxs))         # angle from center of driving circle (depending on turningRadiusM) to center of wheel
    distFRxws=math.cos(FRangle)*distFRws                             # width center of wheel to center of servo, depending on angle
    distFRyws=math.sin(FRangle)*distFRws                             # length center of wheel to center of servo, depending on angle
    FRturningRadiusM=math.hypot((math.fabs(turningRadiusM)-distFRxs-distFRxws),(distFRys-distFRyws))    # turningRadiusM from center of driving circle to center of wheel
    FRdegrees=math.degrees(FRangle)                          # wheel angle in degrees (turn wheel right)
    FRpowerpercent=FRturningRadiusM/FLturningRadiusM*powerpercent                          # set Front Right powerpercent relative to Front Left powerpercent

    MRturningRadiusM=math.fabs(turningRadiusM)-distMRxs                          # turningRadiusM from center of driving circle to center of wheel
    MRpowerpercent=MRturningRadiusM/FLturningRadiusM*powerpercent                          # set Middle Right powerpercent relative to Front Left powerpercent

    RRangle=math.atan(distRRys/(math.fabs(turningRadiusM)-distRRxs))         # angle from center of driving circle (depending on turningRadiusM) to center of wheel
    distRRxws=math.cos(RRangle)*distRRws                             # width center of wheel to center of servo, depending on angle
    distRRyws=math.sin(RRangle)*distRRws                             # length center of wheel to center of servo, depending on angle
    RRturningRadiusM=math.hypot((math.fabs(turningRadiusM)-distRRxs-distRRxws),(distRRys-distRRyws))    # turningRadiusM from center of driving circle to center of wheel
    RRdegrees=-1*math.degrees(RRangle)                       # wheel angle in degrees (turn wheel left) note the minus sign !
    RRpowerpercent=RRturningRadiusM/FLturningRadiusM*powerpercent                          # set rear Right powerpercent relative to Front Left powerpercent

    if (turningRadiusM < 0):                                         # switch all wheel settings left/right around if turningRadiusM is negative (turn left)
        tempstore=-1*FLdegrees
        FLdegrees=-1*FRdegrees
        FRdegrees=tempstore

        tempstore=-1*RLdegrees
        RLdegrees=-1*RRdegrees
        RRdegrees=tempstore
        
        tempstore=FLpowerpercent
        FLpowerpercent=FRpowerpercent
        FRpowerpercent=tempstore

        tempstore=MLpowerpercent
        MLpowerpercent=MRpowerpercent
        MRpowerpercent=tempstore

        tempstore=RLpowerpercent
        RLpowerpercent=RRpowerpercent
        RRpowerpercent=tempstore

    if (turningRadiusM == 0):                                        # spin around center, make left wheels turn in opposite direction
        FRpowerpercent=-1*FRpowerpercent
        MRpowerpercent=-1*MRpowerpercent
        RRpowerpercent=-1*RRpowerpercent

    brakeMotorsIfNeeded(FLpowerpercent,FRpowerpercent)
    setWheelServosSmooth(FLdegrees,FRdegrees,RLdegrees,RRdegrees)
    setLeftMotor(FLpowerpercent)
    setRightMotor(FRpowerpercent)
    # only FLpowerpercent and FRpowerpercent used since power can only be set per side, not per wheel
    











