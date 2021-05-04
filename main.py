import smbus, time, math, threading
import RPi.GPIO as GPIO
import cv2 as cv

class RepeatTimer(threading.Timer):
    def run(self):
        while not self.finished.wait(self.interval):
            self.function(*self.args, **self.kwargs)

#This class is made by - https://github.com/momoru-kun/AHT10
class AHT10:
    CONFIG = [0x08, 0x00]
    MEASURE = [0x33, 0x00]

    # bus - your I2C bus. You can watch it with "ls /dev | grep i2c-" command. If no output, enable I2C in raspi-config
    # addr - AHT10 I2C address. Can be switched by change resistor position on the bottom of your board. Default is 0x38
    #     You can set in to 0x39
    def __init__(self, bus, addr=0x38):
        self.bus = smbus.SMBus(bus)
        self.addr = addr
        self.bus.write_i2c_block_data(self.addr, 0xE1, self.CONFIG)
        print("AHT initializing...")
        time.sleep(0.2) #Wait for AHT to do config (0.2ms from datasheet)

    # getData - gets temperature and humidity
    # returns tuple of collected data. getData[0] is Temp, getData[1] is humidity
    def getData(self):
        byte = self.bus.read_byte(self.addr)
        self.bus.write_i2c_block_data(self.addr, 0xAC, self.MEASURE)
        time.sleep(0.5)
        data = self.bus.read_i2c_block_data(self.addr, 0x00)
        temp = ((data[3] & 0x0F) << 16) | (data[4] << 8) | data[5]
        ctemp = ((temp*200) / 1048576) - 50
        hum = ((data[1] << 16) | (data[2] << 8) | data[3]) >> 4
        chum = int(temp * 100 / 1048576)
        return (ctemp, chum)

class Relay:
    def __init__(self):
        self.pin = 21
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin, GPIO.OUT)
        
    #Returns true if pin is high
    def PinChecker(self):
        if GPIO.input(self.pin):
            return True
        else:
            return False
    
    def TurnOn(self):
        if not self.PinChecker():
            GPIO.output(self.pin, GPIO.HIGH)
    
    def TurnOff(self):
        if self.PinChecker():
            GPIO.output(self.pin, GPIO.LOW)
    
    def CleanUp(self):
        self.TurnOff()
        GPIO.cleanup()

class PiCam:
    def __init__(self):
        #Initialize dependency classes
        self.AHT10 = AHT10(1)
        self.relay = Relay()

        #Init webcamera
        self.webcam = cv.VideoCapture(0)

        #Load cascade data
        self.cascade = cv.CascadeClassifier(cv.data.haarcascades + "haarcascade_frontalface_default.xml")
        
        self.temperature = 0
        self.temperatureString = ""
        self.UpdateTempString()

        #Recurring timer to update temperature every x seconds
        self.tempTimer = RepeatTimer(15, self.UpdateTempString)
        self.tempTimer.start()

        #Recurring timer to check if faces have been consistent
        self.faceCheck = 0

        self.faceTimer = RepeatTimer(3, self.FaceChecker)
        self.faceTimer.start()

    #Make sure that faces have been detected somewhat consistently
    def FaceChecker(self):
        if (self.faceCheck > 7):
            self.relay.TurnOn()
        else:
            self.relay.TurnOff()
        self.faceCheck = 0

    def UpdateTempString(self):
        self.temperature = int(self.AHT10.getData()[0])
        self.temperatureString = "{} Celsius".format(self.temperature)
        print(self.temperatureString)

    def MainLoop(self):
        while True:
            #Get frame from webcamera
            ret, frame = self.webcam.read()

            #Only do face detection if temp is over x celsius
            if(self.temperature > 23):
                gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
                faces = self.cascade.detectMultiScale(gray, 1.15, 6, minSize=(30,30))
                #Loop through faces! What a weird thing to write
                for (x, y, w, h) in faces:
                    cv.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 4)

                face_num = len(faces)
                if(face_num > 0):
                    self.faceCheck += 1

            #Temperature text
            font = cv.FONT_HERSHEY_SIMPLEX
            textPos = (5, 30)
            fontColor = (0,0,0)
            cv.putText(frame,self.temperatureString,textPos,font,1,fontColor,2)

            #Show the current frame on screen
            cv.imshow('Webcam', frame)

            #Break and stop timer if the space key is pressed
            if cv.waitKey(1) == 32:
                self.relay.CleanUp()
                self.tempTimer.cancel()
                self.faceTimer.cancel()
                cv.destroyAllWindows()
                break

if __name__ == "__main__":
    PiCam().MainLoop()
    