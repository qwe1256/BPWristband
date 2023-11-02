import RPi.GPIO as GPIO
from smbus2 import SMBus
import time
from threading import Lock
#Reg Addr
INT_STATUS1_ADDR=0x00
INT_STATUS2_ADDR=0x01
INT_ENABLE1_ADDR=0x02
INT_ENABLE2_ADDR=0x03
FIFO_WRITE_PTR_ADDR=0x04
OVF_CNTR_ADDR=0x05
FIFO_READ_PTR_ADDR=0x06
FIFO_DATA_REG_ADDR=0x07
FIFO_CONFIG_ADDR=0x08
MODE_CONFIG_ADDR=0x09
SPO2_CONFIG_ADDR=0x0A
LED1_PA_ADDR=0x0C
LED2_PA_ADDR=0x0D
MULTI_LED_MODE1_ADDR=0x11
MULTI_LED_MODE2_ADDR=0x12
TEMP_INT_ADDR=0x1F
TEMP_FRA_ADDR=0x20
TEMP_EN_ADDR=0x21
REVISION_ID_ADDR=0xFE
PART_ID_ADDR=0xFF
# Mask
BIT7=0x80
BIT6=0x40
BIT5=0x20
BIT4=0x10
BIT3=0x08
BIT2=0x04
BIT1=0x02
BIT0=0x01
INT_STATUS1_A_FULL=BIT7
INT_STATUS1_PPG_READY=BIT6
INT_STATUS1_ALC_OVF=BIT5
INT_STATUS1_PWR_READY=BIT0
INT_STATUS2_TEMP_READY=BIT1
INT_ENABLE1_A_FULL_EN=BIT7
INT_ENABLE1_PPG_READY_EN=BIT6
INT_ENABLE1_ALC_OVF_EN=BIT5
INT_ENABLE2_TEMP_EN=BIT1
FIFO_WRITE_PTR=BIT4|BIT3|BIT2|BIT1|BIT0
OVF_CNTR=BIT4|BIT3|BIT2|BIT1|BIT0
FIFO_READ_PTR=BIT4|BIT3|BIT2|BIT1|BIT0
FIFO_DATA_REG=0xFF
FIFO_CONFIG_SMP_AVE=BIT7|BIT6|BIT5
FIFO_CONFIG_ROLL_OVER_EN=BIT4
FIFO_CONFIG_A_FULL=BIT3|BIT2|BIT1|BIT0
MODE_CONFIG_SHDN=BIT7
MODE_CONFIG_RESET=BIT6
MODE_CONFIG_MODE=BIT2|BIT1|BIT0
SPO2_CONFIG_ADC_RGE=BIT6|BIT5
SPO2_CONFIG_SR=BIT4|BIT3|BIT2
SPO2_CONFIG_LED_PW=BIT1|BIT0
LED1_PA=0xFF
LED2_PA=0xFF
SLOT2=BIT6|BIT5|BIT4
SLOT1=BIT2|BIT1|BIT0
SLOT4=BIT6|BIT5|BIT4
SLOT3=BIT2|BIT1|BIT0
TEMP_INT=0xFF
TEMP_FRA=BIT3|BIT2|BIT1|BIT0
TEMP_CONFIG_TEMP_EN=BIT0
REVISION_ID=0xFF
PART_ID=0xFF
#Part ID
PART_ID_VAL=0x15
#LSB
ADC_LSB_2048=7.81
ADC_LSB_4096=15.63
ADC_LSB_8192=31.25
ADC_LSB_16384=62.5
#Resolution
RESOLUTION_15=0x7FFF
RESOLUTION_16=0xFFFF
RESOLUTION_17=0x1FFFF
RESOLUTION_18=0x3FFFF

#Mode
PPGSENSOR_HR=0x02
PPGSENSOR_SPO2=0x03
PPGSENSOR_MLED=0x07

#ADC Range
PPGSENSOR_ADC_2048=0x00
PPGSENSOR_ADC_4096=0x20
PPGSENSOR_ADC_8192=0x40
PPGSENSOR_ADC_16384=0x60
#Sample Rate
PPGSENSOR_SR_50=0x00
PPGSENSOR_SR_100=0x04
PPGSENSOR_SR_200=0x08
PPGSENSOR_SR_400=0x0C
PPGSENSOR_SR_800=0x10
PPGSENSOR_SR_1000=0x14
PPGSENSOR_SR_1600=0x18
PPGSENSOR_SR_3200=0x1C

#Pulse Width
PPGSENSOR_PW_15=0x00
PPGSENSOR_PW_16=0x01
PPGSENSOR_PW_17=0x02
PPGSENSOR_PW_18=0x03
#Sample Average
PPGSENSOR_SMPAVG_NONE=0x00
PPGSENSOR_SMPAVG_2=0x20
PPGSENSOR_SMPAVG_4=0x40
PPGSENSOR_SMPAVG_8=0x60
PPGSENSOR_SMPAVG_16=0x80
PPGSENSOR_SMPAVG_32=0xA0
#Afull
PPGSENSOR_AFULL_32=0x0
PPGSENSOR_AFULL_31=0x01
PPGSENSOR_AFULL_30=0x02
PPGSENSOR_AFULL_29=0x03
PPGSENSOR_AFULL_28=0x04
PPGSENSOR_AFULL_27=0x05
PPGSENSOR_AFULL_26=0x06
PPGSENSOR_AFULL_25=0x07
PPGSENSOR_AFULL_24=0x08
PPGSENSOR_AFULL_23=0x09
PPGSENSOR_AFULL_22=0x0a
PPGSENSOR_AFULL_21=0x0b
PPGSENSOR_AFULL_20=0x0c
PPGSENSOR_AFULL_19=0x0d
PPGSENSOR_AFULL_18=0x0e
PPGSENSOR_AFULL_17=0x0f
#slot
PPGSENSOR_SLOT_NONE=0x00
PPGSENSOR_SLOT_LED1=0x01
PPGSENSOR_SLOT_LED2=0x02

rwLock=Lock()


#deviceBus=1
deviceAddr=0x57
intChannel=26
bus = SMBus(1)
def ppgInit(sampleAverage,mode,ADCRange,sampleRate,pluseWidth,powerLevel):
    readRegister(INT_STATUS1_ADDR)
    id=readRegister(PART_ID_ADDR)
    #print(id)
    if id!=PART_ID_VAL:
        print(type(id))
        raise Exception('Part ID Error')
    setSampleAverage(sampleAverage)
    setMode(mode)
    setADCRange(ADCRange)
    setSampleRate(sampleRate)
    setPulseWidth(pluseWidth)
    setAmplitude(LED1_PA_ADDR,powerLevel)
    setAmplitude(LED2_PA_ADDR,powerLevel)
    if mode==PPGSENSOR_MLED:
        val=readRegister(MULTI_LED_MODE1_ADDR)
        val=(val&(~SLOT1))|0x01
    resetFIFO()
    shutDown()
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(intChannel, GPIO.IN, pull_up_down=GPIO.PUD_UP)

def startMonitor():
    shutDown()
    readRegister(INT_STATUS1_ADDR)
    setAFULL(PPGSENSOR_AFULL_17)
    enableAFull()
    resetFIFO()
    wakeUp()
    #readRegister(INT_STATUS1_ADDR)

def stopMonitor():
    shutDown()
    resetFIFO()

def softReset():
    setBit(MODE_CONFIG_ADDR, MODE_CONFIG_RESET, BIT6)
    while readRegister(MODE_CONFIG_ADDR)&MODE_CONFIG_RESET:
        time.sleep(.5)

def shutDown():
    setBit(MODE_CONFIG_ADDR,MODE_CONFIG_SHDN,BIT7)
    
def wakeUp():
    setBit(MODE_CONFIG_ADDR,MODE_CONFIG_SHDN,0)

def enableAFull():
    setBit(INT_ENABLE1_ADDR,INT_ENABLE1_A_FULL_EN,0xff)

def disableAFull():
    setBit(INT_ENABLE1_ADDR,INT_ENABLE1_A_FULL_EN,0)

def enableDATARDY():
    setBit(INT_ENABLE1_ADDR, INT_ENABLE1_PPG_READY_EN, 0xff)

def disableDATARDY():
    setBit(INT_ENABLE1_ADDR, INT_ENABLE1_PPG_READY_EN, 0)

def enableALCOVF():
    setBit(INT_ENABLE1_ADDR, INT_ENABLE1_ALC_OVF_EN, 0xff)

def disableALCOVF():
    setBit(INT_ENABLE1_ADDR, INT_ENABLE1_ALC_OVF_EN, 0)

def enableDIETEMPRDY():
    setBit(INT_ENABLE2_ADDR, INT_ENABLE2_TEMP_EN, 0xff)

def disableDIETEMPRDY():
    setBit(INT_ENABLE2_ADDR, INT_ENABLE2_TEMP_EN, 0)

def setMode(mode):
    val=readRegister(MODE_CONFIG_ADDR)
    val=(val&(~MODE_CONFIG_MODE))|(mode)
    writeRegister(MODE_CONFIG_ADDR,val)

def setADCRange(ADCRange):
    val=readRegister(SPO2_CONFIG_ADDR)
    val=(val&(~SPO2_CONFIG_ADC_RGE))|(ADCRange)
    writeRegister(SPO2_CONFIG_ADDR,val)

def setSampleRate(sR):
    val=readRegister(SPO2_CONFIG_ADDR)
    val=(val&(~SPO2_CONFIG_SR))|(sR)
    writeRegister(SPO2_CONFIG_ADDR,val)

def setPulseWidth(pW):
    val=readRegister(SPO2_CONFIG_ADDR)
    val=(val&(~SPO2_CONFIG_LED_PW))|(pW)
    writeRegister(SPO2_CONFIG_ADDR,val)

def setAmplitude(led,amp):
    writeRegister(led,amp)

def enableRoll():
    setBit(FIFO_CONFIG_ADDR, FIFO_CONFIG_ROLL_OVER_EN, 0xff)

def disableRoll():
    setBit(FIFO_CONFIG_ADDR, FIFO_CONFIG_ROLL_OVER_EN, 0)

def setSampleAverage(avg):
    val=readRegister(FIFO_CONFIG_ADDR)
    val=(val&(~FIFO_CONFIG_SMP_AVE))|(avg)
    writeRegister(FIFO_CONFIG_ADDR,val)

def setAFULL(afull):
    val=readRegister(FIFO_CONFIG_ADDR)
    val=(val&(~FIFO_CONFIG_A_FULL))|(afull)
    writeRegister(FIFO_CONFIG_ADDR,val)

def resetFIFO():
    writeRegister(FIFO_WRITE_PTR_ADDR,0x00)
    writeRegister(OVF_CNTR_ADDR,0x00)
    writeRegister(FIFO_READ_PTR_ADDR,0x00)

def getWritePointer():
    return readRegister(FIFO_WRITE_PTR_ADDR)&FIFO_WRITE_PTR

def getReadPointer():
    return readRegister(FIFO_READ_PTR_ADDR)&FIFO_READ_PTR

def setReadPointer(addr):
    writeRegister(FIFO_READ_PTR_ADDR,addr)

def checkAvailable():
    write=getWritePointer()
    read=getReadPointer()
    return (write-read+32)%32

def readRegister(reg):
    rwLock.acquire()
    val=bus.read_i2c_block_data(deviceAddr,reg,1)
    rwLock.release()
    return val[0]
def writeRegister(reg,data):
    rwLock.acquire()
    bus.write_i2c_block_data(deviceAddr, reg, [data])
    rwLock.release()
def setBit(reg,mask,data):
    val=readRegister(reg)
    val=(val&(~mask))|(mask&data)
    writeRegister(reg,val)

def readPPGData(num):
    rwLock.acquire()
    data=[]
    for _ in range(num):
        data+=bus.read_i2c_block_data(deviceAddr,FIFO_DATA_REG_ADDR,3)
    rwLock.release()
    return data
