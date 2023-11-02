import sensor
import time
import RPi.GPIO as GPIO
import csv
from struct import pack
import numpy as np
from scipy.signal import resample
def toByteArray(Arr):
    vals=[]
    for i in range(0,len(Arr),3):
        val=sensor.ADC_LSB_4096*float((Arr[i] << 16 | Arr[i+1] << 8 | Arr[i+2]) & 0x03FFFF)*0.001
        vals.append(val)
    return vals
sensor.ppgInit(sensor.PPGSENSOR_SMPAVG_NONE,
                            sensor.PPGSENSOR_HR,
                            sensor.PPGSENSOR_ADC_4096,
                            sensor.PPGSENSOR_SR_200,
                            sensor.PPGSENSOR_PW_15,
                            0x24)

sensor.startMonitor()
lst=[]
while True:
    GPIO.wait_for_edge(sensor.intChannel, GPIO.FALLING)
    sensor.readRegister(sensor.INT_STATUS1_ADDR)
    sensor.readRegister(sensor.INT_STATUS2_ADDR)
    lst+=toByteArray(sensor.readPPGData(sensor.checkAvailable()))
    if len(lst)>=1000:
        sensor.stopMonitor()
        break
arr=np.asarray(lst[:1000])
re=resample(arr,875)
np.savetxt(f'{time.asctime( time.localtime(time.time()) )}.csv',re,delimiter=',')

#print(sensor.readRegister(sensor.INT_STATUS1_ADDR))