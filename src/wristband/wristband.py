import sys
sys.path.append(".")
import ble
import dbus
from gi.repository import GLib
import RPi.GPIO as GPIO
import sensor
from threading import Thread,Lock
from struct import pack
import numpy as np
#define
SERVICE_UUID='9838e941-a41d-44a7-9a7b-2ef6557a4b1a'
MONITORCHAR1_UUID='734450ca-e3c7-4e01-a81e-c8b7d971a8c9'
MONITORCHARDES1_UUID='f6081bca-3a7d-43b4-a5e8-d46320ee5a1b'
#public varible
mainloop=None




class WristbandMonitorService(ble.Service):
    def __init__(self, bus, index, uuid, primary,smpAvg,mode,adcRange,sr,pw,amp):
        super().__init__(bus, index, uuid, primary)
        self.notifying=False
        self.ampAvg=smpAvg
        self.mode=mode
        self.adcRange=adcRange
        self.sr=sr
        self.pw=pw
        self.amp=amp
        if adcRange==sensor.PPGSENSOR_ADC_2048:
            self.lsb=sensor.ADC_LSB_2048
        elif adcRange==sensor.PPGSENSOR_ADC_4096:
            self.lsb=sensor.ADC_LSB_4096
        elif adcRange==sensor.PPGSENSOR_ADC_8192:
            self.lsb=sensor.ADC_LSB_8192
        elif adcRange==sensor.PPGSENSOR_ADC_16384:
            self.lsb==sensor.ADC_LSB_16384
        sensor.ppgInit(smpAvg,
                            mode,
                            adcRange,
                            sr,
                            pw,
                            amp)
        self.notifyLock=Lock()
        Thread(target=self.dataListener).start()
    def dataListener(self):
        #print('thread started')
        seqNum=0x00
        #PACKETSIZE=200
        MAXDATAUNIT=1400
        left=MAXDATAUNIT
        data=[]
        while True:
            GPIO.wait_for_edge(sensor.intChannel, GPIO.FALLING)
            #print('getting data')
            sensor.readRegister(sensor.INT_STATUS1_ADDR)
            numRead=min(sensor.checkAvailable(),left)
            data+=self.toByteArray(sensor.readPPGData(numRead))
            left=left-numRead
            self.notifyLock.acquire()
            if self.notifying==False:
                sensor.stopMonitor()
                left=MAXDATAUNIT
                self.notifyLock.release()
                continue
            self.notifyLock.release()
            if left==0:
                sensor.stopMonitor()
                #print('data full, sending')
                left=MAXDATAUNIT
                arr=np.asarray(data)
                if 2061-np.mean(arr)<500:
                    bytedata=b''
                    for d in data:
                        bytedata+=pack('f',d)
                    chars=self.get_characteristics()
                    for i in range(28):
                        buffer=seqNum.to_bytes(1,'big',signed=False)+i.to_bytes(1,'big',signed=False)+bytedata[i*200:i*200+200]
                        #print(buffer)
                        chars[0].PropertiesChanged(ble.GATT_CHRC_IFACE, { 'Value': dbus.ByteArray(buffer) }, [])
                seqNum=(seqNum+1)%256
                data=[]
                sensor.startMonitor()
                #sensor.readRegister(sensor.INT_STATUS1_ADDR)
            #print(byteArr)
                
    def toByteArray(self,Arr):
        vals=[]
        for i in range(0,len(Arr),3):
            val=self.lsb*float((Arr[i] << 16 | Arr[i+1] << 8 | Arr[i+2]) & 0x03FFFF)*0.001
            vals.append(val)
            #vals+=pack('f',val)
        return vals



class WristbandMonitorChar(ble.Characteristic):
    def __init__(self, bus, uuid,index,service,last=False):
        super().__init__(bus, index, uuid, ['notify'], service)
        self.last=last
        #GPIO.setmode(GPIO.BCM)
    

    

    def StartNotify(self):
        #print('start notifying')
        print('start notifying,last={},UUID:{}'.format(self.last,self.uuid))
        if self.last==True:
            self.service.notifyLock.acquire()
            self.service.notifying=True
            sensor.startMonitor()
            self.service.notifyLock.release()
        

    def StopNotify(self):
        print('stop notifying,last={},UUID:{}'.format(self.last,self.uuid))
        if self.last==True:
            self.service.notifyLock.acquire()
            self.service.notifying=False
            sensor.stopMonitor()
            self.service.notifyLock.release()

class WristbandMonitorCharDes(ble.Descriptor):
    def __init__(self, bus, uuid,index, characteristic):
        super().__init__(bus, index, uuid, ['read','write'], characteristic)
    def ReadValue(self, options):
        return dbus.ByteArray('WRISTBAND MONITOR')






def register_ad_cb():
    print('Advertisement registered')


def register_ad_error_cb(error):
    print('Failed to register advertisement: ' + str(error))
    mainloop.quit()
def register_app_cb():
    print('GATT application registered')


def register_app_error_cb(error):
    print('Failed to register application: ' + str(error))
    mainloop.quit()


def main():
    global mainloop
    dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
    bus=dbus.SystemBus()
    adapter = ble.find_adapter(bus)
    if not adapter:
        print('LEAdvertisingManager1 interface not found')
        return
    
    adapterProps = dbus.Interface(bus.get_object(ble.BLUEZ_SERVICE_NAME, adapter),
                                   "org.freedesktop.DBus.Properties")
    adapterProps.Set("org.bluez.Adapter1", "Powered", dbus.Boolean(1))

    mainloop = GLib.MainLoop()
    ## Set Advertisement
    adManager = dbus.Interface(bus.get_object(ble.BLUEZ_SERVICE_NAME, adapter),
                                ble.LE_ADVERTISING_MANAGER_IFACE)
    wristbandAdv=ble.Advertisement(bus,0,'peripheral')
    wristbandAdv.add_manufacturer_data(0xFF,[0x12, 0x17],)
    wristbandAdv.add_local_name("iWrist")
    wristbandAdv.add_data(0x26, [0x01, 0x01, 0x00])
    wristbandAdv.include_tx_power=True
    
    
    ## Set Service & Char & App
    serviceManager = dbus.Interface(
            bus.get_object(ble.BLUEZ_SERVICE_NAME, adapter),
            ble.GATT_MANAGER_IFACE)
    wristbandService=WristbandMonitorService(bus,1,SERVICE_UUID,True,
                                sensor.PPGSENSOR_SMPAVG_NONE,
                                sensor.PPGSENSOR_HR,
                                sensor.PPGSENSOR_ADC_4096,
                                sensor.PPGSENSOR_SR_200,
                                sensor.PPGSENSOR_PW_15,
                                0x24)
    wristbandMonitorChar1=WristbandMonitorChar(bus,MONITORCHAR1_UUID,2,wristbandService,True)
    wristbandMonitorChar1.add_descriptor(WristbandMonitorCharDes(bus,MONITORCHARDES1_UUID,3,wristbandMonitorChar1))
    wristbandService.add_characteristic(wristbandMonitorChar1)
    wristbandApp=ble.Application(bus)
    wristbandApp.add_service(wristbandService)
    
    
    adManager.RegisterAdvertisement(wristbandAdv.get_path(),
                                    {},
                                    reply_handler=register_ad_cb,
                                    error_handler=register_ad_error_cb)
    serviceManager.RegisterApplication(wristbandApp.get_path(), {},
                                    reply_handler=register_app_cb,
                                    error_handler=register_app_error_cb)
    mainloop.run()
    adManager.UnregisterAdvertisement(wristbandAdv)
    dbus.service.Object.remove_from_connection(wristbandAdv)

if __name__ == '__main__':
    main()



    