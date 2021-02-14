import clr
import os, sys
import time
from System import Decimal

clr.AddReference(r"C:\\Program Files\\Thorlabs\\Kinesis\\Thorlabs.MotionControl.DeviceManagerCLI.dll")
clr.AddReference(r"C:\\Program Files\\Thorlabs\\Kinesis\\Thorlabs.MotionControl.GenericMotorCLI.dll")
clr.AddReference(r"C:\\Program Files\\Thorlabs\\Kinesis\\Thorlabs.MotionControl.TCube.DCServoCLI.dll")

from Thorlabs.MotionControl.DeviceManagerCLI import *
from Thorlabs.MotionControl.GenericMotorCLI import *
from Thorlabs.MotionControl.TCube.DCServoCLI import *


class KinesisMotor():
    
    def __init__(self,SerialNum=None,polling=100,verbose = False):
        self.verbose = verbose
        self.Connected = False
        self.device = None
        self.polling = polling
        self.lims = None
        if SerialNum is not None:
            if type(SerialNum)==int:
                SerialNum = str(SerialNum)
            if self.verbose: print ("Serial is", SerialNum)
            self.SerialNum = str(SerialNum)
            self.initialize()
        else:
            raise Exception('Please provide a serial number')

    def initialize(self):
        try:
            device_list_result = DeviceManagerCLI.BuildDeviceList()
            
            
            #device being used is th TCubeDCServo, can easily be replaced with a KCube since most commands are the same, just need to change the dll being used above
            self.device = TCubeDCServo.CreateTCubeDCServo(self.SerialNum)
            
            
            self.device.Connect(self.SerialNum)
            self.device.WaitForSettingsInitialized(5000)
            #use this line to set motor to real world units
            self.motorSettings = self.device.LoadMotorConfiguration(self.SerialNum,DeviceConfiguration.DeviceSettingsUseOptionType.UseFileSettings)

            self.device.StartPolling(self.polling)
            self.device.EnableDevice()
            
            #wait for the device to get enabled
            time.sleep(0.5)
            
            self.getVelParams()
            self.getLimits()

            if self.verbose:
                deviceInfo = self.device.GetDeviceInfo()
                print('Motor Connected: ',deviceInfo.Name, '  ', 'SN:',deviceInfo.SerialNumber)

        except DeviceNotReadyException:
            raise

    def disconnect(self):
        
        self.device.StopPolling()
        
        # This shuts down the controller. This will use the .NET Disconnect() function to close communications & will then close the used library.
        self.device.ShutDown()
        time.sleep(0.5)

    def resetConnection(self):
        try:
            self.device.ResetConnection(self.SerialNum)
            self.device.StartPolling(self.polling)
            self.device.EnableDevice()
        except (NameError,AttributeError):
            raise Exception('No device connected')

    def getHardwareInfo(self):
        try:
            self.deviceInfo = self.device.GetDeviceInfo()
            self.deviceName = self.deviceInfo.Name
            self.SerialNum_fromDev = self.deviceInfo.SerialNumber
        except (NameError,AttributeError):
            raise Exception('Please initialize the device first')

    def getLimits(self):
        try:
            self.lims = self.device.AdvancedMotorLimits
            self.max_accel_lim = Decimal.ToDouble(self.lims.get_AccelerationMaximum())
            self.max_vel_lim = Decimal.ToDouble(self.lims.get_VelocityMaximum())
            self.max_travel_lim = Decimal.ToDouble(self.lims.get_LengthMaximum())
            self.min_travel_lim = Decimal.ToDouble(self.lims.get_LengthMinimum())
            
            if self.verbose:
                print('Acceleration limit',self.max_accel_lim,'mm/s^2')
                print('Max velocity limit',self.max_vel_lim,'mm/s')
                print('Max travel limit',self.max_travel_lim,'mm')
                print('Min travel limit',self.min_travel_lim,'mm')
            
            
        except (NameError,AttributeError):
            raise Exception('Please initialize the device first')


    def getVelParams(self):
        try:
            self.velPars = self.device.GetVelocityParams()
            self.accel = Decimal.ToDouble(self.velPars.get_Acceleration())
            self.min_vel = Decimal.ToDouble(self.velPars.get_MinVelocity())
            self.max_vel = Decimal.ToDouble(self.velPars.get_MaxVelocity())

            if self.verbose:
                print('Acceleration:',self.accel,'mm/s^2')
                print('Min Velocity:',self.min_vel,'mm/s')
                print('Max Velocity:',self.max_vel,'mm/s')

        except (NameError,AttributeError):
            raise Exception('Please initialize the device first')

    def set_acceleration(self, accel):

        try:
            if accel>self.max_accel_lim:
                # raise Exception('Maximum acceleration of %.2f mm/s^2 exceeded'%self.max_accel_lim)
                print('Maximum acceleration of %.2f mm/s exceeded, setting to max acceleration' % self.max_accel_lim)
                accel = self.max_accel_lim
            
            self.velPars.set_Acceleration(Decimal(accel))
            self.device.SetVelocityParams(self.velPars)
            
            #wait for parameters to get set. the above function returns immediately, so messing with other motor commands before everything is set can lock up the motor connection
            if self.verbose:
                print('Setting acceleration....')
            time.sleep(0.5)
        except (NameError,AttributeError):
            raise Exception('Please initialize the device first')

    def set_velocity(self, max_vel=2.1,min_vel=0):
        

        try:
            if max_vel>self.max_vel_lim:
                # raise Exception('Maximum velocity of %.2f mm/s exceeded' % self.max_vel_lim)
                print('Maximum velocity of %.2f mm/s exceeded, setting to max velocity' % self.max_vel_lim)
                max_vel = self.max_vel_lim
                
            self.velPars.set_MinVelocity(Decimal(min_vel))
            self.velPars.set_MaxVelocity(Decimal(max_vel))
            self.device.SetVelocityParams(self.velPars)
            
            #wait for parameters to get set. the above function returns immediately, so messing with other motor commands before everything is set can lock up the motor connection
            if self.verbose:
                print('Setting velocity....')
            time.sleep(0.5)
        except( NameError,AttributeError):
            raise Exception('Please initialize the device first')
            
    def getPos(self):
        try:
            pos = Decimal.ToDouble(self.device.Position)

        except (NameError, AttributeError):
            raise Exception('Please initialize the device first')

        return pos

    def moveTo(self,position,time_out = 60000):
        if position>=self.min_travel_lim and position <= self.max_travel_lim:
            self.device.MoveTo(Decimal(position),time_out)

        else:
            raise Exception('Requested position outside of device travel range')

    def moveRel(self, relDist, time_out = 60000):
        
        if relDist>=0:
            self.device.MoveRelative(MotorDirection.Forward,Decimal(abs(relDist)),time_out)
        else:
            self.device.MoveRelative(MotorDirection.Backward,Decimal(abs(relDist)),time_out)

    def setHomingVel(self,vel):
        self.device.SetHomingVelocity(Decimal(vel))

    def go_home(self,time_out=60000):
        
        if self.device.CanHome:
            self.device.Home(time_out)
        else:
            raise Exception('Device unable to home')

    def identify(self):
        self.device.IdentifyDevice()

    def stop(self,time_out=60000):
        self.device.Stop(time_out)

    def stopImmediate(self):
        self.device.StopImmediate()
    
#returns True if motor hasn't been homed before and doesn't know it's absolute location. Otherwise returns false   
    def needsHoming(self):
        return self.device.get_NeedsHoming()



if __name__ == "__main__":

    serial = '83860125'
    mot = KinesisMotor(serial,polling=250,verbose=True)
    mot.set_velocity(2.6)
    mot.set_acceleration(4)
    
    init_pos = mot.getPos()
    t0 = time.time()
    # mot.moveTo(init_pos+0.05)
    # mot.moveTo(init_pos+0.05*2)
    mot.moveRel(-0.1)

    print('Initial pos: ' + str(init_pos))
    print('New pos: '+str(mot.getPos()))

    t1 = time.time()-t0
    
    print('Motor movement took %.4f seconds'%t1)
    
    mot.moveTo(init_pos)
    
    
    mot.disconnect()



