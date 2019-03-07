#OpenGS : The Open Srouce Ground Station Project
#MSP v2 Protocol Handler
#Wiki: https://github.com/iNavFlight/inav/wiki/MSP-V2

#
from serial import Serial
import time
from threading import Event, Thread
#

# Standard commands
MSP_IDENT = 100
MSP_STATUS = 101
MSP_RAW_IMU = 102
MSP_SERVO = 103
MSP_MOTOR = 104
MSP_RC = 105
MSP_RAW_GPS = 106
MSP_COMP_GPS = 107
MSP_ATTITUDE = 108
MSP_ALTITUDE = 109
MSP_ANALOG = 110
MSP_GET_WP = 118
MSP_SERVO_CONF = 120
MSP_NAV_STATUS = 121
MSP_CPU = 150           # NOT SURE OF CORRECT NAME FOR THIS COMMAND
MSP_SET_RAW_RC = 200
MSP_RESET_CONF = 208
MSP_SET_WP = 209

mappable_channels = { b'\x00' : "throttle",
        b'\x01' : "pitch",
        b'\x02' : "roll",
        b'\x03' : "yaw",
}

class RepeatedTimer:
    #Repeat `function` every `interval` seconds made by shlapion (https://github.com/shlapion)
    def __init__(self, interval, function, *args, **kwargs):
        self.interval = interval
        self.function = function
        self.args = args
        self.kwargs = kwargs
        self.start = time.time()
        self.event = Event()
        self.thread = Thread(target=self._target)

    def startTimer(self):
        self.thread.start()

    def _target(self):
        while not self.event.wait(self._time):
            self.function(*self.args, **self.kwargs)

    @property
    def _time(self):
        return self.interval - ((time.time() - self.start) % self.interval)

    def stopTimer(self):
        self.event.set()
        self.thread.join()

class MSP(object):
    def __init__(self, controller: Serial):
        self._controller = controller
        
    def crc8_dvb_s2(self,crc, a):
        crc ^= a 
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0xD5) % 256
            else:
                crc = (crc << 1) % 256
        return crc
    
    def checksum(self,message): 
        crc = 0  #initialize crc
        #calculate crc8_dvb_s2
        for i in range(len(message)):
            crc  = self.crc8_dvb_s2(crc, message[i])
        #-------------
        return crc
    
    def construct_payload(self,code: int,payload = bytes()):
        preamble = b'$X<' #MSP v2 Preamble
        flag = b'\x00' #uint8, flag, usage to be defined (set to zero)
        function = code.to_bytes(2, byteorder='little') #uint16 (little endian). 0 - 255 
        payload_size = len(payload).to_bytes(2, byteorder='little') #unit16
        ck = self.checksum(flag + function + payload_size + payload).to_bytes(1, byteorder='little') #unit8 checksum
        return preamble + flag + function + payload_size + payload + ck
    
    def read_payload(self, func):
        if (self._controller.read(3) == b'$X>'): #find the preamble
            flag = self._controller.read(1) #read the unit8 flag
            function = self._controller.read(2) # read the unit16 function 
            if (int.from_bytes(function, byteorder='little') == func):
                payload_size = self._controller.read(2) #read the unit16 payload size
                payload = self._controller.read(int.from_bytes(payload_size, byteorder='little'))#read the payload using the payload_size to know how many bytes to read
                ck = self._controller.read(1)# read the checksum
                if (ck == self.checksum(flag + function + payload_size + payload).to_bytes(1, byteorder='little')): #compare the checksum
                    return payload
        self._controller.flushInput()
        raise RuntimeError("Controller has not responded.")

    # SEnd an MSP Request and return payload
    def Send_MSP_Request(self, Command):
        self._controller.flushInput()
        self._controller.write(self.construct_payload(Command)) #write the command
        payload = self.read_payload(Command) # get the payload
        return payload

    # MSP_IDENT   100 FC →    
    # VERSION UINT 8  version of MultiWii

    # MULTITYPE   UINT 8  type of multi:
    #   TRI/QUADP,QUADX,BI,GIMBAL,Y6,HEX6,FLYING_WING,Y4,HEX6X,OCTOX8, OCTOFLATP,OCTOFLATX,AIRPLANE,HELI_120,HELI_90,VTAIL4,HEX6H,SINGLECOPTER,DUALCOPTER
    # NOTE: INAV USES QUADX FOR EVERYTHING
    # MSP_VERSION UINT 8  not used currently
    # capability  UINT 32 A 32 bit variable to indicate capability of FC board.
    # Currently, BIND button is used on first bit, DYNBAL on second, FLAP on third
    def get_ident(self):
        payload = self.Send_MSP_Request(MSP_IDENT) # get the payload
        values = dict()
        values['version'] = int.from_bytes(payload[0:1], byteorder='little', signed=False) 
        values['multi_type'] = int.from_bytes(payload[1:2], byteorder='little', signed=False) 
        values['msp_version'] = int.from_bytes(payload[2:3], byteorder='little', signed=False) 
        values['capability'] = int.from_bytes(payload[3:7], byteorder='little', signed=False) 
        return values

#MSP_STATUS  101 FC →    
#cycleTime   UINT 16 unit: microseconds
#i2c_errors_count    UINT 16 
#sensor  UINT 16 BARO<<1|MAG<<2|GPS<<3|SONAR<<4
#flag    UINT 32 a bit variable to indicate which BOX are active, the bit position depends on the BOX which are configured
#global_conf.currentSet  UINT 8  to indicate the current configuration setting
    def get_status(self):
        payload = self.Send_MSP_Request(MSP_STATUS) # get the payload
        values = dict()
        values['cycletime'] = int.from_bytes(payload[0:2], byteorder='little', signed=False) 
        values['i2c_err_count'] = int.from_bytes(payload[2:4], byteorder='little', signed=False) 
        values['sensor'] = int.from_bytes(payload[4:6], byteorder='little', signed=False) 
        values['flags'] = int.from_bytes(payload[6:10], byteorder='little', signed=False) 
        values['global_conf_current_set'] = int.from_bytes(payload[10:11], byteorder='little', signed=False) 
        return values

    def get_cpu_status(self):
        payload = self.Send_MSP_Request(MSP_CPU) # get the payload
        values = dict()
        values['cpu_load'] = int.from_bytes(payload[0:2], byteorder='little', signed=False) 
        values['arming_flags'] = int.from_bytes(payload[2:4], byteorder='little', signed=False) 
        values['calibration_axis_flags'] = int.from_bytes(payload[4:5], byteorder='little', signed=False) 
        return values
        
    # Get RAW IMU Data
    def get_raw_imu(self):
        payload = self.Send_MSP_Request(MSP_RAW_IMU) # get the payload
        values = dict()
        values['accx'] = int.from_bytes(payload[0:2], byteorder='little', signed=True) 
        values['accy'] = int.from_bytes(payload[2:4], byteorder='little', signed=True) 
        values['accz'] = int.from_bytes(payload[4:6], byteorder='little', signed=True) 
        values['gyrx'] = int.from_bytes(payload[6:8], byteorder='little', signed=True) 
        values['gyry'] = int.from_bytes(payload[8:10], byteorder='little', signed=True) 
        values['gyrz'] = int.from_bytes(payload[10:12], byteorder='little', signed=True) 
        values['magx'] = int.from_bytes(payload[12:14], byteorder='little', signed=True) 
        values['magy'] = int.from_bytes(payload[14:16], byteorder='little', signed=True) 
        values['magz'] = int.from_bytes(payload[16:18], byteorder='little', signed=True) 
        return values
    
    # Get Current Board Attitude
    def get_attitude(self):
        payload = self.Send_MSP_Request(MSP_ATTITUDE) # get the payload
        values = dict()
        #deconstrunct the payload
        values['roll'] = int.from_bytes(payload[0:2], byteorder='little', signed=True) / 10 #make int from the first 2 bytes 
        values['pitch'] = int.from_bytes(payload[2:4], byteorder='little', signed=True) / 10
        values['yaw'] = int.from_bytes(payload[4:6], byteorder='little', signed=False)
        return values
            
    # Get RAW GPS Data
    def get_raw_gps(self):
        payload = self.Send_MSP_Request(MSP_RAW_GPS) # get the payload
        values = dict()
        values['fix_type'] = int.from_bytes(payload[0:1], byteorder='little', signed=False)
        values['sats'] = int.from_bytes(payload[1:2], byteorder='little', signed=False)
        values['lat'] = int.from_bytes(payload[2:6], byteorder='little', signed=True) / 10000000
        values['lon'] = int.from_bytes(payload[6:10], byteorder='little', signed=True) / 10000000
        values['alt'] = int.from_bytes(payload[10:12], byteorder='little', signed=True)
        values['groundSpeed'] = int.from_bytes(payload[12:14], byteorder='little', signed=False)
        values['groundCourse'] = int.from_bytes(payload[14:16], byteorder='little', signed=False)
        values['hdop'] = int.from_bytes(payload[16:18], byteorder='little', signed=False) / 100
        return values

    def get_channel_map(self):
        payload = self.Send_MSP_Request(64)
        values = []
        for i in range(len(payload)):
            values.append(mappable_channels[payload[i:i+1]])
        return  values

    def get_rc(self, channel_map = ["throttle","yaw","pitch","roll"]):
        payload = self.Send_MSP_Request(MSP_RC)
        values = dict()
        types = ( #channel names
        channel_map[0],  channel_map[1],  channel_map[2],  channel_map[3],
        "aux1", "aux2", "aux3", "aux4",
        "aux5", "aux6", "aux7", "aux8",
        "aux9", "aux10", "aux11", "aux12",
        "aux12", "aux13","aux14", "aux15"
        )
        for i in range(len(payload)//2): #each channel is 2 bytes, so we need half of the payload lenght
            x = i * 2
            values[types[i]] = int.from_bytes(payload[x:x+2], byteorder='little', signed=False)#separate all channels
        return values
    
    def get_wp(self, wp_number : int):
        self._controller.flushInput()
        self._controller.write(self.construct_payload(MSP_GET_WP,wp_number.to_bytes(1, byteorder='little')))
        payload = self.read_payload(MSP_GET_WP)
        values = dict()
        values['number'] = int.from_bytes(payload[0:1], byteorder='little', signed=False)
        values['action'] = int.from_bytes(payload[1:2], byteorder='little', signed=False)
        values['lat'] = int.from_bytes(payload[2:6], byteorder='little', signed=True) / 10000000
        values['lon'] = int.from_bytes(payload[6:10], byteorder='little', signed=True) / 10000000
        values['alt'] = int.from_bytes(payload[10:14], byteorder='little', signed=True)
        values['p1'] = int.from_bytes(payload[14:16], byteorder='little', signed=False)
        values['p2'] = int.from_bytes(payload[16:18], byteorder='little', signed=False)
        values['p3'] = int.from_bytes(payload[18:20], byteorder='little', signed=False)
        values['flags'] = int.from_bytes(payload[20:21], byteorder='little', signed=False)
        return values
    
    def get_analog(self):
        payload = self.Send_MSP_Request(MSP_ANALOG)
        values = dict()
        values['battery_voltage'] = int.from_bytes(payload[0:1], byteorder='little', signed=False) #vbat
        values['mah_drawn'] = int.from_bytes(payload[1:3], byteorder='little', signed=False) #mah drawn
        values['rssi'] = int.from_bytes(payload[3:5], byteorder='little', signed=False) #rssi
        values['amp'] = int.from_bytes(payload[5:7], byteorder='little', signed=True) #current Amp draw
        return values

    def set_wp(self, wp_number,action,lat,lon,alt,p1,p2,p3,flag : int):
        payload = bytes()
        payload += wp_number.to_bytes(1, byteorder='little')
        payload += action.to_bytes(1, byteorder='little')#action
        payload += int(lat * 10000000).to_bytes(4, byteorder='little')#lat 
        payload += int(lon * 10000000).to_bytes(4, byteorder='little')#lon
        payload += alt.to_bytes(4, byteorder='little')#to set altitude (cm)
        payload += p1.to_bytes(2, byteorder='little')#P1
        payload += p1.to_bytes(2, byteorder='little')#P2
        payload += p1.to_bytes(2, byteorder='little')#P3
        payload += flag.to_bytes(1, byteorder='little')#P3
        self._controller.flushInput()
        self._controller.write(self.construct_payload(MSP_SET_WP,payload)) # pass payload
        self.read_payload(MSP_SET_WP)
        
    def set_raw_rc(self, channels: list):
        payload = bytes()
        for channel in channels:
            payload += channel.to_bytes(2, byteorder='little')#add all channels together in one signle 'bytes' variable
        self._controller.flushInput()
        self._controller.write(self.construct_payload(MSPSET_RAW_RC,payload)) # pass payload
        self.read_payload(MSP_SET_RAW_RC)

    def start_threaded_rc(self,refreshRate): #refreshrate in Hertz. Start sending data to MSP_RX every x seconds.
      global timer, secondsPeriod
      secondsPeriod = 1 / refreshRate #calculate Hz to seconds(period)
      timer = RepeatedTimer(secondsPeriod,self.set_raw_rc,[1500,1500,1000,1500,1000,1000,1000,1000]) #set the timer
      timer.startTimer() #stop the timer
        
    def set_threaded_rc(self,channels: list): #change the data beeing sent
      global timer, secondsPeriod
      timer.stopTimer() #stop the timer
      timer = RepeatedTimer(secondsPeriod,self.set_raw_rc,channels)#set the timer with the new channel values
      timer.startTimer()#restat the timer
    
    def stop_threaded_rc(self): #stops the timer
      global timer
      timer.stopTimer()
