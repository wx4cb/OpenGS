from MSP import MSP
from serial import Serial

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

ser = Serial('/dev/ttyS21', baudrate=115200 , timeout=0.1) #initialize serial connection
uav = MSP(ser)
cpu = uav.get_cpu_status()
load = str(cpu['cpu_load'])

ident = uav.get_ident()
status= uav.get_status()

print("IDENT: ", ident)
print("STATUS: ", status)
