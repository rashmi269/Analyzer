def push_data(value,flag,testname):

    """
    Library imported: SMBus,time,os,RPI.GPIO
    we are programming the GPIO by BCM pin numbers. (PIN35 as ‘GPIO19’)

    method to set the peltier temperature  based on PID algorithm
     :param self: self
     :param set_temperature: temperature to be set
     :return: None
     :rtype: None
     :raises: None
     
     temperature_read = 0.0
     PID_error = 0.0
     previous_error = 0.0
     elapsedTime=0.0
     Time=0.0
     timePrev=0.0
     PID_value=0.0


    contains the remote database connections variables and authentication parameters

    this module also contains specific instructions on how to install and configure\n 
    myslpython on raspberripy and how to connect to remote database.\n
    please refer to the commnets at the end of the source file of this module. (module name = push_data.py)
    
    #First we read the real value of temperature
         temperature_read = sensor.get_obj_temp()\n
         #Next we calculate the error between the setpoint and the real value\n
         PID_error = set_temperature - temperature_read


    #For derivative we need real time to calculate speed change rate
         timePrev = Time\n                         # the previous time is stored before the actual time read\n
         Time = millis()  

    Remember to store the previous error for next loop
    """

    # from smbus2 import SMBus
# from mlx90614 import MLX90614
# import time
# import os
# import RPi.GPIO as IO
# IO.setwarnings(False)           #do not show any warnings
# IO.setmode (IO.BCM)         #we are programming the GPIO by BCM pin numbers. (PIN35 as ‘GPIO19’)
# IO.setup(19,IO.OUT)           # initialize GPIO19 as an output.

# p = IO.PWM(19,490.27)
# p.start(100)

# bus = SMBus(1)
# time.sleep(2)
# os.system('i2cdetect -y 1')

# #wait here to avoid 121 IO Error
# sensor = MLX90614(bus, address=0x5A)



# millis = lambda: int(round(time.time() * 1000))

# def set_peltier_temperature(set_temperature = 37.0):

    
#     """
#     method to set the peltier temperature  based on PID algorithm
#     :param self: self
#     :param set_temperature: temperature to be set
#     :return: None
#     :rtype: None
#     :raises: None
#     """
#     temperature_read = 0.0
#     PID_error = 0.0
#     previous_error = 0.0
#     elapsedTime=0.0
#     Time=0.0
#     timePrev=0.0
#     PID_value=0.0

#     #PID constants:
#     kp=100.0
#     ki=0.3
#     kd=1.8

#     PID_p = 0.0
#     PID_i = 0.0
#     PID_d = 0.0


#     millisstaytime = time.time()
#     firstflag = False

#     while (temperature_read <= set_temperature+2 and time.time() - millisstaytime <= 15):

#         #print ("Ambient Temperature :", sensor.get_amb_temp())
#         print ("Object Temperature :", sensor.get_obj_temp())
#         time.sleep(1);

#         if(temperature_read == set_temperature and not firstflag):
#             millisstaytime = time.time()
#             firstflag = True
#         else:
#             millisstaytime = time.time()

#         #First we read the real value of temperature
#         temperature_read = sensor.get_obj_temp()
#         #Next we calculate the error between the setpoint and the real value
#         PID_error = set_temperature - temperature_read
#         #Serial.print("PID_error :");
#         #Serial.println(PID_error);
#         #Calculate the P value
#         PID_p = kp * PID_error
#         #Calculate the I value in a range on +-3
#         if(-3 < PID_error and PID_error<3):

#             PID_i = PID_i + (ki * PID_error)


#         #For derivative we need real time to calculate speed change rate
#         timePrev = Time                         # the previous time is stored before the actual time read
#         Time = millis()                         # actual time read
#         elapsedTime = (Time - timePrev) / 1000 
#         #Now we can calculate the D calue
#         PID_d = kd*((PID_error - previous_error)/elapsedTime)
#         #Final total PID value is the sum of P + I + D
#         PID_value = PID_p + PID_i + PID_d

#         #We define PWM range between 0 and 255
#         if(PID_value < 0):
        
#             PID_value = 0   

#         if(PID_value > 255)  :
        
#             PID_value = 255 

#         #Now we can write the PWM signal to the mosfet on digital pin D3
#         #analogWrite(PWM_pin,255-PID_value)  #255-PID_value
#         p.ChangeDutyCycle(((255-PID_value)/255)*100)
#         #print(((255-PID_value)/255)*100)
        

#         previous_error = PID_error     #Remember to store the previous error for next loop.
#         time.sleep(0.200)

#     p.ChangeDutyCycle(100)

