# -*- coding: utf-8 -*-
"""
Created on Sat Jul  4 13:40:45 2020
@author: MEMS
"""



# import cv2
# import numpy as np
# from picamera import PiCamera
# import time
# import board
# import busio
# import math 
# import RPi.GPIO as GPIO
# import mlx90614_rpi

# import adafruit_ads1x15.ads1115 as ADS
# from adafruit_ads1x15.analog_in import AnalogIn


# camera = PiCamera()

def visible_spectrum(h_weight,s_weight,l_weight,v_weight,intercept,flag_low,flag_high,temperature):
    """
    Visible spectrum analysis function
    must be called after the camera is initialized

    :param h_weight: weight of hue
    :param s_weight: weight of saturation
    :param l_weight: weight of lightness
    :param v_weight: weight of value
    :param intercept: intercept of the regression line
    :param flag_low: lower limit of the concentration
    :param flag_high: upper limit of the concentration
    :param temperature: temperature of the peltier
    :return: concentration and flag
    :rtype: tuple
    :raises ImportError: if the module is not found
    :raises ValueError: if the value is not valid
    :raises RuntimeError: if the function is called at the wrong time
    """
    mlx90614_rpi.set_peltier_temperature(temperature)

    camera.start_preview()

    time.sleep(0.0001)
    image= camera.capture("image12.jpg")
    image_read = cv2.imread('image12.jpg') 
    print(image_read)
    camera.stop_preview()

    #crop image
    # input coordinates generated from coordinates.py script.

    img = image_read[650:700,800:850]
    # cv2.imshow('img1', img)
    # cv
    # Average

   

    height, width, _ = np.shape(img)
    # calculate the average color of each row of our image
    avg_color_per_row = np.average(img, axis=0)
    # calculate the averages of our rows
    avg_colors = np.average(avg_color_per_row, axis=0)
    # print(avg_colors)
    int_averages = np.array(avg_colors, dtype=np.uint8)
    # print(f'BGR: {int_averages}')
    ## create a new image of the same height/width as the original
    average_image = np.zeros((height, width, 3), np.uint8)
    # # and fill its pixels with our average color
    average_image[:] = int_averages
    # #finally, show it side-by-side with the original
    #cv2.imshow("Avg Color", np.hstack([img, average_image]))

    # # Data extraction

    rgb =cv2.cvtColor(average_image, cv2.COLOR_BGR2RGB)
    hsv =cv2.cvtColor(average_image, cv2.COLOR_BGR2HSV)
    lab =cv2.cvtColor(average_image, cv2.COLOR_BGR2LAB)
    ycrcb = cv2.cvtColor(average_image, cv2.COLOR_BGR2YCrCb)
    px0=rgb[1,1]
    px1=hsv[1,1]
    px2=lab[1,1]
    px3=ycrcb[1,1]

    # #
    
    # print(str(px0), str(px1), str(px2), str(px3))
    R=px0[0]
    G=px0[1]
    B=px0[2]
    H=px1[0]
    S=px1[1]
    V=px1[2]
    L=px2[0]
    Y=px3[0]


    #Conc = (17.676294340765775 -(float(S)*0.02440896) + (float(R)*0.13483882)- (float(G)*0.17656559) + (float(B)*0.21852045)- (float(L)*0.23707172) + (float(Y)*0.03344113))

    #print(Conc) 

#     print(f"Iteration :{number}")
  
    # cv2.waitKey(1) 

    cv2.destroyAllWindows()
    time.sleep(0.0001)
        
    Conc = (intercept + (float(H)* h_weight) + (float(S)* s_weight) + (float(L)*l_weight) + (float(V)*v_weight))
    # Conc = h_weight*s_weight*l_weight*v_weight*intercept
    print("conc:", Conc)
    if Conc <flag_low :
        flag = 'LOW'
    elif Conc >flag_high:
        flag = 'HIGH'
    else:
        flag = 'NORMAL'

        
    return(Conc,flag)





class UV:
    """
    UV spectrum analysis class
    
    """
    def __init__(self):
        """
        :param self: self
        instialize the class with i2c handles and gpio pins
        """
        i2c = busio.I2C(board.SCL, board.SDA)

        # Create the ADC object using the I2C bus
        ads = ADS.ADS1115(i2c,1)

        # Create single-ended input on channel 0
        #chan = AnalogIn(ads, ADS.P0)

        # Create differential input between channel 0 and 1
        self.chan = AnalogIn(ads, ADS.P0, ADS.P1)



        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        GPIO.setup(26,GPIO.OUT,initial = GPIO.LOW)
        GPIO.setup(19,GPIO.OUT,initial = GPIO.LOW)
        GPIO.setup(6,GPIO.OUT,initial = GPIO.LOW)
        GPIO.setup(13,GPIO.OUT,initial = GPIO.LOW)


        self.factor1 = 296
        self.vzero=4.096
        self.vdark=0.0936






    def readAdcVal(self):
        """
        method to read adc values and return averaged value of 25 readings
        :param self: self
        """
        avg_v = 2.3567
#         print("{:>5}\t{:>5}".format('raw', 'v'))

#         for i in range(0,25):
                
#             print("{:>5}\t{:>5.2f}".format(self.chan.value, self.chan.voltage))
#             avg_v += self.chan.voltage
# #             time.sleep(0.2)

#         avg_v /=  25       
        print("avg_v:"+ str(avg_v))
        
    
        return avg_v





    def uv_spectrum(self,flag_low,flag_high,temperature):

        """
        method to calutaed the uv spectrum values

        first it sets the petier temperature and then reads the adc values

        :param self: self
        :param flag_low: lower limit of the flag
        :param flag_high: upper limit of the flag
        :param temperature: temperature of the peltier
        :return: value,absorbance,flag
        :rtype: float,float,str
        :raises: None

        """
        mlx90614_rpi.set_peltier_temperature(temperature)
        # self.calculate_std_values(temperature)

        value,absorbance = self.calculate_sample_values(temperature)

        if value < flag_low :
            flag = 'LOW'
        elif value > flag_high:
            flag = 'HIGH'
        else:
            flag = 'NORMAL'
        
        return(value,absorbance,flag) 




    def calculate_std_values(self,temperature):

        """
       
        method to calculate the standard values
        first it aspirates the sample from testube, sets the peltier temperature and reda the adc values
        :param self: self
        :param temperature: temperature of the peltier
        :return: None
        :rtype: None
        :raises: None
        """
        self.respiration()
        mlx90614_rpi.set_peltier_temperature(temperature)

        std_val  = int(input("input std concentration : "))
        
        

        avg_v1 = self.readAdcVal()

        print (avg_v1)
        x=(self.vzero-self.vdark)/(avg_v1-self.vdark)
        std_absp= math.log((self.vzero/avg_v1),10)
        absp1=math.log(((self.vzero-self.vdark)/(avg_v1-self.vdark)),10)
        od=2-math.log(((100*(avg_v1/self.vzero))),10)
        print(std_absp)
        print(absp1)
        print("factorod : " + str(od))
        
        factor = std_val/std_absp
        factor2=std_val/absp1
        factorod=std_val/od
        print("factor : " + str(factor) + "  std_absp: " + str(std_absp))
        
        # print("factor2 : " + str(factor) + "  std_absp: " + str(std_absp1))
        print("factorod : " + str(factor) + "  std_absp: " + str(od))

        self.clean()
            
    

    def calculate_sample_values(self,temperature):

        """
        method to calculate the sample values
        first it aspirates the sample from testube, sets the peltier temperature and reda the adc values

        :param self: self
        :param temperature: temperature of the peltier
        :return: value,absorbance
        :rtype: float,float
        :raises: None

        """

        mlx90614_rpi.set_peltier_temperature(temperature)

        self.respiration()
        avg_v1 = self.readAdcVal()


#                 absp= math.log((vzero/avg_v1),10)
#                 absp1=math.log((vzero-vdark)/(avg_v1-vdark),10)
        od=2-math.log(((100*(avg_v1/self.vzero))),10)
                
                    #             print("sample absp : " + str(absp))

        result = self.factor1 * od
                
        print("sample conc :" + str(result))
                
#       print("sample absp : " + str(absp1))
# 
#       result1 = factor2 * absp1
#                 
#       print("sample conc :" + str(result1))
#                 
#       print("sample absp : " + str(od))
# 
#       resultod = factorod * od
#                 
#       print("sample conc :" + str(resultod))
#                 
#                 
        self.clean()

        return (result,od)



    def respiration(self):

        """
        method to aspirate the sample from testube

        :param self: self
        :return: None
        :rtype: None
        :raises: None


        """

        GPIO.output(26,GPIO.HIGH)
        GPIO.output(6,GPIO.HIGH)
        print("motor on")
        time.sleep(0.3) #respiration delay
        
        GPIO.output(26,GPIO.LOW)
        GPIO.output(6,GPIO.LOW)
        
        print("motor off")

    def clean(self):

        """
        
        method to clean the testube
        :param self: self
        :return: None
        :rtype: None
        :raises: None
       
        """

        GPIO.output(26,GPIO.HIGH)
        GPIO.output(6,GPIO.HIGH)
        print("motor on")
        time.sleep(3)  #cleansing delay
        GPIO.output(26,GPIO.LOW)
        GPIO.output(6,GPIO.LOW)
        print("motor off") 




    
    
    

    



