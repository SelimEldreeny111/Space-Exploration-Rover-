from crc import Calculator, Crc16
import time
import serial
import cv2
import numpy as np
import tensorflow as tf

ser = serial.Serial(port="COM6", baudrate=9600, bytesize=8, timeout=2,
                    stopbits=serial.STOPBITS_ONE)

ACK=0x01


t=bytearray(7)
for i in range(7):
    t[i]=0x0a
    
    
def cap_Myimage():
    cv2.namedWindow("preview")
    vc = cv2.VideoCapture(0)
    
    if vc.isOpened(): # try to get the first frame
        rval, frame = vc.read()
        cv2.imshow("preview", frame)
        cv2.imwrite("1.png",frame)
        rval, frame = vc.read()
    
    vc.release()
    cv2.destroyWindow("preview")

def get_Myimage ():
    image = cv2.imread('1.png')
    resized = cv2.resize(image, (100,100), interpolation = cv2.INTER_AREA)
    resized =resized[:,:,1] # cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    resized=resized[:,:]
    print(resized)
    resized=np.array(resized)
    img_numpy_array =  tf.keras.preprocessing.image.img_to_array(resized)
    
    print(img_numpy_array)       
    bb=np.split(img_numpy_array,50)  
    return img_numpy_array
      
def packetconst(src,data,dest,Cmdtype,datasize):
    outputdata=[]
    outputdata1=[]
    outputdata.insert(0,dest)
    outputdata.insert(1,src)
    outputdata.insert(2,Cmdtype)
    outputdata.insert(3,datasize)
    i=0
    for x in (data):
        outputdata.insert(4+i,x)
        i=i+1
    
    calculator1 = Calculator(Crc16.CCITT)
    expected=calculator1.checksum(bytes(outputdata))
    print(expected)
    crc1=expected&0xff
    crc2=expected>>8
    
    outputdata1.insert(1,dest)
    outputdata1.insert(2,src)
    outputdata1.insert(3,Cmdtype)
    outputdata1.insert(4,datasize)
    for x in (data):
        outputdata1.insert(5+i,x)
        i=i+1
    outputdata1.insert(0,192)
    outputdata1.insert(datasize+5,crc1)
    outputdata1.insert(datasize+6,crc2)
    outputdata1.insert(datasize+7,192)
   

    # print(outputdata)
    return outputdata1




def packet_exetraction(res_data,datasize):
    
    outputdata=res_data[:]
     
    for i in range (datasize):
        if res_data[i]==0xc0:
            data_len=res_data[i+4]
            if datasize-data_len-i-7>=0:
                if res_data[i+data_len+7]==0xc0:
                    outputdata.pop(0)
                    del outputdata[-3:]
                    #print(outputdata)
                    calculator1 = Calculator(Crc16.CCITT)
                    expected=calculator1.checksum(bytes(outputdata))
                    print(expected)
                    crc2=int(expected&0xff)
                    crc1=int(expected>>8)
                    print(crc1)
                    print(res_data[-3])
                    print(crc2)
                    print(res_data[-2])
                    if (crc1 ==int(res_data[-2] ))& (crc2==int(res_data[-3])):
                        return outputdata
                    else:
                        return(-1)
                    
def ssp_CMD_Response(ssp_received_packet):
    ping            =0x00
    get_online_TLM  =0x0f
    cap_image       =0x0c
    get_image       =0x08
    start_scan      =0x16
    stop_scan       =0x1c
    respons=[]
    Cmdtype = ssp_received_packet[2]

    mysrc=ssp_received_packet[0]
    dest_to=ssp_received_packet[1]
    if Cmdtype == ping:
        respons.clear() 
        respons=packetconst(mysrc, [], dest_to, ACK, 0)
        
        
    elif Cmdtype ==  get_online_TLM:
        respons.clear() 
        respons=packetconst(mysrc , t, dest_to, get_online_TLM, 7)
        print(t)
        print(respons)
        
    elif Cmdtype ==  cap_image:
        respons.clear() 
        cap_Myimage()

        respons=packetconst(mysrc , [], dest_to, cap_image, 0)

        
    elif Cmdtype ==  get_image:
        myim=get_Myimage()
        respons.clear() 
        for i in range (50):
             ss=bytearray(myim[i])
         
             respons.insert(i, (packetconst(mysrc ,ss , dest_to, get_image, 200)))
           
        
    elif Cmdtype ==  start_scan:
        respons=packetconst(mysrc ,[] , dest_to, start_scan, 0)
     
        
    elif Cmdtype ==  stop_scan:
        respons=packetconst(mysrc , [], dest_to, stop_scan, 0)
        
   
        
        
        print(respons)    
    return respons
       
# groundstationsource =1
# destinationtosat    =20
# tyofdata            =0x00
# sizeofdata          =0
# cmddata=[]
# cmddata.insert(0,48)
# cmddata.insert(1,49)
# cmddata.insert(2,50)
# cmddata.insert(3,51)
# cmddata.insert(4,52)
# cmddata.insert(5,53)
# cmddata.insert(6,54)
# cmddata.insert(7,55)
# cmddata.insert(8,65)
# values=bytearray(cmddata)
cmddata=[]
cmddata.insert(0,0)
cmddata.insert(1,135)
cmddata.insert(2,0)
cmddata.insert(3,90)
cmddata.insert(4,90)
cmddata.insert(5,8)
cmddata.insert(6,22)

c=packetconst(1,bytearray(cmddata),11,15,7)
print(bytearray(c))

while True:
    
    bytesWaiting = ser.inWaiting()
    if (bytesWaiting >7):
        x=ser.read(bytesWaiting)
        if x:
            x=list(x)
            
            extracteddata =packet_exetraction(x,bytesWaiting)

            CMDResponse  =ssp_CMD_Response(extracteddata)
            
            for i in range (len(CMDResponse)):
                # ser.write(CMDResponse[i])
                # ser.write(("\n").encode('Ascii'))
                time.sleep(2)
                #print(CMDResponse[i])
                #print(x)
        # time.sleep(1)
            ser.write(bytearray(CMDResponse))
     
    
ser.close()
