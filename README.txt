README

1. Code Composer Studio v6 for Tiva C Series is used to develop the project.
2. The required TivaWare folder was downloaded and used for linking the driver libraries.
3. The SimpleLink Wi-Fi CC3100 SDK folder was used for the development of this project.
4. The lab was executed using main.c from udp_socket folder from CC3100 SDK folder as the base file and 
   the changes were made to the main.c file according to the requirements for both server and client.
5. The predefined symbols like __CCS__, __SL__ were added to the project properties for functioning and 
   connection to the network.
6. The header file sl_common.h was edited according to the network it was intended to connect.
   (Here, SSID_NAME "Embedded_Lab_EXT", PASSKEY "embedded" and SEC_TYPE "WPA2").
7. The function SimpleLinkNetAppEventHandler() is used to get the IP address of the server board which is 
   then printed on the CCS terminal in the format A.B.C.D as per the requirement. Here, it was printed as
   192.168.2.47 on the terminal and then the function BsdUdpServer() function is called where the receiving
   of the messages takes place and eventually displays the ambient temperature on the LCD.
8. On the client side, the IP_ADDR macro is changed according to the server CC3100 in Hex format. 
   (Here, 0xC0A8022F).
9. After calling the BsdUdpClient() function the sensor value is sent continously on to the server using the
   UDP communication protocol where the client packs the messages in UDP packets and server unpacks the received 
   packets.
10.The client side Tiva board enables the 12-bit ADC on function call of adc_init(). The ADC converts the 
   analog input from the sensor packs it into UDP packets.
   WORKING:
  [The ADC value is calculated and then divided by 20, because the value of LM35 ranges from -55 to 150 degree
   celcius which is 205 units. Thus, as the highest value of the 12-bit ADC is 4095, the value acquired by 
   4095/205 = 20 units per centigrade. The value is converted fronm integer to character and stored into the buffer.
   This is done by getting each digit from hundredth place to units place and then adding decimal value(48) of char 
   '0' to it [i.e temp+48 in this case.]
11.The functions sl_SendTo() and sl_RecvFrom() along with the buffer uBuf.BsdBuf[] is used by the client 
   and server respectively to send and receive the data via UDP communication. 