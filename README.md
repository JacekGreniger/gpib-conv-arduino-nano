gpib-conv-arduino-nano
======================

GPIB to USB converter

Ported to Arduino nano board (with ATMega328P)

Connect Arduino pins to the Centronics connector using table below. All the GND can be connected together.

Connect D12 to GND to enter printer mode immediately after reset.



Pin  Name    Description          Source              Arduino nano pin

1    DIO1    Data bit 1 (LSB)     Talker              PC0   A0

2    DIO2    Data bit 2           Talker              PC1   A1

3    DIO3    Data bit 3           Talker              PC2   A2

4    DIO4    Data bit 4           Talker              PC3   A3

5    EOI     End Or Indentity     Talker/Controller   PD2   D2

6    DAV     Data Valid           Controller          PD3   D3

7    NRFD    Not Ready For Data   Listener            PD4   D4

8    NDAC    No Data Accepted     Listener            PD5   D5

9    IFC     Interface Clear      Controller          PD6   D6

10   SRQ     Service Request      Talker              PD7   D7

11   ATN     Attention            Controller          PB2   D10

12           Shield                                            

13   DIO5    Data bit 5           Talker              PC4   A4

14   DIO6    Data bit 6           Talker              PC5   A5

15   DIO7    Data Bit 7           Talker              PB0   D8

16   DIO8    Data bit 8 (MSB)     Talker              PB1   D9

17   REN     Remote Enabled       Controller          PB3   D11

18           GND DAV                                           

19           GND NRFD                                          

20           GND NDAC   

21           GND IFC    

22           GND SRQ    

23           GND ATN    

24           GND data   
