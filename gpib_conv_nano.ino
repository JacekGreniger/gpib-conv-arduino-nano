/* GPIB to USB converter
   Arduino nano with ATmega328P
*/

/*
GPIB Connector pinout

Pin | Name  | Description        | Source            | Atmega pin 	
----+-------+--------------------+-------------------+--------------------
1   | DIO1  | Data bit 1 (LSB)   | Talker            | PC0	A0
2   | DIO2  | Data bit 2         | Talker            | PC1	A1
3   | DIO3  | Data bit 3         | Talker            | PC2	A2
4   | DIO4  | Data bit 4         | Talker            | PC3	A3
5   | EOI   | End Or Indentity   | Talker/Controller | PD2	D2
6   | DAV   | Data Valid         | Controller        | PD3	D3
7   | NRFD  | Not Ready For Data | Listener          | PD4	D4
8   | NDAC  | No Data Accepted   | Listener          | PD5	D5
9   | IFC   | Interface Clear    | Controller        | PD6	D6
10  | SRQ   | Service Request    | Talker            | PD7	D7
11  | ATN   | Attention          | Controller        | PB2	D10
12  |       | Shield             |                   |          
13  | DIO5  | Data bit 5         | Talker            | PC4	A4
14  | DIO6  | Data bit 6         | Talker            | PC5	A5
15  | DIO7  | Data Bit 7         | Talker            | PB0	D8
16  | DIO8  | Data bit 8 (MSB)   | Talker            | PB1	D9
17  | REN   | Remote Enabled     | Controller        | PB3	D11
18  |       | GND DAV            |                   |          
19  |       | GND NRFD           |                   |          
20  |       | GND NDAC           |                   |          
21  |       | GND IFC	         |                   |          
22  |       | GND SRQ	         |                   |          
23  |       | GND ATN	         |                   |	        
24  |       | GND data           |                   |	        
*/

#define EOI (1<<2)  //PD2 - D2, pin 5 GPIB
#define DAV (1<<3)  //PD3 - D3, pin 6 GPIB
#define NRFD (1<<4) //PD4 - D4, pin 7 GPIB, output
#define NDAC (1<<5) //PD5 - D5, pin 8 GPIB, output
#define IFC (1<<6)  //PD6 - D6, pin 9 GPIB
#define SRQ (1<<7)  //PD7 - D7, pin 10 GPIB
#define ATN (1<<2)  //PB2 - D10, pin 11 GPIB
#define REN (1<<3)  //PB3 - D11, pin 17 GPIB

#define SetLed(x) ( PORTB = x?(PORTB | (1<<5)) : (PORTB & ~(1<<5)) )

#define SetEOI(x) ( PORTD = (x)? (PORTD | EOI) : (PORTD & ~EOI) )
#define SetDAV(x) ( PORTD = (x)? (PORTD | DAV) : (PORTD & ~DAV) )
#define SetNRFD(x) ( PORTD = (x)? (PORTD | NRFD) : (PORTD & ~NRFD) )
#define SetNDAC(x) ( PORTD = (x)? (PORTD | NDAC) : (PORTD & ~NDAC) )
#define SetIFC(x) ( PORTD = (x)? (PORTD | IFC) : (PORTD & ~IFC) )
#define SetSRQ(x) ( PORTD = (x)? (PORTD | SRQ) : (PORTD & ~SRQ) )
#define SetATN(x) ( PORTB = (x)? (PORTB | ATN) : (PORTB & ~ATN) )
#define SetREN(x) ( PORTB = (x)? (PORTB | REN) : (PORTB & ~REN) )

#define ESC_KEY_UP 0x41
#define ESC_KEY_DOWN 0x42
#define ESC_KEY_RIGHT 0x43
#define ESC_KEY_LEFT 0x44

#define MAX_COMMANDS 15
#define BUF_SIZE 64
#define GPIB_BUF_SIZE 128
#define GPIB_MAX_RECEIVE_TIMEOUT 50000
#define GPIB_MAX_TRANSMIT_TIMEOUT 50000
#define EMPTY_LINE 1

#define DEFAULT_ADDRESS 21

#define HELP_LINES 18
#define HELP_STRING_LEN 64

const char helpStrings[HELP_LINES][HELP_STRING_LEN] PROGMEM = {
  "GPIB to USB converter v4\r\n\r\n",
  "Transmit commands, OK/TIMEOUT/ERROR\r\n",
  "  <D> Data (ATN false), <M> Data without EOI\r\n",
  "  <C> Command (ATN true)\r\n",
  "  <T> Hex transmit (0C - command, 0D - data)\r\n",
  "Receive commands (receives until EOI,max 127 bytes)\r\n",
  "  <X> ASCII, <payload> or TIMEOUT\r\n",
  "  <Y> BINARY, <length><payload>\r\n",
  "  <Z> HEX, <length><payload>\r\n",
  "  <P> Continous read (plotter mode)\r\n",
  "General commands\r\n",
  "  <A> Set/get converter talk address\r\n",
  "  <S> Get REQ/SRQ/LISTEN state (1 if true)\r\n",
  "  <R> Set REMOTE mode (REN true)\r\n",
  "  <L> Set LOCAL mode (REN false)\r\n",
  "  <I> Generate IFC pulse\r\n",
  "  <E> Get/set echo on(E1)/off(E0)\r\n",
  "  <H> Commands history\r\n"
};


typedef enum {OFF = 0, SLOW, FAST} ledBlinking_t;
ledBlinking_t ledBlinking = OFF;
unsigned char listenAddress = DEFAULT_ADDRESS;
unsigned char msgEndSeq = 0;
unsigned char remoteState = 0;


#define FOSC 16000000UL // Clock Speed
#define BAUD 115200UL
#define MYUBRR FOSC/8/BAUD-1 //U2X set to 1

// The UDRE Flag indicates if the transmit buffer (UDR) is ready to receive new data
#define UARTTransmitBufferEmpty() ( UCSR0A & (1<<UDRE0))

// This flag bit is set when there are unread data in the receive buffer and cleared after UDR read
#define UARTDataAvailable() (UCSR0A & (1<<RXC0))

/* USART on */
void UART_init (void) {
  unsigned int ubrr = MYUBRR;
  UBRR0H = (unsigned char)(ubrr>>8);
  UBRR0L = (unsigned char)ubrr;
  UCSR0A = (1<<U2X0);
  /* Enable receiver and transmitter */
  UCSR0B = (1<<RXEN0) | (1<<TXEN0);
  /* Set frame format: 8data, 1 stop bit, no parity */
  UCSR0C = (3<<UCSZ00);
}

void UART_transmit( unsigned char data ) {
  /* Wait for empty transmit buffer */
  while ( !UARTTransmitBufferEmpty() );
  /* Put data into buffer, sends the data */
  UDR0 = data;
}

unsigned char UART_receive( void ) {
  /* Wait for data to be received */
  while ( !UARTDataAvailable() );
  /* Get and return received data from buffer */
  return UDR0;
}


unsigned char DataRead()
{
  unsigned char val;
  
  val = PINC & 0x3f;
  val |= (PINB & 0x03) << 6;
  
  return val;
}

void DataWrite(unsigned char val)
{
  PORTC = val&0x3f;
  PORTB = (PORTB&0xfc) | (val>>6);
}


/* ======================================================= */

void GPIO_init() {
  // DDR = 1 output
  // DDR = 0 input
  DDRB = (1<<5); // PB5 output, PB4 input
  PORTB = (1<<4);
}


int uart_putchar(char ch, FILE* file)
{
  UART_transmit(ch);
  return ch;
}


void ReconfigureGPIO_GPIBReceiveMode()
{
  DDRB = (1<<5) | ATN | REN; // these lines are outputs, other as inputs
  PORTB = (1<<4) | 0x03 | ATN | (remoteState?0:REN); // pullup on

  DDRC = 0x00; // PC0-PA5 inputs
  PORTC = 0x3f; // pullup on
  
  DDRD = IFC | REN | NRFD | NDAC; // these lines are outputs, other as inputs
  PORTD = IFC | (remoteState?0:REN) | EOI | DAV | SRQ; // pullup on
}


void ReconfigureGPIO_GPIBNormalMode()
{
  DDRC = 0x3f; //outputs

  DDRB = (1<<5) | ATN | REN | 0x03; // these lines are outputs
  PORTB = (1<<4) | ATN | (remoteState?0:REN); // pullup on

  DataWrite(0x00); // output level 0

  DDRD = IFC | EOI | DAV; // these lines are outputs
  PORTD = IFC | EOI | DAV | SRQ | NRFD | NDAC; // pullup on
}


int GPIB_Receive(unsigned char * buf, unsigned char bufLength, unsigned char * receivedLength)
{
  unsigned char index = 0;
  unsigned char c;
  unsigned int timeout;

  do
  {
    SetNRFD(1); //ready for receiving data
    //-1 & 5
    
    timeout = 0;
    while (PIND & DAV) // waiting for falling edge
    {
      timeout++;
      if (timeout > GPIB_MAX_RECEIVE_TIMEOUT)
      {
        *receivedLength = index;
        SetNRFD(0);
        return 0;
      }
    }
    // 0
    
    SetNRFD(0); //not ready for receiving data
    // 1
    
    c = ~DataRead(); //read data

    buf[index++] = c;

    SetNDAC(1); //data accepted
    //2
    
    while (!(PIND & DAV)) // waiting for rising edge
    {
      //delay(1);
      timeout++;
      if (timeout > GPIB_MAX_RECEIVE_TIMEOUT)
      {
        *receivedLength = index;
        SetNDAC(0);
        return 0;
      }
    }
    //3
    
    SetNDAC(0);
    //4
  } while ((index < bufLength) /*&& (c != 13)*/);
  *receivedLength = index;
  return 255;
}


int GPIB_Receive_till_eoi(unsigned char * buf, unsigned char bufLength, unsigned char * receivedLength)
{
  unsigned char index = 0;
  unsigned char c;
  unsigned char eoi = 0;
  unsigned int timeout;

  do
  {
    SetNRFD(1); //ready for receiving data
    //-1 & 5
    
    timeout = 0;
    while (PIND & DAV) // waiting for falling edge
    {
      timeout++;
      if (timeout > GPIB_MAX_RECEIVE_TIMEOUT)
      {
        *receivedLength = index;
        SetNRFD(0);
        return 0;
      }
    }
    // 0
    
    if ((PIND & EOI) == 0)
      eoi = 1;
    
    SetNRFD(0); //not ready for receiving data
    // 1
    
    c = ~DataRead(); //read data

    buf[index++] = c;

    SetNDAC(1); //data accepted
    //2
    
    while (!(PIND & DAV)) // waiting for rising edge
    {
      timeout++;
      if (timeout > GPIB_MAX_RECEIVE_TIMEOUT)
      {
        *receivedLength = index;
        SetNDAC(0);
        return 0;
      }
    }
    //3
    
    SetNDAC(0);
    //4
  } while ((index < bufLength) && (eoi == 0));
  *receivedLength = index;
  return 255;
}


int GPIB_Receive_till_lf(unsigned char * buf, unsigned char bufLength, unsigned char * receivedLength)
{
  unsigned char index = 0;
  unsigned char c;
  unsigned int timeout;

  //SetNDAC(0);
  //SetNRFD(0);

  do
  {
    SetNRFD(1); //ready for receiving data
    //-1 & 5
    
    timeout = 0;
    while (PIND & DAV) // waiting for falling edge
    {
      timeout++;
      if (timeout > GPIB_MAX_RECEIVE_TIMEOUT)
      {
        *receivedLength = index;
        SetNRFD(0);
        return 0;
      }
    }
    // 0
       
    SetNRFD(0); //not ready for receiving data
    // 1
    
    c = ~DataRead(); //read data

    buf[index++] = c;

    SetNDAC(1); //data accepted
    //2
    
    while (!(PIND & DAV)) // waiting for rising edge
    {
      timeout++;
      if (timeout > GPIB_MAX_RECEIVE_TIMEOUT)
      {
        *receivedLength = index;
        SetNDAC(0);
        return 0;
      }
    }
    //3
    
    SetNDAC(0);
    //4
  } while ((index < bufLength) && (c != 10));
  *receivedLength = index;
  return 255;
}


int GPIB_Transmit(unsigned char * buf, unsigned char bufLength, unsigned char eoi)
{
  unsigned char index = 0;
  unsigned int timeout;
  
  if ((0 == bufLength) || ((PIND & NRFD) && (PIND & NDAC)))
    return 0;
  
  do
  {
    if ((index+1 == bufLength) && eoi)
      SetEOI(0); // last byte
    
    //transmit debug    
    //printf("%02x ", buf[index]);
    
    DataWrite(~buf[index]);
    index++;
    
    delayMicroseconds(100);
     
    timeout = 0;
    while (!(PIND & NRFD)) // waiting for high on NRFD
    {
      timeout++;
      if (timeout > GPIB_MAX_TRANSMIT_TIMEOUT)
      {
        SetEOI(1);
        return 0;
      }
    }
    
    SetDAV(0);
    delayMicroseconds(100);
   
    while (!(PIND & NDAC)) // waiting for high on NDAC
    {
      timeout++;
      if (timeout > GPIB_MAX_TRANSMIT_TIMEOUT)
      {
        SetEOI(1);
        SetDAV(1);
        return 0;
      }
    }
    
    SetEOI(1);
    SetDAV(1);
    //4
  } while ((index < bufLength));

  //printf("\r\n");
  return 255;
}


void ShowHelp()
{
  char buf[64];
  unsigned char i;

  for (i=0; i<HELP_LINES; i++)
  {
    memcpy_P(buf, helpStrings[i], HELP_STRING_LEN);
    printf("%s", buf);
  }
}


char UART_RcvEscapeSeq()
{
  while (!UARTDataAvailable());
  if (UART_receive() != 0x5B)
    return 0;
  while (!UARTDataAvailable());
  return UART_receive();
}

#define T1_INIT 64910 // preload timer 65536-16MHz/256/100Hz

ISR(TIMER1_OVF_vect)
{
  static unsigned char timCnt = 0;
  static unsigned char led = 0;
  TCNT1 = T1_INIT; 
  if (OFF == ledBlinking)
    return;
	
  timCnt++;
  if (timCnt >= ((ledBlinking==SLOW)?25:5))
  {
    timCnt = 0;
    led = !led;
    SetLed(led);
  }
}


#define ishexdigit(x) \
       (((x >= '0') && (x <= '9')) ||   \
        ((x >= 'A') && (x <= 'F')))


unsigned char hex2dec(unsigned char x)
{
  if ((x >= '0') && (x <= '9'))
    return (x-'0');
  else
    return (10+x-'A');
}


unsigned char CheckHexMsg(unsigned char * buf, unsigned char len, unsigned char *outputMsg, unsigned char *outputLen, unsigned char *eoi)
{
  unsigned char i; 
  *eoi = 1; //default

  if (('D'==toupper(buf[1])) && (';'==buf[len-1]))
  {
    --len;
    *eoi = 0;
  }

  if ((len & 0x01) || (len < 4))
    return 0; //msg length is not even
    
  if (('0'!=buf[0]) || (('C'!=toupper(buf[1])) && ('D'!=toupper(buf[1]))))
    return 0;
    
  for (i=2; i<len; i++)
  {
    if (!ishexdigit(toupper(buf[i])))
      return 0;
  }

  *outputLen = 0;
  
  for (i=2; i<len; i=i+2)
  {
    *outputMsg = hex2dec(toupper(buf[i])) << 4;
    *outputMsg += hex2dec(toupper(buf[i+1]));
    //printf("%d ", *outputMsg);
    outputMsg++;
    *outputLen += 1;
  }
  //printf("\r\n");
  return 1;
}

char commandsHistory[BUF_SIZE*MAX_COMMANDS];
char savedCommands = 0;
char selectedCommand = 0;

unsigned char listenMode = 0;
unsigned char listenMode_prev = 0;

unsigned char buf[BUF_SIZE+4];
unsigned char msgBuf[BUF_SIZE+4];
unsigned char gpibBuf[GPIB_BUF_SIZE];

void setup()
{
}

void loop() 
{
  unsigned char bufPos = 0;
  unsigned char cursorPos = 0;
  unsigned char localEcho = 1;
  unsigned char c;
  int i;
  unsigned char gpibIndex = 0;
  unsigned char command = 0;
  int result = 0;
  unsigned char msgLen = 0;
  unsigned char msgEOI = 1;

  GPIO_init();

#if 1  
  // initialize Timer1
  noInterrupts(); // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
	 
  TCNT1 = 64910; // preload timer 65536-16MHz/256/100Hz
  TCCR1B |= (1 << CS12); // 256 prescaler
  TIMSK1 |= (1 << TOIE1); // enable timer overflow interrupt
  interrupts(); // enable all interrupts
#endif

  ReconfigureGPIO_GPIBNormalMode();
  UART_init();
  fdevopen(uart_putchar, NULL);
 
#if 1 
  if (0 == (PINB & (1<<4))) // printer mode
  {
    ledBlinking = SLOW;
    ReconfigureGPIO_GPIBReceiveMode();
    delay(1);

    while (1)
    {
      result = GPIB_Receive(gpibBuf, GPIB_BUF_SIZE-2, &gpibIndex);
      if (gpibIndex != 0)
      {
        for (i=0; i<gpibIndex; i++)
          UART_transmit(gpibBuf[i]);
      }
      else
      {
        delay(10);
      }
    }
  }

  //localEcho = (PINB & _BV(PB7))?1:0;
#endif

  while (1) //main loop
  {
    selectedCommand = savedCommands;
    
    if (localEcho && !bufPos)
      printf("<GPIB> ");
      
    do
    {
      while (!UARTDataAvailable());
      c = UART_receive();

      if (0x08 == c) //backspace
      {
        if ((bufPos > 0) && (cursorPos == bufPos))
        {
          --bufPos;
          --cursorPos;
          if (localEcho)
          {
            UART_transmit(0x08);
            UART_transmit(' ');
            UART_transmit(0x08);
          }
        }
        else if ((bufPos > 0) && (cursorPos > 0))
        {
          --bufPos;
          --cursorPos;
          memmove(&buf[cursorPos], &buf[cursorPos+1], bufPos-cursorPos);
          if (localEcho)
          {
            UART_transmit(0x08);
            buf[bufPos] = 0;
            printf("%s ", &buf[cursorPos]);
            for (i=cursorPos; i<(bufPos+1); i++)
              UART_transmit(0x08);
          }
        }
      }
      else if (10 == c) //ignore LF
      {
      }
/*      else if (9 == c) //tab key
      {
        printf("<bufPos=%d cursorPos=%d>", bufPos, cursorPos);
      }
*/      else if (0x1b == c) //escape character
      {
        switch (UART_RcvEscapeSeq())
        {
          case ESC_KEY_UP:
            selectedCommand = selectedCommand?selectedCommand-1:0;
            memmove(&buf[0], &commandsHistory[selectedCommand*BUF_SIZE], BUF_SIZE);
            if (localEcho)
            {
              while (cursorPos < bufPos)
              {
                UART_transmit(' ');
                cursorPos++;
              }
              while (bufPos--)
              {
                UART_transmit(0x08);
                UART_transmit(' ');
                UART_transmit(0x08);
              }
              printf("%s", &buf[0]);
            }
            bufPos = strlen((char*)&buf[0]);
            cursorPos = bufPos;
            break;
            
          case ESC_KEY_DOWN:
            if ((selectedCommand+1) == savedCommands) //current command is last command in buffer
            {
              selectedCommand = savedCommands;
              if (localEcho)
              {
                while (cursorPos < bufPos)
                {
                  UART_transmit(' ');
                  cursorPos++;
                }
                while (bufPos--)
                {
                  UART_transmit(0x08);
                  UART_transmit(' ');
                  UART_transmit(0x08);
                }
              }
              bufPos = 0;
              cursorPos = 0;
            }
            else if ((selectedCommand+1) < savedCommands) // <MAX_COMMANDS
            {
              selectedCommand++;
              memmove(&buf[0], &commandsHistory[selectedCommand*BUF_SIZE], BUF_SIZE);
              if (localEcho)
              {
                while (cursorPos < bufPos)
                {
                  UART_transmit(' ');
                  cursorPos++;
                }
                while (bufPos--)
                {
                  UART_transmit(0x08);
                  UART_transmit(' ');
                  UART_transmit(0x08);
                }
                printf("%s", &buf[0]);
              }
              bufPos = strlen((char*)&buf[0]);
              cursorPos = bufPos;
            }
            break;
            
          case ESC_KEY_LEFT:
            if (cursorPos)
            {
              --cursorPos;
              if (localEcho)
              {
                UART_transmit(0x1B);
                UART_transmit(0x5B);
                UART_transmit('D');
              }
            }
            break;
            
          case ESC_KEY_RIGHT:
            if (cursorPos < bufPos)
            {
              cursorPos++;
              if (localEcho)
              {
                UART_transmit(0x1B);
                UART_transmit(0x5B);
                UART_transmit('C');
              }
            }
            break;
            
          default:
            break;
        }
      }
      else if (13 == c)
      {
        if (localEcho)
        {
          UART_transmit(13); //CR
          UART_transmit(10); //LF
        }
		
        if (bufPos)
          command = toupper(buf[0]);
        else
          command = EMPTY_LINE;
      }
      else
      {
        if (bufPos < BUF_SIZE-1)
        {
          if (cursorPos == bufPos)
          {
            buf[bufPos++] = c;
            cursorPos++;
            if (localEcho)
              UART_transmit(c); //local echo
          }
          else
          {
            memmove(&buf[cursorPos+1], &buf[cursorPos], bufPos-cursorPos);
            buf[cursorPos++] = c;
            bufPos++;
            buf[bufPos] = 0;
            if (localEcho)
            {
              UART_transmit(c); //local echo
              printf("%s", &buf[cursorPos]);
              for (i=cursorPos; i<bufPos; i++)
                UART_transmit(0x08);
            }
          }
        }
      }
    } while (!command);
    

    if ('D' == command) //send data
    {
      if (!listenMode)
      {
        if (1 == msgEndSeq)
          buf[bufPos++] = 13; //CR
        else if (2==msgEndSeq)
          buf[bufPos++] = 10; //LF
        else if (3==msgEndSeq)
        {
          buf[bufPos++] = 13; //CR
          buf[bufPos++] = 10; //LF
        }
		
        result = GPIB_Transmit(buf+1, bufPos-1, 1); 
        if (result == 255) // transmit ok
          printf("OK\r\n");
        else //timeout
          printf("TIMEOUT\r\n");

        if ((1==msgEndSeq) || (2==msgEndSeq))
          --bufPos;
        else if (3==msgEndSeq)
          bufPos -= 2;
      }
      else
        printf("ERROR\r\n");	  
    }
    else if ('M' == command) //send data without EOI
    {
      if (!listenMode)
      {
        if (1 == msgEndSeq)
          buf[bufPos++] = 13; //CR
        else if (2==msgEndSeq)
          buf[bufPos++] = 10; //LF
        else if (3==msgEndSeq)
        {
          buf[bufPos++] = 13; //CR
          buf[bufPos++] = 10; //LF
        }
		
        result = GPIB_Transmit(buf+1, bufPos-1, 0); 
        if (result == 255) // transmit ok
          printf("OK\r\n");
        else //timeout
          printf("TIMEOUT\r\n");

        if ((1==msgEndSeq) || (2==msgEndSeq))
          --bufPos;
        else if (3==msgEndSeq)
          bufPos -= 2;
      }
      else
        printf("ERROR\r\n");	  
    }
    else if ('C' == command) //send command
    {
      for (i=1; i<bufPos; i++)
      {
        if ((buf[i] == '?') || (buf[i] == (64+listenAddress)))//unlisten
        {
          listenMode = 0;
          ledBlinking = OFF;
          SetLed(1);
        }
        else if (buf[i] == (32+listenAddress))
        {
          listenMode = 1;
          ledBlinking = FAST;
        }
      }

      if (1 == msgEndSeq)
        buf[bufPos++] = 13; //CR
      else if (2==msgEndSeq)
        buf[bufPos++] = 10; //LF
      else if (3==msgEndSeq)
      {
        buf[bufPos++] = 13; //CR
        buf[bufPos++] = 10; //LF
      }

      ReconfigureGPIO_GPIBNormalMode();

      SetATN(0);
      delayMicroseconds(100);
      result = GPIB_Transmit(buf+1, bufPos-1, 1);
     
      if (result == 255) // transmit ok
        printf("OK\r\n");
      else //timeout
        printf("TIMEOUT\r\n");

      SetATN(1);
	  
      if ((1==msgEndSeq) || (2==msgEndSeq))
        --bufPos;
      else if (3==msgEndSeq)
        bufPos -= 2;
      
      if (listenMode)
        ReconfigureGPIO_GPIBReceiveMode();
      else
        ReconfigureGPIO_GPIBNormalMode();
		  
      listenMode_prev = listenMode;
    }
    else if ('R' == command)
    {
      SetREN(0);
      remoteState = 1;
      printf("OK\r\n");
    }
    else if ('L' == command)
    {
      SetREN(1);
      remoteState = 0;
      printf("OK\r\n");
    }
    else if ('I' == command)
    {
      SetIFC(0);
      delay(1);
      SetIFC(1);
      printf("OK\r\n");
    }
    else if ('S' == command)
    {
      UART_transmit(remoteState?'1':'0');
      UART_transmit((0 == (PINC & SRQ))?'1':'0');
      UART_transmit(listenMode?'1':'0');
      UART_transmit(13);
      UART_transmit(10);
    }
    else if ('P' == command)
    {
      listenMode_prev = 0; //cancel listen mode
      listenMode = 0; //cancel listen mode
      ledBlinking = SLOW;
      ReconfigureGPIO_GPIBReceiveMode();
//      if (localEcho)
//        printf("PRINTER MODE, send <ESC> to return to normal mode\r\n");
        
      delay(1);
      while (c != 27)
      {
        if (UARTDataAvailable())
          c = UART_receive();
          
        result = GPIB_Receive(gpibBuf, GPIB_BUF_SIZE-2, &gpibIndex);
        if (gpibIndex != 0)
        {
          for (i=0; i<gpibIndex; i++)
            UART_transmit(gpibBuf[i]);
        }
        else
        {
          delay(10);
        }
      }
      c = 0;
      ReconfigureGPIO_GPIBNormalMode();
      ledBlinking = OFF;
      SetLed(1);
    }
    else if ('X' == command) //ascii receive
    {
      if (!listenMode)
      {
        ReconfigureGPIO_GPIBReceiveMode();
        delay(1);
      }
      result = GPIB_Receive_till_eoi(gpibBuf, GPIB_BUF_SIZE-2, &gpibIndex);
      
      if (gpibIndex != 0)
      {
        gpibBuf[gpibIndex] = 0;
        printf("%s",gpibBuf);
      }
      else
        printf("TIMEOUT\r\n");

      if (!listenMode)
        ReconfigureGPIO_GPIBNormalMode();
    }
    else if ('Y' == command) //binary receive
    {
      if (!listenMode)
      {
        ReconfigureGPIO_GPIBReceiveMode();
        delay(1);
      }

      result = GPIB_Receive_till_eoi(gpibBuf, GPIB_BUF_SIZE-2, &gpibIndex);
      UART_transmit(gpibIndex);
      for (i=0; i<gpibIndex; i++)
      {
        UART_transmit(gpibBuf[i]);
      }
      
      if (!listenMode)
        ReconfigureGPIO_GPIBNormalMode();
    }
    else if ('Z' == command) //hex receive
    {
      if (!listenMode)
      {
        ReconfigureGPIO_GPIBReceiveMode();
        delay(1);
      }
      result = GPIB_Receive_till_eoi(gpibBuf, GPIB_BUF_SIZE-2, &gpibIndex);

      printf("%02x", gpibIndex);
      for (i=0; i<gpibIndex; i++)
        printf("%02x",gpibBuf[i]);
      printf("\r\n");
	  
      if (!listenMode)
        ReconfigureGPIO_GPIBNormalMode();
    }
    else if ('?' == command)
    {
      ShowHelp();
    }
    else if ('E' == command)
    {
      if (bufPos == 1)
        printf("%d\r\n", localEcho);
      else if ((bufPos==2) && ('0' == buf[1]))
      {
        localEcho = 0;
        printf("OK\r\n");
      }
      else if ((bufPos==2) && ('1' == buf[1]))
      {
        localEcho = 1;
        printf("OK\r\n");
      }
      else
        printf("ERROR\r\n");
    }
    else if ('H' == command) //show history
    {
      for (i=0; i<savedCommands; i++)
        printf("%d: %s\r\n", i, &commandsHistory[i*BUF_SIZE]);
        
      command = 0; //to avoid saving this command in history
    }
    else if ('A' == command) //listen address
    {
      if (bufPos == 1)
        printf("%02d\r\n", listenAddress);
      else if ((bufPos==3) && isdigit(buf[1]) && isdigit(buf[2])) 
      {
        i = atoi((char*)buf+1);
        if ((i>=0) && (i<=30))
        {
          listenAddress = i;
          printf("OK\r\n");
        }
        else
          printf("ERROR\r\n");
      }
      else
        printf("ERROR\r\n");
    }
    else if ('Q' == command) 
    {
      if (bufPos == 1)
        printf("%d\r\n", msgEndSeq);
      else if ((2==bufPos) && (('0'==buf[1]) ||  ('1'==buf[1]) || ('2'==buf[1]) || ('3'==buf[1])))
      {
        if ('0'==buf[1])
          msgEndSeq = 0;
        else if ('1'==buf[1])
          msgEndSeq = 1;
        else if ('2'==buf[1])
          msgEndSeq = 2;
        else if ('3'==buf[1])
          msgEndSeq = 3;
		  
        printf("OK\r\n");
      }
      else
        printf("ERROR\r\n");
    }
    else if ('T' == command) 
    {
      if (CheckHexMsg(&buf[1], bufPos-1, msgBuf, &msgLen, &msgEOI))
      {
        if ('D' == toupper(buf[2])) //send data
        {	
          result = GPIB_Transmit(msgBuf, msgLen, msgEOI); 
          if (result == 255) // transmit ok
            printf("OK\r\n");
          else //timeout
            printf("TIMEOUT\r\n");
        }
        else //send command
        {
          for (i=0; i<msgLen; i++)
          {
            if ((msgBuf[i] == '?') || (msgBuf[i] == (64+listenAddress)))//unlisten
            {
              listenMode = 0;
              ledBlinking = OFF;
              SetLed(1);
            }
            else if (buf[i] == (32+listenAddress))
            {
              listenMode = 1;
              ledBlinking = FAST;
            }
          }

          ReconfigureGPIO_GPIBNormalMode();

          SetATN(0);
          delayMicroseconds(100);
          result = GPIB_Transmit(msgBuf, msgLen, 1);
     
          if (result == 255) // transmit ok
            printf("OK\r\n");
          else //timeout
            printf("TIMEOUT\r\n");

          SetATN(1);
	       
          if (listenMode)
            ReconfigureGPIO_GPIBReceiveMode();
          else
            ReconfigureGPIO_GPIBNormalMode();
		  
          listenMode_prev = listenMode;
        }      
      }
      else
        printf("ERROR\r\n");
    }
    else
    {
      if (bufPos)
        printf("WRONG COMMAND\r\n");
      command = 0;
    }

    if (command && bufPos)
    {
      buf[bufPos] = 0; //add string termination

      //avoids saving same command twice    
      if ((savedCommands > 0) && (0 == strcmp(&commandsHistory[(savedCommands-1)*BUF_SIZE], (char*)&buf[0])))
      {
        command = 0;
      }
      else //save command
      {
        if (savedCommands < MAX_COMMANDS)
        {
          memmove(&commandsHistory[savedCommands*BUF_SIZE], &buf[0], BUF_SIZE);
          ++savedCommands;
        }
        else
        {
          memmove(&commandsHistory[0], &commandsHistory[BUF_SIZE], BUF_SIZE*(savedCommands-1));
          memmove(&commandsHistory[(savedCommands-1)*BUF_SIZE], &buf[0], BUF_SIZE);
        }
      }
    }
	
    command = 0;
    bufPos = 0;
    cursorPos = 0;
    buf[0] = 0;
  } //end of endless loop block
}

