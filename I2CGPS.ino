/* I2C GPS Module - see http://www.technoblogy.com/show?1LJI

   David Johnson-Davies - www.technoblogy.com - 27th September 2018
   ATtiny841 @ 8MHz (external crystal; BOD disabled)
   
   CC BY 4.0
   Licensed under a Creative Commons Attribution 4.0 International license: 
   http://creativecommons.org/licenses/by/4.0/
*/

// GPS Parser **********************************************

const long DEGREE = 600000; // Number of 1e-4 minutes in a degree

// Example: $GPRMC,092741.00,A,5213.13757,N,00008.23605,E,0.272,,180617,,,A*7F
char fmt[]="$GPRMC,dddtdd.ds,A,eeae.eeee,l,eeeae.eeee,o,jdk,c,dddy";

int state;
unsigned int temp;
long ltmp;

const int buffersize = 18;

// GPS variables
typedef union {
  struct {
    unsigned int Time, Csecs;
    long Lat, Long;
    unsigned int Knots, Course, Date;
  };
  uint8_t Data[buffersize];
} buffer_t;

buffer_t i2c, buf, gps;

volatile boolean Fix;

void ParseGPS (char c) {
  if (c == '$') { state = 0; temp = 0; ltmp = 0; }
  char mode = fmt[state++];
  // If received character matches format string, return
  if (mode == c) return;
  char d = c - '0';
  // Ignore extra digits of precision
  if (mode == ',') state--; 
  // d=decimal digit; j=decimal digits before decimal point
  else if (mode == 'd') temp = temp*10 + d;
  else if (mode == 'j') { if (c != '.') { temp = temp*10 + d; state--; } }
  // e=long decimal digit
  else if (mode == 'e') ltmp = (ltmp<<3) + (ltmp<<1) + d; // ltmp = ltmp*10 + d;
  // a=angular measure
  else if (mode == 'a') ltmp = (ltmp<<2) + (ltmp<<1) + d; // ltmp = ltmp*6 + d;
  // t=Time - hhmm
  else if (mode == 't') { gps.Time = temp*10 + d; temp = 0; }
  // s=Centisecs
  else if (mode == 's') { gps.Csecs = temp*10 + d; temp = 0; }
  // l=Latitude - in minutes*1000
  else if (mode == 'l') { if (c == 'N') gps.Lat = ltmp; else gps.Lat = -ltmp; ltmp = 0; }
  // o=Longitude - in minutes*1000
  else if (mode == 'o') { if (c == 'E') gps.Long = ltmp; else gps.Long = -ltmp; ltmp = 0; }
   // k=Speed - in knots*100
  else if (mode == 'k') { gps.Knots = temp*10 + d; temp = 0; }
  // c=Course (Track) - in degrees*100. Allow for empty field.
  else if (mode == 'c') {
    if (c == ',') { gps.Course = temp; temp = 0; state++; }
    else if (c == '.') state--;
    else { temp = temp*10 + d; state--; }
  }
  // y=Date - ddmm
  else if (mode == 'y') { gps.Date = temp*10 + d ; Fix = 1; state = 0; }
  else state = 0;
}

// I2C Interface **********************************************

const int I2CAddress = 0x3A;
volatile int Position;
const int Ready = 10; // pa0

void SetupI2C () {
  // Set up I2C slave
  TWSCRA = 1<<TWDIE | 1<<TWASIE | 1<<TWEN;
  TWSA = I2CAddress<<1;
}

// TWI interrupt
ISR(TWI_SLAVE_vect) {
  if (TWSSRA & 1<<TWASIF) {                // Address received
    if (TWSSRA & 1<<TWDIR) {               // Master read
      // Copy buf buffer to i2c buffer so it's ready to be read
      for (int i=0; i<buffersize; i++) i2c.Data[i] = buf.Data[i];
      // Take Ready pin high
      digitalWrite(Ready, HIGH);
      TWSCRB = 3<<TWCMD0;                  // ACK
    } else {                               // Master write
      TWSCRB = 3<<TWCMD0;                  // ACK
    }
  } else if (TWSSRA & 1<<TWDIF) {          // Data interrupt
    if (TWSSRA & 1<<TWDIR) {               // Master read
      if (Position >= 0 && Position < buffersize) {
        TWSD = i2c.Data[Position++];
        TWSCRB = 3<<TWCMD0;
      } else {
        TWSCRB = 2<<TWCMD0;                // NAK and complete
      }
    } else {                               // Master write
      Position = TWSD;                     // Write sets position
      TWSCRB = 3<<TWCMD0;                  // ACK
    }
  }
}

// Setup **********************************************

void setup (void) {
  SetupI2C();
  Serial.begin(9600);
  // PA0 as output
  pinMode(Ready, OUTPUT);
  digitalWrite(Ready, HIGH);
  Fix = 0;
}
 
void loop (void) {
  do {
    while (!Serial.available());
    char c = Serial.read();
    ParseGPS(c);
  } while(!Fix);
  // Copy gps buffer to buf buffer
  cli(); 
  for (int i=0; i<buffersize; i++) buf.Data[i] = gps.Data[i];
  sei();
  // Take Ready pin low to say that new data's ready
  digitalWrite(Ready, LOW);
  Fix = 0;
}
