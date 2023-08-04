#include <SPI.h>

const int snesReadPin = A1;   //Cart pin 23 - aka /RD, CE - Address bus read
const int snesWritePin = A2;  //Cart pin 54 - aka /WR - Address bus write
const int snesCartPin = A3;   //Cart pin 49 - aka /CS, OE, /ROMSEL, /Cart - Goes low when reading ROM
// const int snesResetPin = A4;  //Cart pin 26 - SNES reset pin. Goes high when reading cart
const int AddressLatchPin = 10;

const int pin_we = A0;  //Write Enable (#WE)
const int pin_ce = A1;  //Chip Enable (#CE) 
const int pin_oe = A3;  //Output Enable (#OE)

const int pinSequence[] = { 8, 9, 2, 3, 4, 5, 6, 7 };  //eeprom: D0, D1, D2...
const int sector_size = 256;

typedef enum COMMANDS {
  CTRL,
  READSECTION,
  WRITESECTION,
  FLASHSECTION
};

void setup() {
  //begin serial
  Serial.begin(115200);
  //to increase speed, uncomment above lines if you arduino have ftdi
  //Serial.begin(2000000);
  //UBRR0 = 0; //max baud rate
  bitSet(UCSR0A, U2X0);  // change UART divider from 16 to 8  for double transmission speed

  //begin spi
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV2);  //8MHz

  setDataBusDir(INPUT);

  //setup cart control pins
  pinMode(snesCartPin, OUTPUT);
  pinMode(snesReadPin, OUTPUT);
  pinMode(snesWritePin, OUTPUT);
  // pinMode(snesResetPin, OUTPUT);

  digitalWrite(pin_we, HIGH);
  pinMode(pin_we, OUTPUT);
  //read the rom header
  //readHeader();

  //Write byte to indicate device ready
  Serial.write(0x00);
}

void loop() {
  int incomingByte = serialReadBlocking();
  switch (incomingByte) {
    // set the cartridge control lines
    case CTRL:
      {
        setCtrlLines(serialReadBlocking());
      }
      break;

    // read a section of data from the cart
    case READSECTION:
      {
        byte bank = serialReadBlocking();
        byte startAddrHi = serialReadBlocking();
        byte startAddrLow = serialReadBlocking();
        byte endAddrHi = serialReadBlocking();
        byte endAddrLow = serialReadBlocking();

        unsigned int addr = bytesToInt(startAddrHi, startAddrLow);
        unsigned int endAddr = bytesToInt(endAddrHi, endAddrLow);
        setDataBusDir(INPUT);

        while (true) {
          writeAddrBus(bank, addr);
          Serial.write(readDataBus());

          /* We must break out of the loop this way because of the potential case
         * where addresses 0x0000 - 0xffff (inclusive) must be used. A standard
         * loop won't work because it will overflow the 16 bit unsigned int.
         */
          if (addr == endAddr) {
            break;
          }
          addr++;
        }
        Serial.flush();
      }
      break;

    case WRITESECTION:
      {
        byte bank = serialReadBlocking();
        byte startAddrHi = serialReadBlocking();
        byte startAddrLow = serialReadBlocking();
        byte endAddrHi = serialReadBlocking();
        byte endAddrLow = serialReadBlocking();

        unsigned int addr = bytesToInt(startAddrHi, startAddrLow);
        unsigned int endAddr = bytesToInt(endAddrHi, endAddrLow);
        setDataBusDir(OUTPUT);

        while (true) {
          writeAddrBus(bank, addr);
          writeDataBus(serialReadBlocking());

          /* We must break out of the loop this way because of the potential case
         * where addresses 0x0000 - 0xffff (inclusive) must be used. A standard
         * loop won't work because it will overflow the 16 bit unsigned int.
         */
          if (addr == endAddr) {
            break;
          }
          addr++;
        }
      }
      break;


    case FLASHSECTION:

      unsigned int receivedBytes = 0;
      unsigned int addr = 0;
      byte block[sector_size];
      byte bank = 0;
      byte num_banks = serialReadBlocking();
      
      while (true) {
        //wait for the block be filled according to the eeprom sector size
        while (receivedBytes < sector_size) {
          block[receivedBytes] = serialReadBlocking();
          receivedBytes++;
        }

        writeBlock(bank, addr, block);
        receivedBytes = 0;
        addr += sector_size;

        if (bank >= num_banks) {
          Serial.println("");
          break;
        }

        if (addr >= 0x8000) {   //lorom
          bank++;
          addr = 0;
        }

        Serial.println("");
      }

      break;
  }
}

byte serialReadBlocking() {
  while (Serial.available() == 0)
    ;
  return Serial.read();
}

unsigned int bytesToInt(byte hi, byte low) {
  return ((unsigned int)hi << 8) | low;
}

void setCtrlLines(byte s) {
  digitalWrite(snesReadPin, s & 0x8);
  digitalWrite(snesWritePin, s & 0x4);
  digitalWrite(snesCartPin, s & 0x2);
  // digitalWrite(snesResetPin, s & 0x1);
}

/* Write a value out to the address bus
 * Uses direct port manipulation and
 * hardware spi for better performance
 */
void writeAddrBus(byte bank, unsigned int addr) {
  PORTB &= ~(B100);         //Set AddressLatchPin low
  SPI.transfer(bank);       // shift out bank
  SPI.transfer(addr >> 8);  // shift out address upper byte
  SPI.transfer(addr);       // shift out address lower byte
  PORTB |= (B100);          //Set AddressLatchPin high
}

//Read byte from data bus
byte readDataBus() {
  //Digital pins 2-7 (PIND) are connected to snes data lines 2-7.
  //Digital pins 8 and 9 (PINB) are connected to data lines 0 and 1 respectively
  //This line of code takes the data from pins 2-7 and from 8&9 and combines them into one byte.
  //The resulting bye looks like this: (pin7 pin6 pin5 pin4 pin3 pin2 pin9 pin8)
  return (PIND & ~0x03) | (PINB & 0x03);
}

//Write byte to data bus
void writeDataBus(byte data) {
  digitalWrite(8, bitRead(data, 0));
  digitalWrite(9, bitRead(data, 1));
  for (int i = 2; i < 8; i++) {
    digitalWrite(i, bitRead(data, i));
  }
}

//Set the data bus to output or input
// false => INPUT, true => OUTPUT
void setDataBusDir(bool dir) {
  for (int p = 2; p <= 9; p++) {
    pinMode(p, dir);
  }
}




void dataPolling(byte dataByte) {

  setDataBusDir(false);  //configura pinos de dados como entrada

  delayMicroseconds(1);
  byte b1 = 0, b2 = 0;

  for (unsigned readCount = 1; (readCount < 1000); readCount++) {
    digitalWrite(pin_oe, LOW);  //habilita leitura do barramento de dados
    delayMicroseconds(1);
    //realiza a leitura dos dados
    for (int i = 7; i >= 0; i--) {
      int pin = pinSequence[i];
      b1 = (b1 << 1) + digitalRead(pin);
    }
    digitalWrite(pin_oe, HIGH);  //desabilita leitura do barramento de dados
    digitalWrite(pin_ce, HIGH);  //desativa a eeprom
    digitalWrite(pin_ce, LOW);   //ativa a eeprom
    digitalWrite(pin_oe, LOW);   //habilita leitura do barramento de dados
    delayMicroseconds(1);
    //realiza a leitura dos dados
    for (int i = 7; i >= 0; i--) {
      int pin = pinSequence[i];
      b2 = (b2 << 1) + digitalRead(pin);
    }
    digitalWrite(pin_oe, HIGH);  //desabilita leitura do barramento de dados
    digitalWrite(pin_ce, HIGH);  //desativa a eeprom
    if ((b1 == b2) && (b1 == dataByte)) {
      return true;
    }
  }
}

void writeBlock(byte bank, unsigned int addr, byte dataArray[sector_size]) {

  setDataBusDir(true);  //configura pinos de dados como saída

  digitalWrite(pin_oe, HIGH);  //desativa entrada no barramento de dados
  digitalWrite(pin_we, HIGH);  //desativa escrita
  digitalWrite(pin_ce, LOW);   //ativa a eeprom

  for (int ix = 0; ix < sector_size; ix++) {

    writeAddrBus(bank, addr + ix);

    //Latch dos dados
    for (int i = 0; i < 8; i++) {
      int pin = pinSequence[i];
      digitalWrite(pin, dataArray[ix] & 1);
      dataArray[ix] = dataArray[ix] >> 1;
    }

    delayMicroseconds(1);
    digitalWrite(pin_we, LOW);
    delayMicroseconds(1);
    digitalWrite(pin_we, HIGH);
  }

  dataPolling(dataArray[sector_size - 1]);  //verifica fim do ciclo de escrita em memórias flash
}
