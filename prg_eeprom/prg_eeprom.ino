/**
 * This sketch programs the microcode EEPROMs for the 8-bit breadboard computer
 * See this video for more: https://youtu.be/JUVt_KYAp-I
 */
#define SHIFT_DATA 2
#define SHIFT_CLK 3
#define SHIFT_LATCH 4
#define EEPROM_D0 5
#define EEPROM_D7 12
#define WRITE_EN 13

#define NOP 0b0101111110100110  //No oeration-0

#define HLT 0b1101111110100110  // Halt clock-1
#define MI  0b0001111110100110  // Memory address register in-2
#define RI  0b0010000000000000  // RAM data in-3
#define RO  0b0100111110100110  // RAM data out-4
#define IO  0b0101011110100110  // Instruction register out-5
#define II  0b0101101110100110  // Instruction register in-6
#define AI  0b0101110110100110  // A register in-7
#define AO  0b0101111010100110  // A register out-8
#define EO  0b0101111100100110  // ALU out-9
#define SU  0b0000000001000000  // ALU subtract-10
#define BI  0b0101111110000110  // B register in-11
#define OI  0b0000000000010000  // Output register in-12
#define CE  0b0000000000001000  // Program counter enable-13
#define CO  0b0101111110100010  // Program counter out-14
#define J   0b0101111110100100  // Jump (program counter in)-15


uint16_t data[] = {
  MI&CO,  (RO&II)|CE,  NOP,      NOP,      NOP,         NOP, NOP, NOP,   // 0000 - NOP
  MI&CO,  (RO&II)|CE,  IO&MI,    RO&AI,    NOP,         NOP, NOP, NOP,   // 0001 - LDA
  MI&CO,  (RO&II)|CE,  IO&MI,    RO&BI,    EO&AI,       NOP, NOP, NOP,   // 0010 - ADD
  MI&CO,  (RO&II)|CE,  IO&MI,    RO&BI,    (EO&AI)|SU,  NOP, NOP, NOP,   // 0011 - SUB
  MI&CO,  (RO&II)|CE,  IO&MI,    AO|RI,    NOP,         NOP, NOP, NOP,   // 0100 - STA
  MI&CO,  (RO&II)|CE,  IO&AI,    NOP,      NOP,         NOP, NOP, NOP,   // 0101 - LDI
  MI&CO,  (RO&II)|CE,  IO&J,     NOP,      NOP,         NOP, NOP, NOP,   // 0110 - JMP
  MI&CO,  (RO&II)|CE,  NOP,      NOP,      NOP,         NOP, NOP, NOP,   // 0111
  MI&CO,  (RO&II)|CE,  NOP,      NOP,      NOP,         NOP, NOP, NOP,   // 1000
  MI&CO,  (RO&II)|CE,  NOP,      NOP,      NOP,         NOP, NOP, NOP,   // 1001
  MI&CO,  (RO&II)|CE,  NOP,      NOP,      NOP,         NOP, NOP, NOP,   // 1010
  MI&CO,  (RO&II)|CE,  NOP,      NOP,      NOP,         NOP, NOP, NOP,   // 1011
  MI&CO,  (RO&II)|CE,  NOP,      NOP,      NOP,         NOP, NOP, NOP,   // 1100
  MI&CO,  (RO&II)|CE,  NOP,      NOP,      NOP,         NOP, NOP, NOP,   // 1101
  MI&CO,  (RO&II)|CE,  AO|OI,    NOP,      NOP,         NOP, NOP, NOP,   // 1110 - OUT
  MI&CO,  (RO&II)|CE,  HLT,      NOP,      NOP,         NOP, NOP, NOP,   // 1111 - HLT
};


/*
 * Output the address bits and outputEnable signal using shift registers.
 */
void setAddress(int address, bool outputEnable) {
  shiftOut(SHIFT_DATA, SHIFT_CLK, MSBFIRST, (address >> 8) | (outputEnable ? 0x00 : 0x80));
  shiftOut(SHIFT_DATA, SHIFT_CLK, MSBFIRST, address);

  digitalWrite(SHIFT_LATCH, LOW);
  digitalWrite(SHIFT_LATCH, HIGH);
  digitalWrite(SHIFT_LATCH, LOW);
}


/*
 * Read a byte from the EEPROM at the specified address.
 */
byte readEEPROM(int address) {
  for (int pin = EEPROM_D0; pin <= EEPROM_D7; pin += 1) {
    pinMode(pin, INPUT);
  }
  setAddress(address, /*outputEnable*/ true);

  byte data = 0;
  for (int pin = EEPROM_D7; pin >= EEPROM_D0; pin -= 1) {
    data = (data << 1) + digitalRead(pin);
  }
  return data;
}


/*
 * Write a byte to the EEPROM at the specified address.
 */
void writeEEPROM(int address, byte data) {
  setAddress(address, /*outputEnable*/ false);
  for (int pin = EEPROM_D0; pin <= EEPROM_D7; pin += 1) {
    pinMode(pin, OUTPUT);
  }

  for (int pin = EEPROM_D0; pin <= EEPROM_D7; pin += 1) {
    digitalWrite(pin, data & 1);
    data = data >> 1;
  }
  digitalWrite(WRITE_EN, LOW);
  delayMicroseconds(2);
  digitalWrite(WRITE_EN, HIGH);
  delay(15);
}


/*
 * Read the contents of the EEPROM and print them to the serial monitor.
 */
void printContents() {
  for (int base = 0; base <= 255; base += 16) {
    byte data[16];
    for (int offset = 0; offset <= 15; offset += 1) {
      data[offset] = readEEPROM(base + offset);
    }

    char buf[80];
    sprintf(buf, "%03x:  %02x %02x %02x %02x %02x %02x %02x %02x   %02x %02x %02x %02x %02x %02x %02x %02x",
            base, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7],
            data[8], data[9], data[10], data[11], data[12], data[13], data[14], data[15]);

    Serial.println(buf);
  }
}


void setup() {
  // put your setup code here, to run once:
  pinMode(SHIFT_DATA, OUTPUT);
  pinMode(SHIFT_CLK, OUTPUT);
  pinMode(SHIFT_LATCH, OUTPUT);
  digitalWrite(WRITE_EN, HIGH);
  pinMode(WRITE_EN, OUTPUT);
  Serial.begin(57600);

  // Program data bytes
  Serial.print("Programming EEPROM");

  // Program the 8 high-order bits of microcode into the first 128 bytes of EEPROM
  for (int address = 0; address < sizeof(data)/sizeof(data[0]); address += 1) {
    writeEEPROM(address, data[address] >> 8);

    if (address % 64 == 0) {
      Serial.print(".");
    }
  }

  /*for (int address = 0; address < sizeof(data)/sizeof(data[0]); address += 1) {
    writeEEPROM(address, data[address]);

    if (address % 64 == 0) {
      Serial.print(".");
    }
  }*/

  Serial.println(" done");


  // Read and print out the contents of the EERPROM
  Serial.println("Reading EEPROM");
  printContents();
}


void loop() {
  // put your main code here, to run repeatedly:

}

