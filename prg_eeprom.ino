#define DATA 2
#define CLK 3
#define LATCH 4
int i = 0;
int count = 0x01;

void setup() {
  pinMode(DATA, OUTPUT);
  pinMode(CLK, OUTPUT);
  pinMode(LATCH, OUTPUT);
  

}

void loop() {
  if(i == 8)
    i = 0;
  digitalWrite(LATCH,LOW);
  shiftOut(DATA,CLK,MSBFIRST,count<<i);
  digitalWrite(LATCH,HIGH);
  delay(500);
  i++;
  
  // put your main code here, to run repeatedly:

}
