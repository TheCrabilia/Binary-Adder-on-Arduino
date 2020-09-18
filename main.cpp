// Defines LEDs pins
const int led1 = 11;
const int led2 = 10;
const int led4 = 9;
const int led8 = 8;

// Defines first number digits pins
const int n0d0 = 7;
const int n0d1 = 6;
const int n0d2 = 5;

// Defines second number digits pins
const int n1d0 = 4;
const int n1d1 = 3;
const int n1d2 = 2;

void setup()
{
  Serial.begin(9600); // Turn on serial port
  
  // Sets switch pins to OUTPUT mode
  pinMode(n0d0, OUTPUT);
  pinMode(n0d1, OUTPUT);
  pinMode(n0d2, OUTPUT);
  
  pinMode(n1d0, OUTPUT);
  pinMode(n1d1, OUTPUT);
  pinMode(n1d2, OUTPUT);
  
  // Sets LEDs pins to INPUT mode
  pinMode(led1, INPUT);
  pinMode(led2, INPUT);
  pinMode(led4, INPUT);
  pinMode(led8, INPUT);
}

void loop()
{
  // Reads values from switch pins
  int n0d0value = digitalRead(n0d0);
  int n0d1value = digitalRead(n0d1);
  int n0d2value = digitalRead(n0d2);
  
  int n1d0value = digitalRead(n1d0);
  int n1d1value = digitalRead(n1d1);
  int n1d2value = digitalRead(n1d2);
  
  // Disables all LEDs
  digitalWrite(led1, LOW);
  digitalWrite(led2, LOW);
  digitalWrite(led4, LOW);
  digitalWrite(led8, LOW);
  
  // Converts binary numbers to decimal and adds them
  int n0ValueDec = (n0d2value*4) + (n0d1value*2) + (n0d0value*1);
  int n1ValueDec = (n1d2value*4) + (n1d1value*2) + (n1d0value*1);
  int total = n0ValueDec + n1ValueDec;
  
  // Converts decimal sum to binary
  if (total >= 8) {
  	digitalWrite(led8, HIGH);
    total = total - 8;
  }
  if (total >= 4) {
  	digitalWrite(led4, HIGH);
    total = total - 4;
  }
  if (total >= 2) {
  	digitalWrite(led2, HIGH);
    total = total - 2;
  }
  if (total >= 1) {
  	digitalWrite(led1, HIGH);
    total = total - 1;
  }
  
  /*
  // Debug
  Serial.print("n0=");
  Serial.print(n0d0value);
  Serial.print(n0d1value);
  Serial.print(n0d2value);
  Serial.print("n1=");
  Serial.print(n1d0value);
  Serial.print(n1d1value);
  Serial.print(n1d2value);
  Serial.print("Total=");
  Serial.print(total);
  */
}
