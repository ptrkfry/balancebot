// Run a A4998 Stepstick from an Arduino UNO.
// Paul Hurley Aug 2015

int x,dirc;
int direc=8;
int steps=9;
int enabl=10;
int enable_other_motor=6;

void setup()
{
  Serial.begin(9600);
  dirc=1;
  pinMode(enabl,OUTPUT); // Enable
  pinMode(steps,OUTPUT); // Step
  pinMode(direc,OUTPUT); // Dir
  digitalWrite(enabl,LOW); // Set Enable low

  pinMode(enable_other_motor,OUTPUT);
  digitalWrite(enable_other_motor,HIGH); // disable other motor
}

void loop()
{
  digitalWrite(enabl,LOW); // Set Enable low
  if(dirc==1)
  {
    digitalWrite(direc,HIGH); // Set Dir high --> GUZS
  }
  else
  {
    digitalWrite(direc,LOW); // Set Dir LOW --> UZS
  }
  Serial.println("Loop 200 steps (1 rev)");
  
  for(x = 0; x < 5*3200; x++) // Loop 200 times
  {
    stepMotor();
  }
  dirc=dirc*-1;
  Serial.println("Pause");
    digitalWrite(enabl,HIGH); 
  delay(2000); // pause one second
}

void stepMotor()
{
    digitalWrite(steps,HIGH); // Output high
    delayMicroseconds(20); // Wait
    digitalWrite(steps,LOW); // Output low
    delayMicroseconds(20); // Wait
}

