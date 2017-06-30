// Run a A4998 Stepstick from an Arduino UNO.
// Paul Hurley Aug 2015

int x,dirc;

void setup()
{
  Serial.begin(9600);
  dirc=1;
  pinMode(6,OUTPUT); // Enable
  pinMode(5,OUTPUT); // Step
  pinMode(4,OUTPUT); // Dir
  digitalWrite(6,LOW); // Set Enable low
}

void loop()
{
  digitalWrite(6,LOW); // Set Enable low
  if(dirc==1)
  {
    digitalWrite(4,HIGH); // Set Dir high --> GUZS
  }
  else
  {
    digitalWrite(4,LOW); // Set Dir LOW --> UZS
  }
  Serial.println("Loop 200 steps (1 rev)");
  
  for(x = 0; x < 5*3200; x++) // Loop 200 times
  {
    digitalWrite(5,HIGH); // Output high
    delayMicroseconds(20); // Wait
    digitalWrite(5,LOW); // Output low
    delayMicroseconds(20); // Wait
  }
  dirc=dirc*-1;
  Serial.println("Pause");
    digitalWrite(6,HIGH); // Set Enable low
  delay(5000); // pause one second
}
