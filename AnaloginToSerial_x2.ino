
// These constants won't change. They're used to give names to the pins used:
const int analog1InPin = A0;  // Analog input pin that the potentiometer is attached to
const int analog2InPin = A1;  // Analog input pin that the potentiometer is attached to

int sensor1Value = 0;  // value read from the pot
int sensor2Value = 0;  // value output to the PWM (analog out)

void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);
}

void loop() {
  // read the analog in value:
  sensor1Value = analogRead(analog1InPin);
  sensor2Value = analogRead(analog2InPin);

  // print the results to the Serial Monitor:
  Serial.print("sensor1 = ");
  Serial.print(sensor1Value);
  Serial.print("\t sensor2 = ");
  Serial.println(sensor2Value);

  // wait 250 milliseconds before the next loop
  delay(250);
}
