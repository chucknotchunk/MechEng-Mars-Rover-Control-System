const byte interruptPin = 0;
const byte outputPin = 1;
volatile bool interruptFlag = false;

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(9600);

  // Configure the pins
  pinMode(interruptPin, INPUT);
  pinMode(outputPin, OUTPUT);

  // Attach interrupt to the pin
  attachInterrupt(digitalPinToInterrupt(interruptPin), handleInterrupt, RISING);
}

void loop() {
  // Toggle the output pin state
  digitalWrite(outputPin, HIGH);
  delay(500);
  digitalWrite(outputPin, LOW);
  delay(500);

  // Check if the interrupt flag is set
  if (interruptFlag) {
    // Print to Serial for debugging
    Serial.print("1");
    Serial.println();

    // Clear the interrupt flag
    interruptFlag = false;
  }
}

void handleInterrupt() {
  // Set the flag to indicate the interrupt occurred
  interruptFlag = true;
}
