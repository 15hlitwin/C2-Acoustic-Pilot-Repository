const int micPin = A0;  // Microphone connected to analog pin A0

const int baseline = 337;

int amplitude;
const int max_amplitude = 150;

void setup() {
  Serial.begin(9600);  // Start serial communication at 9600 baud rate
}

void loop() {
  int micValue = analogRead(micPin);  // Read the microphone value (0-1023)
  amplitude = abs(micValue-baseline);
  Serial.println(amplitude);           // Print the value to the Serial Monitor
  delay(50);                          // Small delay to make output readable
}