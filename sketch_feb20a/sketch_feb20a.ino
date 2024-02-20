// Define pin assignments
const int pirPin = 2;         // PIR sensor pin
const int buttonPin = 3;      // Button pin
int led1 = 13;                // LED 1 pin
int led2 = 12;                // LED 2 pin
int led3 = 4;                 // LED 3 pin

// Define variables and constants
volatile uint8_t pirState, buttonState;    // Store PIR and button states as volatile
volatile uint8_t prepir, prebutton = 0;    // Store previous states as volatile
const uint16_t timer = 0;         // Initial timer value
const uint16_t compare = 31250;   // Timer comparison value
bool led3State = false;           // State variable for LED 3
bool led2State = false;           // State variable for LED 2
unsigned long buttonPressTime = 0; // Variable to store button press time
bool led2Toggle = false;          // Flag to track LED 2 toggle state

// Setup function - runs once at startup
void setup() {
  Serial.begin(9600);        // Initialize serial communication
  pinMode(pirPin, INPUT);    // Set PIR pin as input
  pinMode(buttonPin, INPUT_PULLUP);  // Set button pin with internal pull-up resistor
  pinMode(led1, OUTPUT);     // Set LED 1 pin as output
  pinMode(led2, OUTPUT);     // Set LED 2 pin as output
  pinMode(led3, OUTPUT);     // Set LED 3 pin as output
  
  // Enable Pin Change Interrupts for pins 2 and 3
  PCICR |= (1 << PCIE2);
  PCMSK2 |= (1 << PCINT18) | (1 << PCINT19);
  
  // Configure Timer1 settings
  TCCR1A = 0;                // Reset Timer1 Control Registers
  TCCR1B |= (1 << CS12);     // Set Timer1 prescaler to 256
  TCCR1B &= ~(1 << CS11);    // (prescaler is 256)
  TCCR1B &= ~(1 << CS10);    // (prescaler is 256)
  TCNT1 = timer;             // Set Timer1 count value
  OCR1A = compare;           // Set Timer1 comparison value
  TIMSK1 = (1 << OCIE1A);    // Enable Timer1 Output Compare A Match interrupt
  sei();                     // Enable global interrupts
}

// Loop function - runs repeatedly after setup
void loop() {
  // Check for changes in PIR sensor state
  if (prepir != pirState) {
    digitalWrite(led1, pirState);  // Set LED 1 based on PIR sensor
    Serial.println("PIR motion detected"); // Print PIR detection message
    prepir = pirState;  // Update previous PIR state
  }
  
  // Check if LED 2 should be turned off
  if (led2Toggle && millis() - buttonPressTime >= 100) {
    digitalWrite(led2, LOW);
    led2State = false;
    led2Toggle = false;
  }
}

// Pin Change Interrupt Service Routine
ISR(PCINT2_vect) {
  // Update PIR and button states based on pin readings
  pirState = PIND & (1 << PD2);    // Read PIR pin state
  buttonState = PIND & (1 << PD3); // Read button pin state
  
  // If button is pressed, turn on LED 2 and record press time
  if (buttonState == LOW) {
    digitalWrite(led2, HIGH);
    led2State = true;
    buttonPressTime = millis();
    led2Toggle = true;
  }
}

// Timer1 Compare Match A Interrupt Service Routine
ISR(TIMER1_COMPA_vect) {
  TCNT1 = timer;  // Reset Timer1 count value
  
  // Toggle LED 3 state every 100 ms
  led3State = !led3State;
  digitalWrite(led3, led3State);
}
