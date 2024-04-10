#define BUTTON 3
#define LED 13

int led_state = LOW;      
int button_state;          
int last_button_state = HIGH;  

unsigned long last_debounce_time = 0; 
unsigned long debounce_delay = 50;

void setup() {
  pinMode(BUTTON, INPUT_PULLUP);
  pinMode(LED, OUTPUT);
  digitalWrite(LED,led_state);
}

void loop() {
  int reading = digitalRead(BUTTON);

  if (reading != last_button_state) {
    last_debounce_time = millis();
  }

  if ((millis() - last_debounce_time) > debounce_delay) {
    if (reading != button_state) {
      button_state = reading;
      if (button_state == LOW) {
        led_state = !led_state;
      }
    }
  }
  digitalWrite(LED,led_state);
  last_button_state = reading;
}
