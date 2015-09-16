#include <arduino.h>
#include <usitwislave.h>
#include <usitwislave_devices.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

// pin 7 == PB2 == i2c SCL == AIN1
// pin 5 == PB0 == i2c SDA == AREF
// pin 2 == PB3 == AIN3 == crystal, pin 3 == AIN2 == PB4 == crystal
// pin 4 = GND pin 8 = VCC
// pin 1 == RST
// pin 6 == PB1 == PWM == PCINT1
// pin 8 == vcc

typedef void (*cbk_t)(uint8_t, const uint8_t *, uint8_t *, uint8_t *);

volatile uint8_t test_counter = 0;

volatile uint32_t pulse_counter = 0;

int get_temp() {
  analogReference(INTERNAL1V1);
  int raw = analogRead(A0+15); 
  /* Original code used a 13 deg adjustment. But based on my results, I didn't seem to need it. */
  // raw -= 13; // raw adjust = kelvin //this value is used to calibrate to your chip
  int in_c = raw - 273; // celcius
  analogReference(DEFAULT);
  return in_c;
}

void blink() {
  digitalWrite(1, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(200);              // wait for a second
  digitalWrite(1, LOW);    // turn the LED off by making the voltage LOW
}

volatile bool next_led = false;

volatile int got_msg = 0;

volatile int cmd = -1;
volatile int32_t idle_wrap = 100000; // 4 seconds

static void twi_callback(
    volatile uint8_t input_buffer_length, volatile const uint8_t *input_buffer,
    volatile uint8_t *output_buffer_length, volatile uint8_t *output_buffer)
{
  output_buffer[0] = 0xc0;
  output_buffer[1] = 0x01;
  *output_buffer_length = 2;

  got_msg ++;
  cmd = input_buffer[0];
  idle_wrap = idle_wrap / 2;
  if (idle_wrap < 2000) { idle_wrap = 100000; }
#if 0
  cli();
  next_led = !next_led;
  bool led_on = next_led;
  sei();
  digitalWrite(1, led_on?HIGH:LOW);
#endif

#if 0
  // input_buffer_length == bytes from master
  // output_buffer_length == result
  if (input_buffer_length < 1) { return; }
  
  const uint8_t cmd = input_buffer[0];
  uint8_t len = 0;

  if (cmd == 1) {
    // echo
    if (input_buffer_length > 1) {
      output_buffer[0] = input_buffer[1];
      len = 1;
    }
  } else
  if (input_buffer[0] == 2) {
    // counter
    output_buffer[0] = ++ test_counter;
    len = 1;
  } else
  if (input_buffer[0] == 3) {
    // pulse counter
    cli();
    const uint32_t value = pulse_counter;
    pulse_counter = 0;
    sei();
    // -mutex}
    output_buffer[0] = value & 0xff;
    output_buffer[1] = (value >> 8) & 0xff;
    output_buffer[2] = (value >> 16) & 0xff;
    len = 3;
  } else
  if (input_buffer[0] == 4) {
    output_buffer[0] = get_temp();
    len = 1;
  } else
  {}
  *output_buffer_length = len;
#endif
}

ISR (PCINT1_vect)
{
  // increment the counter
  pulse_counter ++;
  next_led = !next_led;
  digitalWrite(1, next_led ?  HIGH : LOW);
}

volatile uint32_t idle_counter = 0;

volatile int flash_down = -1;

void idler()
{
#if 0
  if (cmd > 0) {
    flash_down = cmd;
    cmd = -1;
    idle_counter = 0;
  }
  if (flash_down > 0) {
    bool toggle = idle_counter % 20000 == 0;
    if (toggle) {
      next_led = !next_led;
      digitalWrite(1, next_led?HIGH:LOW);
      flash_down --;
      if (flash_down == 0) { idle_counter = 0; }
    }
    return;  
  }
  
  if (got_msg % 2 == 1) {
    digitalWrite(1, LOW);
    return;
  }
#endif
  
  idle_counter ++;
  bool toggle = idle_counter % idle_wrap == 0;
  if (toggle) {
    next_led = !next_led;
    digitalWrite(1, next_led?HIGH:LOW);
  }
}

void setup() {
  pinMode(1, OUTPUT);
  blink(); delay(200); blink();
  delay(1000);
  blink(); delay(200); blink();

#if 0
  // setup interrupt on PB1 (phys pin 6) going low
#ifdef INPUT_PULLUP
  pinMode(2, INPUT_PULLUP);
#else
  pinMode(2, INPUT);
  digitalWrite (2, LOW); // Digispark has no INPUT_PULLUP flag, we have to do it this way instead
#endif
  cli();
  GIMSK = (1 << PCIE);
  PCMSK = (1 << PCINT1);
  sei();
#endif

  // put your main code here, to run repeatedly:
 // usi_twi_slave(0x19, 1, (cbk_t)&twi_callback, NULL);
 usi_twi_slave(0x19, 0, (cbk_t)&twi_callback, &idler);
}

void loop() {
  blink(); delay(3000);
}
