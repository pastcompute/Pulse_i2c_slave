//#include <TinyPinChange.h>

#include <arduino.h>
#include <usitwislave.h>
#include <usitwislave_devices.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

// pin 7 == PB2 == i2c SCL == AIN1
// pin 5 == PB0 == i2c SDA == AREF
// pin 2 == PB3 == AIN3 == crystal, pin 3 == AIN2 == PB4 == crystal
// pin 4 = GND pin 8 = VCC
// pin 1 == PB5 == RST
// pin 6 == PB1 == PWM == PCINT1 == LED(!)
// pin 8 == vcc

typedef void (*cbk_t)(uint8_t, const uint8_t *, uint8_t *, uint8_t *);

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
volatile int i2c_rx = 0;
volatile uint8_t scratch = 0;
volatile uint16_t test_counter = 0;
volatile uint32_t pulse_counter = 0;
volatile uint32_t latch_pulse_counter = 0;

static void twi_callback(
    volatile uint8_t input_buffer_length, volatile const uint8_t *input_buffer,
    volatile uint8_t *output_buffer_length, volatile uint8_t *output_buffer)
{
  i2c_rx ++;
  if (input_buffer_length < 1) { return; }
  
  const uint8_t cmd = input_buffer[0];

  // To make this work properly with i2cget in openwrt , just work on bytes
  // Thus
  // Addr 1 == echo second byte - not sure how to make it work with i2cget/i2cset
  // Addr 2 == roll count, incremented for each valid i2c command (including itself)
  // Addr 3 == roll count MSB
  // Addr 4 == pulse count LSB, latches value for xSB and MSB
  // Addr 5 == pulse count xSB
  // Addr 6 == pulse count MSB
  // Addr 7 == chip temp degC
  // Addr 8 == read scratch
  // Addr 9 == write scratch

  if (cmd > 0 && cmd < 9) { test_counter ++; }

  // most of these seem to lag for some reason

  uint8_t len = 1;
  switch (cmd) {
  case 1: output_buffer[0] = input_buffer[1]; break;
  case 2: output_buffer[0] = test_counter & 0xff; break;
  case 3: output_buffer[0] = (test_counter >> 8) & 0xff; break;
  case 4:
    //noInterrupts();
    cli();
    latch_pulse_counter = pulse_counter;
    pulse_counter = 0;
    sei();
    //interrupts();
    output_buffer[0] = latch_pulse_counter & 0xff; break;
  case 5:
    output_buffer[0] = (latch_pulse_counter >> 8) & 0xff; break;
  case 6: 
    output_buffer[0] = (latch_pulse_counter >> 16) & 0xff; break;
  case 7: 
    output_buffer[0] = get_temp(); break;
  case 8: 
    output_buffer[0] = scratch; break;
  case 9:
    output_buffer[0] = scratch; scratch = input_buffer[1]; break;  // i2cset -y 0 0x19 0x09 0x42 b ; i2cget -y 0 0x19 0x08 b  -- the second call ignores the reg and returns the value
  case 10:
    output_buffer[0] = 0x5a;   // only first is gettable by i2cget. 
    output_buffer[1] = 0xde;
    output_buffer[2] = 0xad;
    output_buffer[3] = 0xbe;
    output_buffer[4] = 0xef;
    len = 5;
    break;
  case 11:
    output_buffer[0] = 0xc0;   // only first is gettable by i2cget
    output_buffer[1] = 0x0l;
    len = 2;
    break;
  }
  *output_buffer_length = len;
}

#if 0
void on_pulse()
{
  pulse_counter ++;
  digitalWrite (1, HIGH);
}
#endif

volatile bool next_ir = true;

#if 1
ISR (PCINT0_vect)
{
  // increment the counter
  if (digitalRead(4) == 1) {
    pulse_counter ++;
    digitalWrite (1, next_ir ? HIGH : LOW);
    next_ir = !next_ir;
  }
}
#endif

volatile uint32_t idle_counter = 0;

volatile int flash_down = -1;

volatile uint32_t prev_counter = 0;

void idler()
{
  if (i2c_rx > 0) {
    // flash a few times each receipt
    flash_down = 8;
    i2c_rx = 0;
    idle_counter = 0;
    next_led = true;
  } else if (flash_down <= 0) {
#if 0
    noInterrupts();
    //cli();
    uint32_t cache_counter = pulse_counter;
    //sei();
    interrupts();
    if (prev_counter != cache_counter) {
      prev_counter = cache_counter;
      flash_down = 6;
    }
#endif 
  }
  if (flash_down > 0) {
    // Note: reprogramming at 8MHz doubles the flash interval here
    bool toggle = idle_counter % 2500 == 0; // this gives a short flash, 50000 about 4 seconds, when NOT using sleep.
    idle_counter ++;
    if (toggle) {
      digitalWrite(1, next_led?HIGH:LOW);
      next_led = !next_led;
      flash_down --;
      if (flash_down == 0) {
        digitalWrite(1, LOW);
        flash_down = -1;      
      }
    }
  }
}

void setup() {
  pinMode(1, OUTPUT);
  blink(); delay(400); blink(); delay(400); blink();

  // setup interrupt on PB4 (phys pin 3) going high
  // PB4 has Zener diode to earth pulling it low otherwise
  pinMode(4, INPUT);

  if (digitalRead(4)) {
    blink(); delay(200); blink(); delay(200); blink();
  }

#if 1
#if 0
#ifdef INPUT_PULLUP
  pinMode(4, INPUT_PULLUP);
#else
  digitalWrite (4, HIGH); // Digispark has no INPUT_PULLUP flag, we have to do it this way instead
#endif
#endif
  cli();
  _SFR_BYTE(GIMSK) |= _BV(PCIE);
  _SFR_BYTE(PCMSK) |= _BV(PCINT4);
  sei();
#else
  attachInterrupt(4, on_pulse, CHANGE);
#endif

  // put your main code here, to run repeatedly:
  // usi_twi_slave(0x19, 1, (cbk_t)&twi_callback, NULL);
  usi_twi_slave(0x19, 0, (cbk_t)&twi_callback, &idler);
}

// Program with No-USB - 8MHz (or 1MHz later)

void loop() {
  // never called
  blink(); delay(3000);
}
