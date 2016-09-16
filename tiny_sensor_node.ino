#include "Manchester.h"
#include <tiny_sensor.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/power.h>

#define SENSOR_ID 4
#define CAP_TEMP CAP_TEMP_DHT22
#define CAP_SW CAP_SW_REED
#define CAP_LDR CAP_LDR_PRESENT

#define WDT_PERIOD WDTO_1S
#define WDT_INTERVAL 3

#define TX_BITRATE MAN_1200

// Information from DS18B20 datasheet
// Mode     Res.      Conversion time
// 9 bits   0.5°C     93.75 ms
// 10 bits  0.25°C    187.5 ms
// 11 bits  0.125°C   375 ms
// 12 bits  0.0625°C  750 ms
#define DS18B20_RESOLUTION 11


#if defined(__AVR_ATtiny24__) || \
  defined(__AVR_ATtiny44__) || \
  defined(__AVR_ATtiny84__) || \
  defined(__AVR_ATtiny25__) || \
  defined(__AVR_ATtiny45__) || \
  defined(__AVR_ATtiny85__)

  // ATtiny specific
  #define ATTINY
  
  #define SW_PIN 0
  #define TX_PIN 1
  #define TEMP_PIN 2
  #define POWER_PIN 3
  #define LDR_PIN 4
#else

  // Other AVR CPU's
  #define SW_PIN 3
  #define TX_PIN 2
  #define TEMP_PIN 13
  #define LDR_PIN A0
  #define POWER_PIN 12

  // This registers have a different name than ATTiny
  #define WDTCR WDTCSR
#endif


struct packet p_buf = {
  .version = TINY_SENSOR_VERSION,
  .sensor_id = SENSOR_ID,
  .seq = 0,
  .cap = {
     .temp = CAP_TEMP,
     .sw = CAP_SW,
     .ldr = CAP_LDR,
     .reserved = 0
  },
  .state = { 0, },
  .crc = 0
};

#if CAP_TEMP == CAP_TEMP_DS18B20
  #include <OneWire.h>
  #include <DallasTemperature.h>
  OneWire dsWire(TEMP_PIN);
  DallasTemperature ds(&dsWire);
#elif CAP_TEMP == CAP_TEMP_DHT11
  #include <DHT.h>
  DHT dht(TEMP_PIN, DHT11);
#elif CAP_TEMP == CAP_TEMP_DHT22
  #include <DHT.h>
  DHT dht(TEMP_PIN, DHT22);
#endif

volatile bool switchIntr = false;
volatile bool watchdogIntr = false;
volatile int watchdogCounter = 0;

void setup() {
  // Configure the SW pin
  #if CAP_SW != CAP_SW_NONE
    #if CAP_SW == CAP_SW_REED
      pinMode(SW_PIN, INPUT);
    #else
      pinMode(SW_PIN, INPUT);
    #endif
  #endif

  pinMode(POWER_PIN, OUTPUT);
  digitalWrite(POWER_PIN, LOW);

  #if CAP_TEMP == CAP_TEMP_DS18B20
    ds.begin();
    ds.setResolution(DS18B20_RESOLUTION);
  #elif CAP_TEMP == CAP_TEMP_DHT11 || CAP_TEMP == CAP_TEMP_DHT22
    dht.begin();
  #endif

  #if CAP_LDR == CAP_LDR_PRESENT
    pinMode(LDR_PIN, INPUT);
  #endif

  man.workAround1MhzTinyCore(); // this seems okay for 8Mhz too.
  man.setupTransmit(TX_PIN, TX_BITRATE);

  setupWatchdog(WDT_PERIOD);
}

void loop() {
  // Who has interrupted us: switch or watchdog
  bool fromSwitch = switchIntr;
  if (fromSwitch) {
    switchIntr = false;
  } else {
    watchdogCounter++;
  }

  // When interrupted by the switch or watchdog
  if (watchdogCounter >= WDT_INTERVAL || fromSwitch) {
    if (!fromSwitch) {
      watchdogCounter = 0;
    }
    sendPacket(fromSwitch);
  }
  
  sleep();
}

void sendPacket(bool fromSwitch) {
  p_buf.seq++;
  
  #if CAP_SW != CAP_SW_NONE
    p_buf.state.sw.val = digitalRead(SW_PIN);
    p_buf.state.sw.intr = fromSwitch ? 1 : 0;
  #endif

  if (fromSwitch) {
    p_buf.state.vcc = 0;
  } else {
    p_buf.state.vcc = readVcc();
  }

  if (fromSwitch) {
    p_buf.state.temp = 0;
    p_buf.state.humid = 0;
    p_buf.state.lux = 0;
  } else {
    // Power up
    digitalWrite(POWER_PIN, HIGH);
    
    #if CAP_TEMP == CAP_TEMP_DS18B20
      ds.requestTemperatures();
      float temp = ds.getTempCByIndex(0);
      if (temp == DEVICE_DISCONNECTED_C) {
        p_buf.state.temp = TEMP_ERROR;
      } else {
        p_buf.state.temp = temp * 100;
      }
    #elif CAP_TEMP == CAP_TEMP_DHT11 || CAP_TEMP == CAP_TEMP_DHT22
      delay(100); // wait for DHT to initialize
      float temp = dht.readTemperature();
      float humid = dht.readHumidity();
      p_buf.state.temp = isnan(temp) ? TEMP_ERROR : temp * 100;
      p_buf.state.humid = isnan(humid) ? HUMID_ERROR : humid * 100;
    #endif
    
    #if CAP_LDR == CAP_LDR_PRESENT
      p_buf.state.lux = analogRead(LDR_PIN);    
    #endif

    // Power down
    digitalWrite(POWER_PIN, LOW);
  }

  // Calculate CRC of packet (excluding the last byte).
  p_buf.crc = crc8((uint8_t *)&p_buf, sizeof(packet) - 1);

  for (int i = 0; i < 3; i++) {
    man.transmitArray(sizeof(packet), (uint8_t *)&p_buf);
    delay(100);
  }
}

#ifdef ATTINY
  ISR(PCINT0_vect) {
    //sleep_disable();     // wake up here
    //GIMSK &= ~_BV(INT0); // disable INT0
    sleep_disable();
    switchIntr = true;
  }
#else
ISR(switchChanged) {
  sleep_disable(); 
  detachInterrupt(digitalPinToInterrupt(SW_PIN));
  switchIntr = true;
}
#endif

ISR(WDT_vect) {
  watchdogIntr = true;
}

// http://digistump.com/wiki/digispark/quickref
uint16_t readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif
 
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring
 
  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both
 
  long result = (high<<8) | low;
 
  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}

void sleep() {
  #ifdef ATTINY
    byte adcsra, mcucr1, mcucr2;

    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();

    GIMSK |= _BV(PCIE);                       // enable PC interrupts
    PCMSK |= _BV(PCINT0);                     // enable PCINT0
    adcsra = ADCSRA;                          // save ADCSRA
    ADCSRA &= ~_BV(ADEN);                     // disable ADC
    cli();                                    // stop interrupts to ensure the BOD timed sequence executes as required
    mcucr1 = MCUCR | _BV(BODS) | _BV(BODSE);  // turn off the brown-out detector
    mcucr2 = mcucr1 & ~_BV(BODSE);            // if the MCU does not have BOD disable capability,
    MCUCR = mcucr1;                           //   this code has no effect
    MCUCR = mcucr2;
    sei();                                    // ensure interrupts enabled so we can wake up again
    
    sleep_cpu();                              // go to sleep
    cli();
    PCMSK &= ~_BV(PCINT0);                    // disable PCINT0
    sleep_disable();                          // wake up here
    ADCSRA = adcsra;                          // restore ADCSRA
    sei();
  #else
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable(); // prepare
    power_adc_disable();
    sleep_bod_disable();
    attachInterrupt(digitalPinToInterrupt(SW_PIN), switchChanged, CHANGE);
    sleep_cpu(); // actually sleep
    sleep_disable(); // when we wake up
    detachInterrupt(digitalPinToInterrupt(SW_PIN));
    power_adc_enable();
  #endif
}

// http://interface.khm.de/index.php/lab/interfaces-advanced/sleep_watchdog_battery/
// 0=16ms, 1=32ms, 2=64ms, 3=128ms, 4=250ms, 5=500ms,
// 6=1 sec,7=2 sec, 8=4 sec, 9=8 sec
void setupWatchdog(int ii) {
  byte bb;
  int ww;
  
  if (ii > 9) ii = 9;
  bb = ii & 7;
  if (ii > 7) bb |= (1<<5);
  bb |= _BV(WDCE);
  ww = bb;

  MCUSR &= ~_BV(WDRF); // clear the reset flag
  //cli();  
  /* In order to change WDE or the prescaler, we need to
   * set WDCE (This will allow updates for 4 clock cycles).
   */
  WDTCR |= _BV(WDCE) | _BV(WDE);

  /* set new watchdog timeout prescaler value */
  WDTCR = bb;
  
  /* Enable the WD interrupt (note no reset). */
  WDTCR |= _BV(WDIE);
  //sei();
}

