/* control a single servo by usec on arduino uno
 * implementing writemicrosend()
 * for arduino mega2560. should be compatible with uno
 * if no low level code and functions are used.
 * pub ROS topics: sensors_raw;
 * 
 */
#include <avr/sleep.h>
#include <ros.h>
#include <maccepavd/SensorsRaw.h>
#include <maccepavd/CommandRaw.h>
#include <std_msgs/UInt16.h>
#include <ros/time.h>
#include <Wire.h>
#include <Adafruit_INA219.h>

Adafruit_INA219 ina219;

//#include <Servo.h>
//#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
//#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

int pin_servo1 = 11;
int pin_servo2 = 12;
int pin_d1 = 5;
int pin_d2 = 6;
int pin_jointsensor = 0;
int pin_servo1sensor = 1;
int pin_servo2sensor = 2;

//int pin_current_servo1 = 3;
//int pin_current_servo2 = 4;

//int pin_current_damping = 8;
//int pin_current_charge = 9;
float Vcc;
unsigned int jntpos_adc;
float jntpos_mavg;
float jntspeed = 0;
//float joint_read;
//float servo1_read;
//float servo2_read;
//int sensor_counter =0;

//const int numReadings = 4;
//int jnt_reads[numReadings];

ros::NodeHandle nh;
maccepavd::SensorsRaw sensors_msg;

union{
  struct{
    unsigned int u1; // servo command of servo1
    unsigned int u2;
    unsigned int D1;
    unsigned int D2;
  };
  unsigned int values[3];
}command_buffer;

void command_cb(const maccepavd::CommandRaw& cmd_msg){
  command_buffer.u1 = cmd_msg.u1;
  command_buffer.u2 = cmd_msg.u2;
  command_buffer.D1 = cmd_msg.D1;
  command_buffer.D2 = cmd_msg.D2;

  if (jntspeed >= 0)
  {
    analogWrite(pin_d1,cmd_msg.D1);
    analogWrite(pin_d2,cmd_msg.D2);
  }
  else
  {
    analogWrite(pin_d1,cmd_msg.D2);
    analogWrite(pin_d2,cmd_msg.D1);
  }
  
}

ros::Subscriber<maccepavd::CommandRaw> sub("command_raw",command_cb);
ros::Publisher sensors_raw("sensors_raw",&sensors_msg);


void setup_timers(){
  uint8_t oldSREG = SREG;
  cli();
  // fast PWM mode (mode14), WGM13-10:1 1 1 0
  // Compare Output mode: non-inverting mode. COM1A1-0: 1 0
  TCCR1A = 1<<WGM11 | 1<<COM1A1 & ~_BV(COM1A0) | 1<<COM1B1 & ~_BV(COM1B0) ;
  TCCR1B = 1<<WGM13 | 1<<WGM12 | 1<<CS11;
  // setup ICR1 at which the clock resets
  // ATmega2560 clock speed: 16MHz, prescaler: 8
  // targeted PWM frequency: 50 Hz
  // ICR1 = F_cpu / (PreScaler*F_pwm) - 1
  ICR1 = 40000 - 1;
  
  OCR1A = command_buffer.u1;
  OCR1B = command_buffer.u2;
  
  // enable timer overflow interrupt
  
  TIMSK1 &=  ~_BV(OCIE1A);   // Disable Timer 1 Compare Match A (OCIE1A) interrupt (not used) 
  TIMSK1 &=  ~_BV(OCIE1B);   // Disable Timer 1 Compare Match B (OCIE1B) interrupt (not used) 
  TIMSK1 |= 1<<TOIE1;
  
  // setup timer2
  //
  //TCCR2A = 0; // WGM22:0 := normal mode
  //TCCR2B = ( 1 << CS22 ) | (1 << CS21 ) ; //prescaler 128: 101; 256: 110; 64:100
  //TCNT2 = 0;
  //
  //-------------------//
  //ASSR   &= ~_BV(AS2); //internal I/O clock
  
  //TCCR2A  = 0;               // Set Timer 2 to normal mode
  //TCCR2B |= _BV(CS02);      // Set prescaler for Timer 2. 
  //TCCR2B |= _BV(CS01);
  //-------------------//
  //TIMSK2 |=  _BV(TOIE2);     // Enable Timer 2 Overflow (TOIE2) interrupt
  //TIMSK2 &= ~_BV(OCIE2A);    // Disable Compare Match A (OCIE2A) interrupt (not used)
  
    
  
  SREG = oldSREG;
}

ISR(TIMER1_OVF_vect){
  OCR1A = command_buffer.u1*2;
  OCR1B = command_buffer.u2*2;
  //sensors_msg.servo1_sensor = servo1_read*Vcc/1023.0;
  
  
}

/*
ISR(TIMER2_OVF_vect){
  //joint_read = joint_read + analogRead(pin_jointsensor);
  //servo1_read = servo1_read + analogRead(pin_servo1sensor);
  //servo1_read += float(analogRead(pin_servo1sensor));
  //servo2_read = servo2_read + analogRead(pin_servo2sensor);
  //sensor_counter += 1;
}
*/

void setupADC(){
  ADCSRA &= ~_BV( ADPS2 );
  ADCSRA |= _BV(ADPS1);
  ADCSRA |= _BV(ADPS0);
  
}

void setup() {
  //Serial.begin(57600);
  //setupADC();
  pinMode(pin_servo1,OUTPUT);
  pinMode(pin_servo2,OUTPUT);
  pinMode(pin_d1,OUTPUT);
  pinMode(pin_d2,OUTPUT);
  Vcc = readVcc()/1000.0;
  ina219.begin();
  Serial.println(Vcc);
  //joint_read = float(analogRead(pin_jointsensor));
  //servo1_read= float(analogRead(pin_servo1sensor));
  //servo2_read= float(analogRead(pin_servo2sensor));
  command_buffer.u1 = 1500;
  command_buffer.u2 = 900;
  setup_timers();
  delay(10);

  //nh.getHardware()->setBaud(115200); change baudrate doesn't work
  nh.initNode();
  //nh.advertise(sensor1);
  nh.advertise(sensors_raw);
  nh.subscribe(sub);

  jntpos_adc = analogRead(pin_jointsensor);
  jntpos_mavg = jntpos_adc;
}

void loop() {
  //transmit_index = 0;
  //UCSR0B |= _BV(TXCIE0);
  //unsigned long t1 = micros();

  //joint_read = analogRead(pin_jointsensor);
  //servo1_read = analogRead(pin_servo1sensor);
  //servo1_read += float(analogRead(pin_servo1sensor));
  //servo2_read = analogRead(pin_servo2sensor);
  //sensor_counter += 1;
  //delay(1);
  //if(sensor_counter == 4){
  
  sensors_msg.header.stamp = nh.now();
  //sensors_msg.u1 = command_buffer.u1;
  //sensors_msg.u2 = command_buffer.u2;
  //sensors_msg.D1 = command_buffer.D1;
  //sensors_msg.D2 = command_buffer.D2;
  jntpos_adc = analogRead(pin_jointsensor);
  old_jntpos_mavg = jntpos_mavg;
  jntpos_mavg = old_jntpos_mavg*0.75 + jntpos_adc*0.25;
  jntspeed = jntpos_mavg - old_jntpos_mavg;
  sensors_msg.joint_sensor = jntpos_adc*Vcc/1023.0;
  //sensors_msg.servo1_sensor = analogRead(pin_servo1sensor)*Vcc/1023.0;
  //sensors_msg.servo2_sensor = analogRead(pin_servo2sensor)*Vcc/1023.0;
  //sensors_msg.motor_current = (analogRead(pin_current_damping)*Vcc/1023.0-Vcc/2)/0.185;
  //sensors_msg.charge_current = (analogRead(pin_current_charge)*Vcc/1023.0-Vcc/2)/0.185;
  //sensors_msg.servo1_current = (analogRead(pin_current_servo1)*Vcc/1023.0-Vcc/2)/0.1;
  //sensors_msg.servo2_current = (analogRead(pin_current_servo2)*Vcc/1023.0-Vcc/2)/0.1;
  sensors_msg.rege_current = ina219.getCurrent_mA();
  sensors_raw.publish(&sensors_msg);
  //sendmsg();
  //servo1_read = 0;
  //servo2_read = 0;
  //joint_read = 0;
  //TCNT2 = 0;
  //sensor_counter = 0;
  //}
  nh.spinOnce();
}

//void sendmsg(){
//  sensors_msg.header.stamp = nh.now() ;
//  sensors_msg.joint_sensor = joint_read*Vcc/1023.0;
//  sensors_msg.servo1_sensor = servo1_read*Vcc/1023.0;
//  sensors_msg.servo2_sensor = servo2_read*Vcc/1023.0;
//  
//  sensors_raw.publish(&sensors_msg);
//  
//}

long readVcc() {
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
  #if defined(__AVR_ATmega2560__)
    ADCSRB &= ~_BV(MUX5); // Without this the function always returns -1 on the ATmega2560
  #endif
  delay(20); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both

  long result = (high<<8) | low;

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}

/*int analogNoiseReducedRead(int pinNumber)
{
 int reading;
 
 ADCSRA |= _BV( ADIE );             //Set ADC interrupt
 set_sleep_mode(SLEEP_MODE_ADC);    //Set sleep mode
 reading = analogRead(pinNumber);   //Start reading
 sleep_enable();                    //Enable sleep
 do
 {                                  //Loop until reading is completed
   sei();                           //Enable interrupts
   sleep_mode();                    //Go to sleep
   cli();                           //Disable interrupts
 } while(((ADCSRA&(1<<ADSC))!= 0)); //Loop if the interrupt that woke the cpu was something other than the ADC finishing the reading
 sleep_disable();                   //Disable sleep
 ADCSRA &= ~ _BV( ADIE );           //Clear ADC interupt
 sei();                             //Enable interrupts
 
 return(reading);
}
*/
