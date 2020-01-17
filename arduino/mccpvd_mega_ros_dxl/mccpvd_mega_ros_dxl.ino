/* control a single servo by usec on arduino uno
 * implementing writemicrosend()
 * for arduino mega2560. should be compatible with uno
 * if no low level code and functions are used.
 * pub ROS topics: sensors_raw;
 * 
 */
//#include <avr/sleep.h>
#include <ros.h>
#include <maccepavd/SensorsRawAdc.h>
#include <maccepavd/CommandRaw.h>
#include <std_msgs/UInt16.h>
#include <ros/time.h>
#include <Wire.h>
#include <Adafruit_INA219.h>

Adafruit_INA219 ina219;

//#include <Servo.h>
//#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
//#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

int pin_d1 = 5;
int pin_d2 = 6;
int pin_jointsensor = 0;

//int pin_current_servo1 = 3;
//int pin_current_servo2 = 4;

//int pin_current_damping = 8;
//int pin_current_charge = 9;
float Vcc;
unsigned int jntpos_adc;
float jntpos_mavg;
float old_jntpos_mavg;
float jntspeed = 0;
float rege_i = 0;
float rege_v = 0;
//float joint_read;
//float servo1_read;
//float servo2_read;
//int sensor_counter =0;

//const int numReadings = 4;
//int jnt_reads[numReadings];

ros::NodeHandle nh;
maccepavd::SensorsRawAdc sensors_msg;

union{
  struct{
    unsigned int D1;
    unsigned int D2;
  };
  unsigned int values[3];
}command_buffer;

void command_cb(const maccepavd::CommandRaw& cmd_msg){
  command_buffer.D1 = cmd_msg.D1;
  command_buffer.D2 = cmd_msg.D2;
}

ros::Subscriber<maccepavd::CommandRaw> sub("command_raw",command_cb);
ros::Publisher sensors_raw("sensors_raw",&sensors_msg);


void setup() {
  //Serial.begin(57600);
  //setupADC();
  nh.getHardware()->setBaud(115200);
  pinMode(pin_d1,OUTPUT);
  pinMode(pin_d2,OUTPUT);
  
  ina219.begin();
  //Serial.println(Vcc);
  //joint_read = float(analogRead(pin_jointsensor));
  command_buffer.D1 = 0;
  command_buffer.D2 = 0;
  //setup_timers();
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
  jntpos_adc = analogRead(pin_jointsensor);
  old_jntpos_mavg = jntpos_mavg;
  jntpos_mavg = old_jntpos_mavg*0.75 + jntpos_adc*0.25;
  jntspeed = jntpos_mavg - old_jntpos_mavg;
  sensors_msg.joint_sensor = jntpos_adc;
  //sensors_msg.servo1_sensor = analogRead(pin_servo1sensor);
  //sensors_msg.servo2_sensor = analogRead(pin_servo2sensor)*Vcc/1023.0;
  //sensors_msg.motor_current = (analogRead(pin_current_damping)*Vcc/1023.0-Vcc/2)/0.185;
  //sensors_msg.charge_current = (analogRead(pin_current_charge)*Vcc/1023.0-Vcc/2)/0.185;
  //sensors_msg.servo1_current = (analogRead(pin_current_servo1)*Vcc/1023.0-Vcc/2)/0.1;
  //sensors_msg.servo2_current = (analogRead(pin_current_servo2)*Vcc/1023.0-Vcc/2)/0.1;
  rege_i = ina219.getCurrent_mA();
  //rege_v = ina219.getBusVoltage_V();
  sensors_msg.rege_current = rege_i/1000;
  sensors_raw.publish(&sensors_msg);
  //sendmsg();
  //servo1_read = 0;
  //servo2_read = 0;
  //joint_read = 0;
  //TCNT2 = 0;
  //sensor_counter = 0;
  //}
  if (jntspeed >= 0)
  {
    analogWrite(pin_d1,command_buffer.D1);
    analogWrite(pin_d2,command_buffer.D2);
  }
  else
  {
    analogWrite(pin_d1,command_buffer.D2);
    analogWrite(pin_d2,command_buffer.D1);
  }
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
