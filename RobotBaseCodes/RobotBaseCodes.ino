/*
  MechEng 706 Base Code

  This code provides basic movement and sensor reading for the MechEng 706 Mecanum Wheel Robot Project

  Hardware:
    Arduino Mega2560 https://www.arduino.cc/en/Guide/ArduinoMega2560
    MPU-9250 https://www.sparkfun.com/products/13762
    Ultrasonic Sensor - HC-SR04 https://www.sparkfun.com/products/13959
    Infrared Proximity Sensor - Sharp https://www.sparkfun.com/products/242
    Infrared Proximity Sensor Short Range - Sharp https://www.sparkfun.com/products/12728
    Servo - Generic (Sub-Micro Size) https://www.sparkfun.com/products/9065
    Vex Motor Controller 29 https://www.vexrobotics.com/276-2193.html
    Vex Motors https://www.vexrobotics.com/motors.html
    Turnigy nano-tech 2200mah 2S https://hobbyking.com/en_us/turnigy-nano-tech-2200mah-2s-25-50c-lipo-pack.html

  Date: 11/11/2016
  Author: Logan Stuart
  Modified: 15/02/2018
  Author: Logan Stuart
*/
#include <Servo.h>  //Need for Servo pulse output

//#define NO_READ_GYRO  //Uncomment of GYRO is not attached.
//#define NO_HC-SR04 //Uncomment of HC-SR04 ultrasonic ranging sensor is not attached.
//#define NO_BATTERY_V_OK //Uncomment of BATTERY_V_OK if you do not care about battery damage.

//State machine states
enum STATE {
  INITIALISING,
  RUNNING,
  STOPPED
};

//Refer to Shield Pinouts.jpg for pin locations

//Default motor control pins
const byte left_front = 46;
const byte left_rear = 47;
const byte right_rear = 50;
const byte right_front = 51;


//Default ultrasonic ranging sensor pins, these pins are defined my the Shield
const int TRIG_PIN = 48;
const int ECHO_PIN = 49;

// Anything over 400 cm (23200 us pulse) is "out of range". Hit:If you decrease to this the ranging sensor but the timeout is short, you may not need to read up to 4meters.
const unsigned int MAX_DIST = 23200;

Servo left_font_motor;  // create servo object to control Vex Motor Controller 29
Servo left_rear_motor;  // create servo object to control Vex Motor Controller 29
Servo right_rear_motor;  // create servo object to control Vex Motor Controller 29
Servo right_font_motor;  // create servo object to control Vex Motor Controller 29
Servo turret_motor;


float speed_val = 0;
float speed_change;

// --------------------- OUR CODE -------------------------

float error = 0;
float integral = 0;
float power = 0;
float u = 0;
float angle = 90;

int count = 0;
int end = 0;



//set up IR sensors, ultra sonic and gyro
//IR1 is Long range front left
//IR2 is Long range front right
//IR3 is Short range front 
//IR4 is Short range rear

float irSensor1 = A1;
float ir1ADC;
float ir1val;

float irSensor2 = A2;
float ir2ADC;
float ir2val;

float irSensor3 = A3;
float ir3ADC;
float ir3val;

float irSensor4 = A4;
float ir4ADC;
float ir4val;

float  fl = 0; //setup modifers
float  fr = 0;
float  lf = 0;
float  lr = 0;


//------------- Gyro variables----------------------------

int gyroSensor = A5;
float T = 100;
byte serialRead = 0;
int gyroSignalADC = 0;
float gyroSupplyVoltage = 5;

float gyroZeroVoltage = 0; // the value of voltage when gyro is zero
float gyroSensitivity = 0.007; // gyro sensitivity unit is (mv/degree/second) get from datasheet
float rotationThreshold = 1.5; // because of gyro drifting, defining rotation angular velocity less than this value will be ignored
float gyroRate = 0; // read out value of sensor in voltage
float currentAngle = 0; // current angle calculated by angular velocity integral on

//-------UltraSonic Vars---------

float mm = 0;

//My globals
float Y = 0; //from ultrasonic
//float sensDist[] = {83.1, 103.64}; //distance between sensors X then Y



int scenario = 1; //either alligning or 
int rotations = 0;


//Setup gyro
  float sum = 0;

//end of our code



//Serial Pointer
HardwareSerial *SerialCom;

int pos = 0;
void setup(void)
{
  turret_motor.attach(11);
  pinMode(LED_BUILTIN, OUTPUT);

  // The Trigger pin will tell the sensor to range find
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);

  // Setup the Serial port and pointer, the pointer allows switching the debug info through the USB port(Serial) or Bluetooth port(Serial1) with ease.
  SerialCom = &Serial;
  SerialCom->begin(115200);
  SerialCom->println("MECHENG706_Base_Code_25/01/2018");
  delay(1000);
  SerialCom->println("Setup....");

  delay(1000); //settling time but no really needed

}

void loop(void) //main loop
{
  static STATE machine_state = INITIALISING;
  //Finite-state machine Code
  switch (machine_state) {
    case INITIALISING:
      machine_state = initialising();
      gyroSet ();
      break;
    case RUNNING: //Lipo Battery Volage OK
      machine_state =  running();             
      break;
    case STOPPED: //Stop of Lipo Battery voltage is too low, to protect Battery
      machine_state =  stopped();
      break;
  };
}


STATE initialising() {
  //initialising
  SerialCom->println("INITIALISING....");
  delay(1000); //One second delay to see the serial string "INITIALISING...."
  SerialCom->println("Enabling Motors...");
  enable_motors();
  SerialCom->println("RUNNING STATE...");
  return RUNNING;
}

STATE running() {
  
  fast_flash_double_LED_builtin(); //theirs

#ifndef NO_BATTERY_V_OK
    if (!is_battery_voltage_OK()) return STOPPED;
#endif



    //Code boi
    //Serial.print("scenario: ");
    //Serial.println(scenario);      

    

    
//   if (SerialCom->available()) {    //the code that allows us to start/stop from laptop
//    char val = SerialCom->read();
//
//    //Perform an action depending on the command
//    switch (val) {
//      case 'b'://Move Forward
//      case 'B':
//        forward ();
//        SerialCom->println("Forward");
          


//measure();
//readGyro();
//              Serial.print("angle : ");
//              Serial.println(currentAngle);
//              delay(1000);
          
      switch (scenario) {
       case 1: //ALIGNING      
          home(1.5, 0.1, 1.5, 0.1);  
              Serial.print("scenario: ");
              Serial.println(scenario);       
          break;          
        case 2: //OPERATING      
          drive(2, 0.2, 15);
             Serial.print("scenario: ");
              Serial.println(scenario); 
          break;
        case 3: //Rotating
          rotate(2, 0.1);  
             Serial.print("scenario: ");
              Serial.println(scenario); 
          break;
        case 4: //stop everything       //remove if using pc comands
          stop();
      }


//      break;        //the code that allows us to start/stop from laptop
//    default:
//      stop();
//      SerialCom->println("stop");
//      break;
//  }
      //end of my code



  return RUNNING;
}






//Stop of Lipo Battery voltage is too low, to protect Battery
STATE stopped() {
  static byte counter_lipo_voltage_ok;
  static unsigned long previous_millis;
  int Lipo_level_cal;
  disable_motors();
  slow_flash_LED_builtin();

  if (millis() - previous_millis > 500) { //print massage every 500ms
    previous_millis = millis();
    SerialCom->println("STOPPED---------");


#ifndef NO_BATTERY_V_OK
    //500ms timed if statement to check lipo and output speed settings
    if (is_battery_voltage_OK()) {
      SerialCom->print("Lipo OK waiting of voltage Counter 10 < ");
      SerialCom->println(counter_lipo_voltage_ok);
      counter_lipo_voltage_ok++;
      if (counter_lipo_voltage_ok > 10) { //Making sure lipo voltage is stable
        counter_lipo_voltage_ok = 0;
        enable_motors();
        SerialCom->println("Lipo OK returning to RUN STATE");
        return RUNNING;
      }
    } else
    {
      counter_lipo_voltage_ok = 0;
    }
#endif
  }
  return STOPPED;
}

void fast_flash_double_LED_builtin()
{
  static byte indexer = 0;
  static unsigned long fast_flash_millis;
  if (millis() > fast_flash_millis) {
    indexer++;
    if (indexer > 4) {
      fast_flash_millis = millis() + 700;
      digitalWrite(LED_BUILTIN, LOW);
      indexer = 0;
    } else {
      fast_flash_millis = millis() + 100;
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
  }
}

void slow_flash_LED_builtin()
{
  static unsigned long slow_flash_millis;
  if (millis() - slow_flash_millis > 2000) {
    slow_flash_millis = millis();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}

void speed_change_smooth()
{
  speed_val += speed_change;
  if (speed_val > 1000)
    speed_val = 1000;
  speed_change = 0;
}

#ifndef NO_BATTERY_V_OK
boolean is_battery_voltage_OK()
{
  static byte Low_voltage_counter;
  static unsigned long previous_millis;

  int Lipo_level_cal;
  int raw_lipo;
  //the voltage of a LiPo cell depends on its chemistry and varies from about 3.5V (discharged) = 717(3.5V Min) https://oscarliang.com/lipo-battery-guide/
  //to about 4.20-4.25V (fully charged) = 860(4.2V Max)
  //Lipo Cell voltage should never go below 3V, So 3.5V is a safety factor.
  raw_lipo = analogRead(A0);
  Lipo_level_cal = (raw_lipo - 717);
  Lipo_level_cal = Lipo_level_cal * 100;
  Lipo_level_cal = Lipo_level_cal / 143;

  if (Lipo_level_cal > 0 && Lipo_level_cal < 160) {
    previous_millis = millis();
    SerialCom->print("Lipo level:");
    SerialCom->print(Lipo_level_cal);
    SerialCom->print("%");
    // SerialCom->print(" : Raw Lipo:");
    // SerialCom->println(raw_lipo);
    SerialCom->println("");
    Low_voltage_counter = 0;
    return true;
  } else {
    if (Lipo_level_cal < 0)
      SerialCom->println("Lipo is Disconnected or Power Switch is turned OFF!!!");
    else if (Lipo_level_cal > 160)
      SerialCom->println("!Lipo is Overchanged!!!");
    else {
      SerialCom->println("Lipo voltage too LOW, any lower and the lipo with be damaged");
      SerialCom->print("Please Re-charge Lipo:");
      SerialCom->print(Lipo_level_cal);
      SerialCom->println("%");
    }

    Low_voltage_counter++;
    if (Low_voltage_counter > 5)
      return false;
    else
      return true;
  }

}
#endif

#ifndef NO_HC-SR04
void HC_SR04_range()
{
  unsigned long t1;
  unsigned long t2;
  unsigned long pulse_width;
  float cm;
  

  // Hold the trigger pin high for at least 10 us
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Wait for pulse on echo pin
  t1 = micros();
  while ( digitalRead(ECHO_PIN) == 0 ) {
    t2 = micros();
    pulse_width = t2 - t1;
    if ( pulse_width > (MAX_DIST + 1000)) {
      SerialCom->println("HC-SR04: NOT found");
      return;
    }
  }

  // Measure how long the echo pin was held high (pulse width)
  // Note: the micros() counter will overflow after ~70 min

  t1 = micros();
  while ( digitalRead(ECHO_PIN) == 1)
  {
    t2 = micros();
    pulse_width = t2 - t1;
    if ( pulse_width > (MAX_DIST + 1000) ) {
      SerialCom->println("HC-SR04: Out of range");
      return;
    }
  }

  t2 = micros();
  pulse_width = t2 - t1;

  // Calculate distance in centimeters and inches. The constants
  // are found in the datasheet, and calculated from the assumed speed
  //of sound in air at sea level (~340 m/s).
  cm = pulse_width / 58.0 + 10.65; //10.65 is distance from centre of bot to edge of sensor
  mm = cm * 10;

  // Print out results
  if ( pulse_width > MAX_DIST ) {
    SerialCom->println("HC-SR04: Out of range");
  } else {
    SerialCom->print("HC-SR04:");
    SerialCom->print(cm);
    SerialCom->println("cm");
  }
}
#endif

void Analog_Range_A4()
{
  SerialCom->print("Analog Range A4:");
  SerialCom->println(analogRead(A4));
}

#ifndef NO_READ_GYRO
void GYRO_reading()
{
  SerialCom->print("GYRO A3:");
  SerialCom->println(analogRead(A3));
}
#endif


//----------------------Motor moments------------------------
//The Vex Motor Controller 29 use Servo Control signals to determine speed and direction, with 0 degrees meaning neutral https://en.wikipedia.org/wiki/Servo_control

void disable_motors()
{
  left_font_motor.detach();  // detach the servo on pin left_front to turn Vex Motor Controller 29 Off
  left_rear_motor.detach();  // detach the servo on pin left_rear to turn Vex Motor Controller 29 Off
  right_rear_motor.detach();  // detach the servo on pin right_rear to turn Vex Motor Controller 29 Off
  right_font_motor.detach();  // detach the servo on pin right_front to turn Vex Motor Controller 29 Off

  pinMode(left_front, INPUT);
  pinMode(left_rear, INPUT);
  pinMode(right_rear, INPUT);
  pinMode(right_front, INPUT);
}

void enable_motors()
{
  left_font_motor.attach(left_front);  // attaches the servo on pin left_front to turn Vex Motor Controller 29 On
  left_rear_motor.attach(left_rear);  // attaches the servo on pin left_rear to turn Vex Motor Controller 29 On
  right_rear_motor.attach(right_rear);  // attaches the servo on pin right_rear to turn Vex Motor Controller 29 On
  right_font_motor.attach(right_front);  // attaches the servo on pin right_front to turn Vex Motor Controller 29 On
}
void stop() //Stop
{
  left_font_motor.writeMicroseconds(1500);
  left_rear_motor.writeMicroseconds(1500);
  right_rear_motor.writeMicroseconds(1500);
  right_font_motor.writeMicroseconds(1500);
}


//MY FUNCTIONS Boi

void home(float kpRotate, float kiRotate, float kpStrafe, float kiStrafe) {

count = 0;
error = 0;
end = 0;

 do { //rotate

   measure();

   error = lf - lr;
   power = controller(error, 20.0, kpRotate, kiRotate);

   Serial.print("power 1 : ");
              Serial.println(power); 

   left_font_motor.writeMicroseconds(1500 - power); //kinematics would fix this?
   left_rear_motor.writeMicroseconds(1500 - power);
   right_rear_motor.writeMicroseconds(1500 - power);
   right_font_motor.writeMicroseconds(1500 - power);

   end = endCondition(error, end, 6); //accounts for overshoot endCondition(error, end, tol);
  
 } while (end < 50);

gyroSet(); //set up

count = 0;
error = 0;
end = 0;

   do { //strafe
  
     measure();
  
     error = 150 - lf;
     power = controller(error, 20.0, kpStrafe, kiStrafe);
  
    left_font_motor.writeMicroseconds(1500 + power); //kinematics would fix this?
    left_rear_motor.writeMicroseconds(1500 - power);
    right_rear_motor.writeMicroseconds(1500 - power);
    right_font_motor.writeMicroseconds(1500 + power);
  
     end = endCondition(error, end, 6); //accounts for overshoot
    
   } while (end < 50);


//count = 0;
//error = 0;
//end = 0;
//
// do { //rotate
//
//   measure();
//
//   error = lf - lr;
//   power = controller(error, 100, kpRotate, kiRotate);
//
//   Serial.print("power 1 : ");
//              Serial.println(power); 
//
//   left_font_motor.writeMicroseconds(1500 - power); //kinematics would fix this?
//   left_rear_motor.writeMicroseconds(1500 - power);
//   right_rear_motor.writeMicroseconds(1500 - power);
//   right_font_motor.writeMicroseconds(1500 - power);
//
//   end = endCondition(error, end, 20); //accounts for overshoot endCondition(error, end, tol);
//  
// } while (end < 30);
 
   scenario = 2;
}


void drive(float kpY, float kiY, float kpX){

float xerror = 0;
float dX = 0;  
count = 0;
error = 0;
end = 0;

   do { //forwards

    Serial.print("scenario: ");
              Serial.println(scenario); 
  
     measure();
  
     error = 150 - fl;
     power = controller(error, 200.0, kpY, kiY);
     //power = kpY * error;

     xerror = 150 - lf; //keeping it straight
    dX = kpX * xerror;
    //dX = controller(xerror, 400, kpX, kiY);
  
    left_font_motor.writeMicroseconds(1500 - power + dX); //kinematics would fix this?
    left_rear_motor.writeMicroseconds(1500 - power + dX);
    right_rear_motor.writeMicroseconds(1500 + power + dX);
    right_font_motor.writeMicroseconds(1500 + power + dX);
  
     end = endCondition(error, end, 20); //accounts for overshoot
    
   } while (end < 15);

   scenario = 3;
}


void rotate(float kp, float ki){

count = 0;
error = 0;
end = 0;

 do { //rotate


  Serial.print("scenario: ");
              Serial.println(scenario); 

   //measure();
   readGyro(); //calculating rotation

              Serial.print("angle : ");
              Serial.println(currentAngle); 
    
   error = angle  - currentAngle;
   power = controller(error, 50, kp, ki);
      

   left_font_motor.writeMicroseconds(1500 - power); //kinematics would fix this?
   left_rear_motor.writeMicroseconds(1500 - power);
   right_rear_motor.writeMicroseconds(1500 - power);
   right_font_motor.writeMicroseconds(1500 - power);


   end = endCondition(error, end, 20); //accounts for overshoot endCondition(error, end, tol);
  
 } while (end < 15);

//gyroSet(); //set up

  if (rotations < 4){
  scenario = 2;
  rotations = rotations + 1;
  }
    else{
      scenario = 4;
    }

  angle += 90;
  
}



//Helper functions

float controller(float error, float ramp, float kp, float ki){

  if (error > 30){   //stops integral affecting power till small error
    integral = 0;
  }
  
  
  integral = integral + error;

  //integral = constrain(u, -200, 200);
  
  u = kp * error + ki * integral; 

  Serial.print("u : ");
              Serial.println(u); 
  
  power = constrain(u, -500, 500); //motor saftey

  if (count > ramp){ //ramp
    count = ramp;
  }
  
  power = (power * (1/ramp * count)); //increases power
  count = count + 1;

  delay(1);
  return power;
}


float endCondition(float error, int count, int tolerance) { //accounts for overshoot and ensures setpoint is reached

  if (abs(error) <= tolerance) {
    count = count + 1;
  }
  else {
    count = 0;
  }


  //delay(1);
  return count;
}


void measure () {

 ir1ADC = analogRead(irSensor1);
  //Serial.print("IR Sensor 1 ADC: ");
  //Serial.println(ir1ADC);
  ir1val = 0 - pow(ir1ADC, 3) * 0.000004 + pow(ir1ADC, 2) * 0.0056 - ir1ADC * 2.9377 + 708.67;
  //Serial.print("IR Sensor 1 distance: ");
  //Serial.println(ir1val);

  ir2ADC = analogRead(irSensor2 );
  //Serial.print("IR Sensor 2 ADC: ");
  //Serial.println(ir2ADC);
  ir2val = 0 - pow(ir2ADC, 3) * 0.000005 + pow(ir2ADC, 2) * 0.0072 - ir2ADC * 3.7209 + 831.08;
  //Serial.print("IR Sensor 2 distance: ");
  //Serial.println(ir2val);

  ir3ADC = analogRead(irSensor3);
  //Serial.print("IR Sensor 3 ADC: ");
  //Serial.println(ir3ADC);
  ir3val = 0 - pow(ir3ADC, 3) * 0.000004 + pow(ir3ADC, 2) * 0.0054 - ir3ADC * 2.4371 + 466.8;
  //Serial.print("IR Sensor 3 distance: ");
  //Serial.println(ir3val);

  ir4ADC = analogRead(irSensor4);
  //Serial.print("IR Sensor 4 ADC: ");
  //Serial.println(ir4ADC);
  ir4val = 0 - pow(ir4ADC, 3) * 0.000003 + pow(ir4ADC, 2) * 0.0043 - ir4ADC * 1.9775 + 404.3;
  //Serial.print("IR Sensor 4 distance: ");
  //Serial.println(ir4val);

  HC_SR04_range(); //caluclating distance ultra
  //readGyro(); //calculating rotation

  fl = ir1val; //setting sensors
  fr = ir2val;
  lf = ir3val;
  lr = ir4val;
  
  Y = mm;
}


void readGyro() { //tom
  gyroRate = (analogRead(gyroSensor) * gyroSupplyVoltage) / 1023;

  // find the voltage offset the value of voltage when gyro is zero (still)
  gyroRate -= (gyroZeroVoltage / 1023 * gyroSupplyVoltage);
  // read out voltage divided the gyro sensitivity to calculate the angular velocity
  float angularVelocity = gyroRate / gyroSensitivity; // from Data Sheet, gyroSensitivity is 0.007 V/dps

  // if the angular velocity is less than the threshold, ignore it

  if (angularVelocity >= rotationThreshold || angularVelocity <= -rotationThreshold) {
    // we are running a loop in T (of T/1000 second).
    //T = millis() - T;
    float angleChange = angularVelocity / (1000 / T);
    //T = millis();
    currentAngle += angleChange;
  }

  // keep the angle between 0-360


  if (currentAngle < 0)
  {
    currentAngle += 360;
  }
  else if (currentAngle > 359)
  {
    currentAngle -= 360;
  }

  //Serial.print(angularVelocity);
  //Serial.print(" ");
  Serial.println(currentAngle);
  delay(T);
}



//gyro zero

void gyroSet (){
  for (int i = 0; i < 100; i++) // read 100 values of voltage when gyro is at still, to calculate the zero-drift.
  {
    gyroSignalADC = analogRead(gyroSensor);
    sum += gyroSignalADC;
    delay(5);
  }
  gyroZeroVoltage = sum / 100; // average the sum as the zero drifting


  //delay(1000); //settling time but no really needed

}
