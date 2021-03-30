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


//Serial Pointer
HardwareSerial *SerialCom;

int pos = 0;

// --------------------- OUR CODE -------------------------

//setting up of the controller
float kpHomeStraight = 2.8;//3.420;
float kiHomeStraight = 0.06;//0.010;
float kpHomeStrafe = 4.2;
float kiHomeStrafe = 0.1;

float kpDriveY = 2;
float kiDriveY = 0.05;
float kpDriveStraight = 20;

float kpRotate = 4.5;
float kiRotate = 0.05;

float over = 20; //

//this is the range the error has to be in to be able to exit
float toleranceParallel = 4;
float toleranceX = 2.5;
float toleranceY = 10;
float toleranceAngle = 3;
float toleranceRotate = 5;

int scenario = 1; //scenario decides the beginning case

//other variables used
float error = 0;
float integral = 0;
float power = 0;
float u = 0;
float angle = 90;

int count = 0;
int end = 0;
int rotations = 0;

//set up IR sensors, ultra sonic and gyro
//IR1 is Long range front left
//IR2 is Long range front right
//IR3 is Short range front
//IR4 is Short range rear

int index = 0;
int MASize = 10;

float irSensor1 = A1;
float ir1ADC[10];
float mair1;
float  fl = 0;

float irSensor2 = A2;
float ir2ADC[10];
float mair2;
float  fr = 0;

float irSensor3 = A3;
float ir3ADC[10];
float mair3;
float  lf = 0;

float irSensor4 = A4;
float ir4ADC[10];
float mair4;
float  lr = 0;





//------------- Gyro variables----------------------------

int gyroSensor = A5;
float T = 10;
byte serialRead = 0;
int gyroSignalADC = 0;
float gyroSupplyVoltage = 5;

float gyroZeroVoltage = 0; // the value of voltage when gyro is zero
float gyroSensitivity = 0.007; // gyro sensitivity unit is (mv/degree/second) get from datasheet
float rotationThreshold = 0.5; // because of gyro drifting, defining rotation angular velocity less than this value will be ignored
float gyroRate = 0; // read out value of sensor in voltage
float currentAngle = 0; // current angle calculated by angular velocity integral on

//-------UltraSonic Vars---------

float mm = 0;

//My globals
float Y = 0; //from ultrasonic
//float sensDist[] = {83.1, 103.64}; //distance between sensors X then Y


//end of our code ---------------------------------------------------

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
  //delay(1000);
  SerialCom->println("Setup....");

  for (int i = 0; i < 10; i++){
    measure();
  }
 
}


void loop(void) //main loop
{
  static STATE machine_state = INITIALISING;
  //Finite-state machine Code
  switch (machine_state) {
    case INITIALISING:
      machine_state = initialising();
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
  //delay(1000); //One second delay to see the serial string "INITIALISING...."
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

  //code begins -------------------------------------------------------------------------------------------

  switch (scenario) {
    case 1: //ALIGNING
      home();
      break;
    case 2: //OPERATING
      drive();
      break;
    case 3: //Rotating
      rotate();
      break;
    case 4: //stop everything       //remove if using pc comands
      stop();
  }

  //end of my code -------------------------------------------------------------------------------------------------


  return RUNNING;
}

//MY FUNCTIONS Boi

void home() { //alligns the robot at the beginning and zeros the gyro

  reset(); //resets vraible to 0

  do { //rotate

    measure(); //measures all the sensors
  
  if (lf > 250) {
    power = 150;
    error = 10; // Stop it exiting due to lack of 'error'
  } else {
    error = lf - lr;
    power = controller(error, kpHomeStraight, kiHomeStraight);
 
  }
    left_font_motor.writeMicroseconds(1500 - power); //kinematics would fix this?
    left_rear_motor.writeMicroseconds(1500 - power);
    right_rear_motor.writeMicroseconds(1500 - power);
    right_font_motor.writeMicroseconds(1500 - power);

    end = endCondition(error, end, toleranceParallel); //accounts for overshoot endCondition(error, end, tol);
  } while (end < 15); //overshoot protection

  gyroSet(); //set up
  reset();

  do { //strafe

    measure();

    error = 160 - lf;
    power = controller(error, kpHomeStrafe, kiHomeStrafe);

    left_font_motor.writeMicroseconds(1500 + power); //kinematics would fix this?
    left_rear_motor.writeMicroseconds(1500 - power);
    right_rear_motor.writeMicroseconds(1500 - power);
    right_font_motor.writeMicroseconds(1500 + power);

    end = endCondition(error, end, toleranceX); //accounts for overshoot

  } while (end < 15);

  scenario = 2;
}

void drive() {

  float xerror = 0;
  float dX = 0;
  int fix = 1;
  reset();

  do { //forwards

    measure();

    error = 150 - Y;
    power = controller(error, kpDriveY, kiDriveY);

    xerror = 150 - lf + lr - lf; //keeping it straight
    dX = kpDriveStraight * xerror * fix;
    dX = constrain(dX, -200, 200); //stops the glitch half way through where rotaion power cancelled out forward power

    if (abs(error) < 30) {    //stops the wackness at the end where it rotates randomly
      fix = 0;
    }

    left_font_motor.writeMicroseconds(1500 - power + dX); //kinematics would fix this?
    left_rear_motor.writeMicroseconds(1500 - power + dX);
    right_rear_motor.writeMicroseconds(1500 + power + dX);
    right_font_motor.writeMicroseconds(1500 + power + dX);

    end = endCondition(error, end, toleranceY); //accounts for overshoot

  } while (end < 15);

  scenario = 3;
}

void rotate() {

  reset();

  float addon = 0;
  if (rotations < 3) {

    do {

      readGyro();

      if (currentAngle > 330) {
        addon = (360 - currentAngle);
        currentAngle = 0;
      }
      else {
        addon = 0;
      }

      error = angle - (currentAngle + addon);
      power = controller(error, kpRotate, kiRotate);


      left_font_motor.writeMicroseconds(1500 + power); //kinematics would fix this?
      left_rear_motor.writeMicroseconds(1500 + power);
      right_rear_motor.writeMicroseconds(1500 + power);
      right_font_motor.writeMicroseconds(1500 + power);

    } while (error > 5); // get within 5 degrees

    reset(); //resets vraible to 0

    do { //rotate

      measure(); //measures all the sensors

      error = lf - lr;
      power = controller(error, kpHomeStraight, 0.1);

      left_font_motor.writeMicroseconds(1500 - power); //kinematics would fix this?
      left_rear_motor.writeMicroseconds(1500 - power);
      right_rear_motor.writeMicroseconds(1500 - power);
      right_font_motor.writeMicroseconds(1500 - power);

      end = endCondition(error, end, toleranceParallel); //accounts for overshoot endCondition(error, end, tol);

    } while (end < 15); //overshoot protection

  





    rotations++;
    scenario = 2;

  } else {
    scenario = 4;
  }
  angle += 80;

}


//Helper functions

float controller(float error, float kp, float ki) {

  if (abs(error) > over) {  //stops integral affecting power till small error
    integral = 0;
  }
  else if (abs(error) < 2){
    integral = 0;
  }
  integral = integral + error;

  u = kp * error + ki * integral;

  power = constrain(u, -300, 300); //motor saftey

  //delay(1); //appears to help?
  return power;
}

float endCondition(float error, int count, int tolerance) { //accounts for overshoot and ensures setpoint is reached
  //if the error is within the tollerance for a set amount of iterations the code will exit

  if (abs(error) <= tolerance) { //if its within the tolerance start counting
    count = count + 1;
  }
  else { //if it leaves the tolerance reset the counter
    count = 0;
  }
  return count;
}

void reset () {
  count = 0;
  error = 0;
  end = 0;
}


void measure () {

/*
  ir1ADC[index] = analogRead(irSensor1);
  mair1 = movingAverage(ir1ADC);
  fl = 0 - pow(mair1, 3) * 0.000004 + pow(mair1, 2) * 0.0056 - mair1 * 2.9377 + 708.67;

  ir2ADC[index] = analogRead(irSensor2 );
  mair2 = movingAverage(ir2ADC);
  fr = 0 - pow(mair2, 3) * 0.000005 + pow(mair2, 2) * 0.0072 - mair2 * 3.7209 + 831.08;

*/

  ir3ADC[index] = analogRead(irSensor3);
  //SerialCom->print(ir3ADC[index]);
  mair3 = movingAverage(ir3ADC);
  //SerialCom->print(' ');
  //SerialCom->println(mair3);
  //SerialCom->println(n);
  lf = 0 - pow(mair3, 3) * 0.00002456 + pow(mair3, 2) * 0.0211 - mair3 * 6.1377 + 745.7;


  ir4ADC[index] = analogRead(irSensor4);
  //SerialCom->print(ir4ADC[index]);
  mair4 = movingAverage(ir4ADC);
  //SerialCom->print(' ');
  //SerialCom->println(mair4);
  //SerialCom->println(n);
  lr = 0 - pow(mair4, 3) * 0.00001452 + pow(mair4, 2) * 0.0124 - mair4 * 3.7308 + 525.54;

  index++;
  if (index > 10-1) {
    index = 0;
  }

  HC_SR04_range(); //caluclating distance ultra
  Y = mm;
}

void readGyro() {
  gyroRate = (analogRead(gyroSensor) * gyroSupplyVoltage) / 1023;

  // find the voltage offset the value of voltage when gyro is zero (still)
  gyroRate -= (gyroZeroVoltage / 1023 * gyroSupplyVoltage);
  
  // read out voltage divided the gyro sensitivity to calculate the angular velocity
  float angularVelocity = gyroRate / gyroSensitivity; // from Data Sheet, gyroSensitivity is 0.007 V/dps

  // if the angular velocity is less than the threshold, ignore it

  if (angularVelocity >= rotationThreshold || angularVelocity <= -rotationThreshold) {
    // we are running a loop in T (of T/1000 second).
    float angleChange = angularVelocity / (1000 / T);
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
  //Serial.println(currentAngle);
  delay(T);
}

void gyroSet () {
  //Setup gyro
  float sum = 0;
  for (int i = 0; i < 100; i++) // read 100 values of voltage when gyro is at still, to calculate the zero-drift.
  {
    gyroSignalADC = analogRead(gyroSensor);
    sum += gyroSignalADC;
    delay(5);
  }
  gyroZeroVoltage = sum / 100; // average the sum as the zero drifting
}

float movingAverage(float irArray[10]) {
  float sum = 0;
  for (int i = 0; i < 10; i++) {
    sum += irArray[i];
  }
  float ma = sum / 10;
  return ma;
}

//Provide code -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//

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
  cm = pulse_width / 58.0 + 8.7; //10.65 is distance from centre of bot to edge of sensor
  mm = cm * 10;

  // Print out results
  if ( pulse_width > MAX_DIST ) {
    //SerialCom->println("HC-SR04: Out of range");
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
