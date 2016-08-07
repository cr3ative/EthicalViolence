#include <ZumoBuzzer.h>
#include <ZumoMotors.h>
#include <Pushbutton.h>
#include <QTRSensors.h>
#include <ZumoReflectanceSensorArray.h>
#include <avr/pgmspace.h>
#include <Wire.h>
#include <LSM303.h>

/* 
   EthicalViolence is a hacked-around version of this:
   https://github.com/pololu/zumo-shield/blob/master/ZumoExamples/examples/SumoCollisionDetect/SumoCollisionDetect.ino 
   
   Please visit that URL for the original README.
   
   I have attempted to condense the code-base here considerably, and provide a little more commenting.
   This project lives here: https://github.com/cr3ative/EthicalViolence
   Where you can find out more about the strategy and what it does.
   
   A note on variables: These were tweaked on the day to match the motor speeds and sunlight during that time.
   They are variables so please vary them. ;)
   
   Paul - twitter.com/cr3 / github.com/cr3ative
*/

/* Set up Zumo libraries */
#define LED 13
#define NUM_SENSORS 6
Pushbutton button(ZUMO_BUTTON);
ZumoReflectanceSensorArray sensors(QTR_NO_EMITTER_PIN); 
ZumoMotors motors;
unsigned int sensor_values[NUM_SENSORS];
unsigned long loop_start_time;

// Accelerometer Settings
#define RA_SIZE 4  // number of readings to include in running average of accelerometer readings
#define XY_ACCELERATION_THRESHOLD 2400  // for detection of contact (~16000 = magnitude of acceleration due to gravity)

// Reflectance Sensor Settings
// this might need to be tuned for different lighting conditions, surfaces, etc.
#define QTR_THRESHOLD  250 // microseconds

// Motor Settings
// these might need to be tuned for different motor types

#define REVERSE_SPEED     200 // 0 is stopped, 400 is full speed
#define TURN_SPEED        200
#define FULL_SPEED        400
#define REVERSE_DURATION  100 // ms
#define TURN_DURATION     300 // ms
#define FULLTURN_DURATION 500 // ms

#define RIGHT 1
#define LEFT -1

// Sound Effects
ZumoBuzzer buzzer;
const char sound_effect[] PROGMEM = "O4 T100 V15 L4 MS g12>c12>e12>G6>E12 ML>G2"; // "charge" melody

// RunningAverage class 
// based on RunningAverage library for Arduino
// source:  http://playground.arduino.cc/Main/RunningAverage
template <typename T> 
class RunningAverage
{
  public:
    RunningAverage(void);
    RunningAverage(int);
    ~RunningAverage();
    void clear();
    void addValue(T);
    T getAverage() const;
    void fillValue(T, int);
  protected:
    int _size;
    int _cnt;
    int _idx;
    T _sum;
    T * _ar;
    static T zero;
};

// Accelerometer Class -- extends the LSM303 Library to support reading and averaging the x-y acceleration 
//   vectors from the onboard LSM303DLHC accelerometer/magnetometer
class Accelerometer : public LSM303
{
  typedef struct acc_data_xy
  {
    unsigned long timestamp;
    int x;
    int y;
    float dir;
  } acc_data_xy;
  
  public: 
    Accelerometer() : ra_x(RA_SIZE), ra_y(RA_SIZE) {};
    ~Accelerometer() {};
    void enable(void);
    void getLogHeader(void);
    void readAcceleration(unsigned long timestamp);
    float len_xy() const;
    float dir_xy() const;
    int x_avg(void) const;
    int y_avg(void) const;
    long ss_xy_avg(void) const;
    float dir_xy_avg(void) const;
  private:
    acc_data_xy last;
    RunningAverage<int> ra_x;
    RunningAverage<int> ra_y;   
};

Accelerometer lsm303;
boolean in_contact;  // set when accelerometer detects contact with opposing robot
boolean last_turn_dir; // set when we find a boundary

// forward declaration

void setup()
{  
  
  // Initiate the Wire library and join the I2C bus as a master
  Wire.begin();
  
  // Initiate LSM303
  lsm303.init();
  lsm303.enable();

  // A4 seems not to be earthed. A0 is earthed only in run-mode, will not provide random!
  randomSeed(analogRead(4));
  
  pinMode(LED, HIGH);
  buzzer.playMode(PLAY_AUTOMATIC);
  waitForButtonAndCountDown(false);
  
}

void waitForButtonAndCountDown(bool restarting)
{ 
  
  digitalWrite(LED, HIGH);
  button.waitForButton();
  digitalWrite(LED, LOW);
   
  // play audible countdown
  for (int i = 0; i < 3; i++)
  {
    delay(1000);
    buzzer.playNote(NOTE_G(3), 50, 12);
  }
  delay(1000);
  buzzer.playFromProgramSpace(sound_effect);
  delay(1000);
  
  // Avoid initial shove
  avoid_shove();
   
}

void loop()
{
  
  if (button.isPressed())
  {
    // if button is pressed, stop and wait for another press to go again
    motors.setSpeeds(0, 0);
    button.waitForRelease();
    waitForButtonAndCountDown(true);
  }
  
  loop_start_time = millis();
  lsm303.readAcceleration(loop_start_time); 
  sensors.read(sensor_values);
  
  if (sensor_values[0] < QTR_THRESHOLD)
  {
    // if leftmost sensor detects line, reverse and turn to the right
    turn(RIGHT);
    last_turn_dir = false;
  }
  else if (sensor_values[0] < QTR_THRESHOLD && sensor_values[5] < QTR_THRESHOLD) {
    // if both sensors detect the line, turn 180 degs and march on
    fullturn();
  }
  else if (sensor_values[5] < QTR_THRESHOLD)
  {
    // if rightmost sensor detects line, reverse and turn to the left
    turn(LEFT);
    last_turn_dir = true;
  }
  else  // Normal - we're not on an edge
  {
  
    // Roam and destroy
    int speed = FULL_SPEED;
    
    // Random speed between 50 and FULL_SPEED for the non-dominant track
    // This will cause the Zumo to go, roughly, in a circle, once it finds the edge of the arena.
    long ran = random(50,FULL_SPEED);
    
    if (last_turn_dir == false) {
      motors.setSpeeds(speed, ran);
    } else {
      motors.setSpeeds(ran, speed);
    }

  }
}

void fullturn()
{

  // reverse first, we are head on
  motors.setSpeeds(-REVERSE_SPEED, -REVERSE_SPEED);
  delay(REVERSE_DURATION);

  // Attempt full turn. This will want tweaking.
  motors.setSpeeds(TURN_SPEED * LEFT, -TURN_SPEED * LEFT);
  delay(FULLTURN_DURATION);
  
  int speed = FULL_SPEED;
  motors.setSpeeds(speed, speed);
  
}

void avoid_shove()
{

  // Don't always go the same way; that can be programmed against
  // Choose a random direction

  // These do not respect FULL_SPEED settings etc; tweak them on the day

  long ran = random(0,9);
  
  if (ran < 5) {

    motors.setSpeeds(-100, -400);
    delay(350);
  
    motors.setSpeeds(-200, -400);
    delay(150);
  
  } else {
    
    motors.setSpeeds(-400, -100);
    delay(350);
  
    motors.setSpeeds(-400, -200);
    delay(150);
  
  }

}

// execute turn 
// direction:  RIGHT or LEFT
void turn(char direction)
{
  
  motors.setSpeeds(TURN_SPEED * direction, -TURN_SPEED * direction);
  delay(TURN_DURATION);
  
  int speed = FULL_SPEED;
  motors.setSpeeds(speed, speed);

  // Random delay after turning away from the sensor

  long ran = random(100,500);
  delay(ran);

}
  
// class Accelerometer -- member function definitions

// enable accelerometer only
// to enable both accelerometer and magnetometer, call enableDefault() instead
void Accelerometer::enable(void)
{
  // Enable Accelerometer
  // 0x27 = 0b00100111
  // Normal power mode, all axes enabled
  writeAccReg(LSM303::CTRL_REG1_A, 0x27);

  if (getDeviceType() == LSM303::device_DLHC)
  writeAccReg(LSM303::CTRL_REG4_A, 0x08); // DLHC: enable high resolution mode
}

void Accelerometer::getLogHeader(void)
{
  Serial.print("millis    x      y     len     dir  | len_avg  dir_avg  |  avg_len");
  Serial.println();
}

void Accelerometer::readAcceleration(unsigned long timestamp)
{
  readAcc();
  if (a.x == last.x && a.y == last.y) return;
  
  last.timestamp = timestamp;
  last.x = a.x;
  last.y = a.y;
  
  ra_x.addValue(last.x);
  ra_y.addValue(last.y);
 
}

float Accelerometer::len_xy() const
{
  return sqrt(last.x*a.x + last.y*a.y);
}

float Accelerometer::dir_xy() const
{
  return atan2(last.x, last.y) * 180.0 / M_PI;
}

int Accelerometer::x_avg(void) const
{
  return ra_x.getAverage();
}

int Accelerometer::y_avg(void) const
{
  return ra_y.getAverage();
}

long Accelerometer::ss_xy_avg(void) const
{
  long x_avg_long = static_cast<long>(x_avg());
  long y_avg_long = static_cast<long>(y_avg()); 
  return x_avg_long*x_avg_long + y_avg_long*y_avg_long;
}

float Accelerometer::dir_xy_avg(void) const
{
  return atan2(static_cast<float>(x_avg()), static_cast<float>(y_avg())) * 180.0 / M_PI;
}



// RunningAverage class 
// based on RunningAverage library for Arduino
// source:  http://playground.arduino.cc/Main/RunningAverage
// author:  Rob.Tillart@gmail.com
// Released to the public domain

template <typename T>
T RunningAverage<T>::zero = static_cast<T>(0);

template <typename T>
RunningAverage<T>::RunningAverage(int n)
{
  _size = n;
  _ar = (T*) malloc(_size * sizeof(T));
  clear();
}

template <typename T>
RunningAverage<T>::~RunningAverage()
{
  free(_ar);
}

// resets all counters
template <typename T>
void RunningAverage<T>::clear() 
{ 
  _cnt = 0;
  _idx = 0;
  _sum = zero;
  for (int i = 0; i< _size; i++) _ar[i] = zero;  // needed to keep addValue simple
}

// adds a new value to the data-set
template <typename T>
void RunningAverage<T>::addValue(T f)
{
  _sum -= _ar[_idx];
  _ar[_idx] = f;
  _sum += _ar[_idx];
  _idx++;
  if (_idx == _size) _idx = 0;  // faster than %
  if (_cnt < _size) _cnt++;
}

// returns the average of the data-set added so far
template <typename T>
T RunningAverage<T>::getAverage() const
{
  if (_cnt == 0) return zero; // NaN ?  math.h
  return _sum / _cnt;
}

// fill the average with a value
// the param number determines how often value is added (weight)
// number should preferably be between 1 and size
template <typename T>
void RunningAverage<T>::fillValue(T value, int number)
{
  clear();
  for (int i = 0; i < number; i++) 
  {
    addValue(value);
  }
}
