//steps per mm calculation
//----------------------------------------------------------------------------

// y motor
// motor steps per turn = 18 * 4 ( microstepping )
// gear ratio gear box = 1:100
// gear ratio iner gear = 22:34
// total steps per complete turn = 11127.27
// steps per deg = 30.9
// inner circumfrance = 94.2
// steps per mm 118.12
// steps per deg = 30.9090
// steps per minute( 6 degs ) = 185.45454545

// x motor
// motor steps per turn = 18 * 4 ( microstepping )
// gear ratio gear box = 1:100 ( to check ) // 286.419 actual
// belt teeth 116
// gear teeth 25
// ratio 1:4.64
// belt lenght - 314.3 //REALITY 908
// total steps = 33408
// steps per mm 106.3

// IN REALITY STEPS TO COMPLETE ONE CIRCLE 194MM COMMANDED
//that makes 194 x 106.3 = 20622.2 steps ( full circle )
// steps for full belt = 20622.2 x 4.64 = 95687

// so actual ratio is 20622.2/(18x4) = 286.419
// so actual steps per mm = total belt steps / belt length
// 95687/314.3 = 304.44

//------------------------------------------------------------------------------
// CONSTANTS
//------------------------------------------------------------------------------
#define VERBOSE              (1)  // add to get a lot more serial output.
#include <TimeLib.h>
#include <math.h>
#include <Wire.h>
#include <DS1307RTC.h>


#define VERSION              (2)  // firmware version
#define BAUD                 (57600)  // How fast is the Arduino talking?
#define MAX_BUF              (64)  // What is the longest message Arduino can store?
#define STEPS_PER_TURN       (100)  // depends on your stepper motor.  most are 200.
#define STEPS_PER_MM         (STEPS_PER_TURN*1/1.25)  // (400*16)/1.25 with a M8 spindle
#define STEPS_PER_MM_X         304.44  //106.3//(STEPS_PER_TURN*1/1.25)
#define STEPS_PER_MM_Y         118.12  //(STEPS_PER_TURN*1/1.25)
#define MAX_FEEDRATE         (2000000)//(1000000)INCREASEING THE VALUE SLOWS IT 
#define MIN_FEEDRATE         (1)
#define NUM_AXIES            (4)

// stops and sensors (ek)
# define color_read             A3
# define test_led               A6
# define upper_led                11//12
# define lower_led                6//actual 6//7
boolean x_homestate = false;
boolean x_homestate2 = false;
boolean y_homestate = false;
boolean y_homestate2 = false;
# define y_stop 10//23// 18
# define x_stop 9//12
# define x2_stop 12//11 fliped with led for pwm
# define y3_stop A2
# define y2_stop 12
# define y4_stop A1
# define y5_stop A0
# define park_button 13
//led fading

int brightness = 0;    // how bright the LED is
int fadeAmount = 5;    // how many points to fade the LED by
long previousMillis = 0;
long interval = 30;
int led = 11;

// time demo

long demo_previousMillis = 0;
long demo_interval = 500;
int execute_time = 0;
int time_count = 0;
int tell_time = 1;
int play_time_enable = 0;
const float x1_home_val = 78.53;
const float x2_home_val = 235.68;
int disp_m = minute();
int hourly_home = 2;
int park_state = 0;
//------------------------------------------------------------------------------
// STRUCTS
//------------------------------------------------------------------------------
// for line()
typedef struct {
  long delta;  // number of steps to move
  long absdelta;
  long over;  // for dx/dy bresenham calculations
}
Axis;


typedef struct {
  int step_pin;
  int dir_pin;
  int enable_pin;
  int limit_switch_pin;
}
Motor;


//------------------------------------------------------------------------------
// GLOBALS
//------------------------------------------------------------------------------
Axis a[NUM_AXIES];  // for line()
Axis atemp;  // for line()
Motor motors[NUM_AXIES];

char buffer[MAX_BUF];  // where we store the message until we get a ';'
int sofar;  // how much is in the buffer

// speeds
float fr = 0; // human version
long step_delay;  // machine version

float px, py, pz, pe; // position

// settings
char mode_abs = 1; // absolute mode? = 1 relative mode = 0

long line_number = 0;


//------------------------------------------------------------------------------
// METHODS
//------------------------------------------------------------------------------


/**
 * delay for the appropriate number of microseconds
 * @input ms how many milliseconds to wait
 */
void pause(long ms) {
  delay(ms / 1000);
  delayMicroseconds(ms % 1000); // delayMicroseconds doesn't work for values > ~16k.
}


/**
 * Set the feedrate (speed motors will move)
 * @input nfr the new speed in steps/second
 */
void feedrate(float nfr) {
  if (fr == nfr) return; // same as last time?  quit now.

  if (nfr > MAX_FEEDRATE || nfr < MIN_FEEDRATE) { // don't allow crazy feed rates
    Serial.print(F("New feedrate must be greater than "));
    Serial.print(MIN_FEEDRATE);
    Serial.print(F("steps/s and less than "));
    Serial.print(MAX_FEEDRATE);
    Serial.println(F("steps/s."));
    return;
  }
  step_delay = MAX_FEEDRATE / nfr;
  fr = nfr;
}


/**
 * Set the logical position
 * @input npx new position x
 * @input npy new position y
 */
void position(float npx, float npy, float npz, float npe) {
  // here is a good place to add sanity tests
  px = npx;
  py = npy;
  pz = npz;
  pe = npe;
}


/**
 * Supports movement with both styles of Motor Shield
 * @input newx the destination x position
 * @input newy the destination y position
 **/



/**
 * Uses bresenham's line algorithm to move both motors
 * @input newx the destination x position
 * @input newy the destination y position
 **/
void line(float newx, float newy, float newz, float newe) {
  a[0].delta = (newx - px) * STEPS_PER_MM_X;
  a[1].delta = (newy - py) * STEPS_PER_MM_Y;
  a[2].delta = (newz - pz) * STEPS_PER_MM;
  a[3].delta = (newe - pe) * STEPS_PER_MM;

  long i, j, maxsteps = 0;

  for (i = 0; i < NUM_AXIES; ++i) {
    a[i].absdelta = abs(a[i].delta);
    if ( maxsteps < a[i].absdelta ) maxsteps = a[i].absdelta;
    // set the direction once per movement
    digitalWrite(motors[i].dir_pin, a[i].delta > 0 ? HIGH : LOW);
  }
  for (i = 0; i < NUM_AXIES; ++i) {
    a[i].over = maxsteps / 2;
  }

  long dt = MAX_FEEDRATE / 5000;
  long accel = 1;
  long steps_to_accel = dt - step_delay;
  if (steps_to_accel > maxsteps / 2 )
    steps_to_accel = maxsteps / 2;

  long steps_to_decel = maxsteps - steps_to_accel;

  //Serial.print("START ");
  // Serial.println(dt);
  // Serial.print("TOP ");
  // Serial.println(step_delay);

  // Serial.print("accel until ");
  // Serial.println(steps_to_accel);
  // Serial.print("decel after ");
  // Serial.println(steps_to_decel);
  // Serial.print("total ");
  // Serial.println(maxsteps);
#ifdef VERBOSE
  Serial.println(F("Start >"));
#endif

  for ( i = 0; i < maxsteps; ++i ) {
    for (j = 0; j < NUM_AXIES; ++j) {
      a[j].over += a[j].absdelta;
      if (a[j].over >= maxsteps) {
        a[j].over -= maxsteps;

        digitalWrite(motors[j].step_pin, HIGH);
        digitalWrite(motors[j].step_pin, LOW);
      }
    }

    if (i < steps_to_accel) {
      dt -= accel;
    }
    if (i >= steps_to_decel) {
      dt += accel;
    }
    delayMicroseconds(dt);
  }

#ifdef VERBOSE
  Serial.println(F("< Done."));
#endif

  position(newx, newy, newz, newe);

  //where();
}


// returns angle of dy/dx as a value from 0...2PI
static float atan3(float dy, float dx) {
  float a = atan2(dy, dx);
  if (a < 0) a = (PI * 2.0) + a;
  return a;
}



/**
 * Look for character /code/ in the buffer and read the float that immediately follows it.
 * @return the value found.  If nothing is found, /val/ is returned.
 * @input code the character to look for.
 * @input val the return value if /code/ is not found.
 **/
float parsenumber(char code, float val) {
  char *ptr = buffer;
  while (ptr && *ptr && ptr < buffer + sofar) {
    if (*ptr == code) {
      return atof(ptr + 1);
    }
    ptr = strchr(ptr, ' ') + 1;
  }
  return val;
}


/**
 * write a string followed by a float to the serial line.  Convenient for debugging.
 * @input code the string.
 * @input val the float.
 */
void output(char *code, float val) {
  Serial.print(code);
  Serial.println(val);
}


/**
 * print the current position, feedrate, and absolute mode.
 */
void where() {
  output("X", px);
  output("Y", py);
  output("Z", pz);
  output("E", pe);
  output("F", fr);
  Serial.println(mode_abs ? "ABS" : "REL");
}


/**
 * display helpful information
 */
void help() {
  Serial.print(F("GcodeCNCDemo6AxisV2 "));
  Serial.println(VERSION);
  Serial.println(F("Commands:"));
  Serial.println(F("G00/G01 [X/Y/Z/E(steps)] [F(feedrate)]; - linear move"));
  Serial.println(F("G04 P[seconds]; - delay"));
  Serial.println(F("G90; - absolute mode"));
  Serial.println(F("G91; - relative mode"));
  Serial.println(F("G92 [X/Y/Z/E(steps)]; - change logical position"));
  Serial.println(F("M18; - disable motors"));
  Serial.println(F("M100; - this help message"));
  Serial.println(F("M114; - report position and feedrate"));
  Serial.println(F("All commands must end with a newline."));
}


/**
 * Read the input buffer and find any recognized commands.  One G or M command per line.
 */
void processCommand() {
  int cmd = parsenumber('G', -1);
  switch (cmd) {
  case  0:
  case  1:
    { // line
      feedrate(parsenumber('F', fr));
      line( parsenumber('X', (mode_abs ? px : 0)) + (mode_abs ? 0 : px),
      parsenumber('Y', (mode_abs ? py : 0)) + (mode_abs ? 0 : py),
      parsenumber('Z', (mode_abs ? pz : 0)) + (mode_abs ? 0 : pz),
      parsenumber('E', (mode_abs ? pe : 0)) + (mode_abs ? 0 : pe) );
      break;
    }
  case  2:
  case  4:
    pause(parsenumber('P', 0) * 1000);
    break; // dwell
  case 90:
    mode_abs = 1;
    break; // absolute mode
  case 91:
    mode_abs = 0;
    break; // relative mode
  case 92:  // set logical position
    position( parsenumber('X', 0),
    parsenumber('Y', 0),
    parsenumber('Z', 0),
    parsenumber('E', 0) );

    break;
  case 28:
    home_x2();
    position(x2_home_val, py, 0, 0);// x this is middle of fulcrum on the bottom
    break;
  case 29:
    home_all();
    break;
  case 30:
    home_y();
    position(px, 0, 0, 0);
    break;
    // case 31: home_y2();
  default:
    break;
  }

  cmd = parsenumber('M', -1);
  switch (cmd) {
  case  17:
    motor_enable();
    break;
  case  18:
    motor_disable();
    break;
  case 100:
    help();
    break;
  case 114:
    where();
    break;
  case 256:
    read_color();
    break;
  case 5:
    home_where();
    break;
  case 119:
    endstop_status();
    break;
  case 20:
    led = 11;
    break;
  case 21:
    digitalWrite(lower_led , LOW);
    break;
  case 22:
    led = 7;
    break;
  case 23:
    digitalWrite(upper_led , LOW);
    break;
  case 40:
    motor_enable_x();
    break;
  case 41:
    motor_disable_x();
    break;
  case 50:
    motor_enable_y();
    break;
  case 51:
    motor_disable_y();
    break;
  case 200:
    which_quad();
    break;
    // case 300:
    // color_2();
    // break;

  case 401:
    motor_enable_x();
    motor_enable_y();
     home_all();
      home_y();
    execute_time = 1;
    tell_time = 0;
    break;
  case 402:
    motor_enable_x();
    motor_enable_y();
    home_all();
    home_y();
    play_time_enable = 1;
    break;
  case 403:
    print_current_time();
    break;
  default:
    break;
  }
}


/**
 * prepares the input buffer to receive a new message and tells the serial connected device it is ready for more.
 */
void ready() {
  sofar = 0; // clear input buffer
  Serial.print(F(">"));  // signal ready to receive input
}

void home_where() {

  Serial.println(x_homestate);
  Serial.println(y_homestate);
}

void endstop_status() {
  int x_home = digitalRead(x_stop);
  int x2_home = digitalRead(x2_stop);
  int y_home = digitalRead(y_stop);
  int y2_home = digitalRead(y2_stop);
  //int y3_home = digitalRead(y3_stop);
  int y4_home = digitalRead(y4_stop);
  int y5_home = digitalRead(y5_stop);
  Serial.print("x1:");
  Serial.println(x_home);
  Serial.print("x2:");
  Serial.println(x2_home);
  Serial.print("y1:");
  Serial.println(y_home);
  Serial.print("y2:");
  Serial.println(y2_home);
  //Serial.print("y3:");
  //Serial.println(y3_home);
  Serial.print("y4:");
  Serial.println(y4_home);
  Serial.print("y5:");
  Serial.println(y5_home);
}

void read_color() {

  int color = analogRead(color_read);
  delayMicroseconds(195);
  Serial.print(color);

}
/**
 * set up the pins for each motor
 * Pins fits a Ramps 1.4 board
 */
void motor_setup() {
  motors[0].step_pin = 2;
  motors[0].dir_pin = 5;
  motors[0].enable_pin = 8;
  motors[0].limit_switch_pin = 9;

  motors[1].step_pin = 3;
  motors[1].dir_pin = 7;//6;fliped pin with fade led
  motors[1].enable_pin = 4;
  motors[1].limit_switch_pin = 10;

  motors[2].step_pin = A6;
  motors[2].dir_pin = A7;
  motors[2].enable_pin = 8;
  motors[2].limit_switch_pin = 11;

  motors[3].step_pin = 11;
  motors[3].dir_pin = 13;
  motors[3].enable_pin = 8;
  motors[3].limit_switch_pin = 11;

  int i;
  for (i = 0; i < NUM_AXIES; ++i) {
    // set the motor pin & scale
    pinMode(motors[i].step_pin, OUTPUT);
    pinMode(motors[i].dir_pin, OUTPUT);
    pinMode(motors[i].enable_pin, OUTPUT);
  }
}


void motor_enable() {
  int i;
  for (i = 0; i < NUM_AXIES; ++i) {
    digitalWrite(motors[i].enable_pin, LOW);
  }



}

void motor_enable_x() {

  digitalWrite(motors[0].enable_pin, LOW);

}
void motor_disable_x() {

  digitalWrite(motors[0].enable_pin, HIGH);

}

void motor_enable_y() {

  digitalWrite(motors[1].enable_pin, LOW);

}
void motor_disable_y() {

  digitalWrite(motors[1].enable_pin, HIGH);

}




void motor_disable() {
  int i;
  for (i = 0; i < NUM_AXIES; ++i) {
    digitalWrite(motors[i].enable_pin, HIGH);
  }
}

void which_quad() {
  int quad = 0;
  int color_quad = analogRead(color_read);

  if (color_quad > 250 && color_quad < 400) {
    quad = 3;
  }
  else if (color_quad > 450 && color_quad < 620) {
    quad = 2;
  }
  else if (color_quad > 30 && color_quad < 200) {
    quad = 4;
  }
  else if (color_quad > 700 && color_quad < 900) {
    quad = 1;
  }

  else quad = 9999;

  Serial.print("QUAD:");
  Serial.println(quad);
}


void onestep(int motor) {


  digitalWrite(motors[motor].step_pin, HIGH);
  digitalWrite(motors[motor].step_pin, LOW);
  delayMicroseconds(195);
}


void onestep_y(int motor) {


  digitalWrite(motors[motor].step_pin, HIGH);
  digitalWrite(motors[motor].step_pin, LOW);
  delayMicroseconds(295);
}


void home_x2() {

  x_homestate = false;

  int x_home = digitalRead(x2_stop);
  int y_crash_right = digitalRead(y4_stop);
  int y_crash_left = digitalRead(y5_stop);


  int color = analogRead(color_read);

  if (color > 20 && color < 850) {//400 bl
    digitalWrite(motors[0].dir_pin, HIGH);
  }
  else digitalWrite(motors[0].dir_pin, LOW);

  if (y_crash_right == 0 || y_crash_left == 0) {
    home_y();
  }


  while (x_home == 1) {
    int x_home = digitalRead(x2_stop);
    int y_crash_right = digitalRead(y4_stop);
    int y_crash_left = digitalRead(y5_stop);
    if (x_homestate == true) {
      //motor_disable();
      break;
    }
    if (x_home == 0) {
      digitalWrite(test_led, HIGH);
      x_homestate = true;
      break;
    }
    if (y_crash_right == 0 || y_crash_left == 0) {
      home_y();
    }

    digitalWrite(test_led, LOW);
    onestep(0);

  }
}


void home_all() {

  x_homestate = false;

  int x_home = digitalRead(x_stop);
  int x2_home = digitalRead(x2_stop);
  int y_crash_right = digitalRead(y4_stop);
  int y_crash_left = digitalRead(y5_stop);


  int color = analogRead(color_read);

  if (color > 20 && color < 850) {//400 black
    digitalWrite(motors[0].dir_pin, HIGH);
  }
  else digitalWrite(motors[0].dir_pin, LOW);

  if (y_crash_right == 0 || y_crash_left == 0) {
    home_y();
  }


  while (x_home == 1 || x2_home == 1) {
    int x_home = digitalRead(x_stop);
    int x2_home = digitalRead(x2_stop);
    int y_crash_right = digitalRead(y4_stop);
    int y_crash_left = digitalRead(y5_stop);
    if (x_homestate == true) {
      //motor_disable();
      break;
    }
    if (x_home == 0) {
      digitalWrite(test_led, HIGH);
      position(x1_home_val, py, 0, 0);
      x_homestate = true;
      break;
    }

    if (x2_home == 0) {
      digitalWrite(test_led, HIGH);
      position(x2_home_val, py, 0, 0);
      x_homestate = true;
      break;
    }


    if (y_crash_right == 0 || y_crash_left == 0) {
      home_y();
    }

    digitalWrite(test_led, LOW);
    onestep(0);

  }
}












void home_y() { // try if method

  y_homestate = false;
  int y_home = digitalRead(y_stop);
  int y_crash_right = digitalRead(y4_stop);
  int y_crash_left = digitalRead(y5_stop);


  if (y_crash_left == 0 ) {
    digitalWrite(motors[1].dir_pin, HIGH);
    //home_dir = 1;
  }
  else digitalWrite(motors[1].dir_pin, LOW);





  while (y_home == 1) {
    int y_home = digitalRead(y_stop);
    int y_crash_left = digitalRead(y5_stop);
    if (y_homestate == true) {
      // motor_disable();
      break;
    }
    if (y_home == 0) {
      digitalWrite(test_led, HIGH);
      y_homestate = true;
      break;
    }
    if (y_crash_left == 0) {
      digitalWrite(motors[1].dir_pin, HIGH);
    }


    digitalWrite(test_led, LOW);

    onestep_y(1);

  }



}







void time_demo_2() {



  if ( execute_time == 1) {
    //home_all();
    //home_y();
    mode_abs = 1;

    unsigned long demo_currentMillis = millis();




    if ( demo_currentMillis - demo_previousMillis > demo_interval) {
      //Serial.println(demo_currentMillis - demo_previousMillis);

      demo_previousMillis = demo_currentMillis;

      time_9_shift(9, time_count);
      //Serial.println(time_count);
      time_count++;
    }


    if (time_count >= 60) {
      execute_time = 0;
      time_count = 0;
    }


  }
}

// eight o clock

void time_8(int hh, int mm) {
  float xx;
  float yy;



  if (mm >= 0 && mm < 18) {
    float hour_angle = (0.5 * (hh * 60 + mm)) - 90 ;
    float minute_angle = (6 * mm) - hour_angle + 270;
    xx = mapf(hour_angle, 0, 180, 55, 102.2);
    yy = mapf(minute_angle, 0, 360, 0, 102.2);

    line(xx, py, 0, 0);
    line(px, yy, 0, 0);
    Serial.print(xx);
    Serial.print(",");
    Serial.println(yy);
    //Serial.println("first quadrant");
  }

  if (mm >= 19 && mm <= 43) {


    float hour_angle = (6 * mm) - 90;
    float minute_angle = 0.5 * (hh * 60 + mm) + 270 - hour_angle;
    xx = mapf(hour_angle, 0, 180, 55, 102.2);
    yy = mapf(minute_angle, 0, 360, -102.2, 0);

    
    line(px, yy, 0, 0);
    line(xx, py, 0, 0);//y called first
    Serial.print(xx);
    Serial.print(",");
    Serial.println(yy);
    led = 6;
    digitalWrite(11, LOW);
    //Serial.println("middle quadrant");
  }

  if (mm >= 44 && mm <= 59) {

    float hour_angle = (0.5 * (hh * 60 + mm)) - 90 ;
    float minute_angle = (6 * mm) - hour_angle + 270;
    xx = mapf(hour_angle, 0, 180, 55, 102.2);
    yy = mapf(minute_angle, 0, 360, -102.2, 0);

    line(xx, py, 0, 0);
    line(px, yy, 0, 0);

    Serial.print(xx);
    Serial.print(",");
    Serial.println(yy);
    led = 11;
    digitalWrite(6, LOW);


  }


}



// seven o clock

void time_7(int hh, int mm) {
  float xx;
  float yy;



  if (mm >= 0 && mm < 23) {
    float hour_angle = (0.5 * (hh * 60 + mm)) - 90 ;
    float minute_angle = (6 * mm) - hour_angle + 270;
    xx = mapf(hour_angle, 0, 180, 55, 102.2);
    yy = mapf(minute_angle, 0, 360, 0, 102.2);

    line(xx, py, 0, 0);
    line(px, yy, 0, 0);
    Serial.print(xx);
    Serial.print(",");
    Serial.println(yy);
    //Serial.println("first quadrant");
  }




  if (mm >= 24 && mm <= 38) {


    float hour_angle = (6 * mm) - 90;
    float minute_angle = 0.5 * (hh * 60 + mm) + 270 - hour_angle;
    xx = mapf(hour_angle, 0, 180, 55, 102.2);
    yy = mapf(minute_angle, 0, 360, -102.2, 0);

    
    line(px, yy, 0, 0);
    line(xx, py, 0, 0);// y called first due to crash
    Serial.print(xx);
    Serial.print(",");
    Serial.println(yy);
    led = 6;
    digitalWrite(11, LOW);
    //Serial.println("middle quadrant");
  }




  if (mm >= 39 && mm <= 59) {

    float hour_angle = (0.5 * (hh * 60 + mm)) - 90 ;
    float minute_angle = (6 * mm) - hour_angle + 270;
    xx = mapf(hour_angle, 0, 180, 55, 102.2);
    yy = mapf(minute_angle, 0, 360, -102.2, 0);

    line(xx, py, 0, 0);
    line(px, yy, 0, 0);

    Serial.print(xx);
    Serial.print(",");
    Serial.println(yy);
    led = 11;
    digitalWrite(6, LOW);


  }


}
// SIX o clock

void time_6(int hh, int mm) {
  float xx;
  float yy;



  if (mm >= 0 && mm < 27) {
    float hour_angle = (0.5 * (hh * 60 + mm)) - 90 ;
    float minute_angle = (6 * mm) - hour_angle + 270;
    xx = mapf(hour_angle, 0, 180, 55, 102.2);
    yy = mapf(minute_angle, 0, 360, 0, 102.2);

    line(xx, py, 0, 0);
    line(px, yy, 0, 0);
    Serial.print(xx);
    Serial.print(",");
    Serial.println(yy);
    //Serial.println("first quadrant");
  }




  if (mm >= 28 && mm <= 32) {


    float hour_angle = (6 * mm) - 90;
    float minute_angle = 0.5 * (hh * 60 + mm) + 270 - hour_angle;
    xx = mapf(hour_angle, 0, 180, 55, 102.2);
    yy = mapf(minute_angle, 0, 360, -102.2, 0);

    line(xx, py, 0, 0);
    line(px, yy, 0, 0);
    Serial.print(xx);
    Serial.print(",");
    Serial.println(yy);
    led = 6;
    digitalWrite(11, LOW);
    //Serial.println("middle quadrant");
  }




  if (mm >= 33 && mm <= 59) {

    float hour_angle = (0.5 * (hh * 60 + mm)) - 90 ;
    float minute_angle = (6 * mm) - hour_angle + 270;
    xx = mapf(hour_angle, 0, 180, 55, 102.2);
    yy = mapf(minute_angle, 0, 360, -102.2, 0);

    line(xx, py, 0, 0);
    line(px, yy, 0, 0);

    Serial.print(xx);
    Serial.print(",");
    Serial.println(yy);
    led = 11;
    digitalWrite(6, LOW);


  }


}

// 5 o clock

void time_5(int hh, int mm) {
  float xx;
  float yy;



  if (mm >= 0 && mm < 27) {
    float hour_angle = (0.5 * (hh * 60 + mm)) - 90 ;
    float minute_angle = (6 * mm) - hour_angle + 270;
    xx = mapf(hour_angle, 0, 180, 55, 102.2);
    yy = mapf(minute_angle, 0, 360, -102.2, 0);

    line(xx, py, 0, 0);
    line(px, yy, 0, 0);
    Serial.print(xx);
    Serial.print(",");
    Serial.println(yy);
    //Serial.println("first quadrant");
  }




  if (mm >= 28 && mm <= 32) {


    float hour_angle = (6 * mm) - 90;
    float minute_angle = 0.5 * (hh * 60 + mm) + 270 - hour_angle;
    xx = mapf(hour_angle, 0, 180, 55, 102.2);
    yy = mapf(minute_angle, 0, 360, -102.2, 0);

    line(xx, py, 0, 0);
    line(px, yy, 0, 0);
    Serial.print(xx);
    Serial.print(",");
    Serial.println(yy);
    led = 6;
    digitalWrite(11, LOW);
    //Serial.println("middle quadrant");
  }




  if (mm >= 33 && mm <= 59) {

    float hour_angle = (0.5 * (hh * 60 + mm)) - 90 ;
    float minute_angle = (6 * mm) - hour_angle - 90;
    xx = mapf(hour_angle, 0, 180, 55, 102.2);
    yy = mapf(minute_angle, 0, 360, -102.2, 0);

    line(xx, py, 0, 0);
    line(px, yy, 0, 0);

    Serial.print(xx);
    Serial.print(",");
    Serial.println(yy);
    led = 11;
    digitalWrite(6, LOW);


  }


}



// 4 o clock

void time_4(int hh, int mm) {
  float xx;
  float yy;



  if (mm >= 0 && mm < 21) {
    float hour_angle = (0.5 * (hh * 60 + mm)) - 90 ;
    float minute_angle = (6 * mm) - hour_angle + 270;
    xx = mapf(hour_angle, 0, 180, 55, 102.2);
    yy = mapf(minute_angle, 0, 360, -102.2, 0);

    line(xx, py, 0, 0);
    line(px, yy, 0, 0);
    Serial.print(xx);
    Serial.print(",");
    Serial.println(yy);
    //Serial.println("first quadrant");
  }




  if (mm >= 22 && mm <= 36) {


    float hour_angle = (6 * mm) - 90;
    float minute_angle = 0.5 * (hh * 60 + mm) + 270 - hour_angle;
    xx = mapf(hour_angle, 0, 180, 55, 102.2);
    yy = mapf(minute_angle, 0, 360, -102.2, 0);

    
    line(px, yy, 0, 0);
    line(xx, py, 0, 0); // y flip due to crash
    Serial.print(xx);
    Serial.print(",");
    Serial.println(yy);
    led = 6;
    digitalWrite(11, LOW);
    //Serial.println("middle quadrant");
  }




  if (mm >= 36 && mm <= 59) {

    float hour_angle = (0.5 * (hh * 60 + mm)) - 90 ;
    float minute_angle = (6 * mm) - hour_angle - 90;
    xx = mapf(hour_angle, 0, 180, 55, 102.2);
    yy = mapf(minute_angle, 0, 360, -102.2, 0);

    line(xx, py, 0, 0);
    line(px, yy, 0, 0);

    Serial.print(xx);
    Serial.print(",");
    Serial.println(yy);
    led = 11;
    digitalWrite(6, LOW);


  }


}



// 3 o clock
void time_3(int hh, int mm) {
  float xx;
  float yy;



  if (mm >= 0 && mm < 16) {
    float hour_angle = (0.5 * (hh * 60 + mm)) - 90 ;
    float minute_angle = (6 * mm) - hour_angle + 270;
    xx = mapf(hour_angle, 0, 180, 55, 102.2);
    yy = mapf(minute_angle, 0, 360, -102.2, 0);

    line(xx, py, 0, 0);
    line(px, yy, 0, 0);
    Serial.print(xx);
    Serial.print(",");
    Serial.println(yy);
    //Serial.println("first quadrant");
  }




  if (mm >= 17 && mm <= 41) {


    float hour_angle = (6 * mm) - 90;
    float minute_angle = 0.5 * (hh * 60 + mm) + 270 - hour_angle;
    xx = mapf(hour_angle, 0, 180, 55, 102.2);
    yy = mapf(minute_angle, 0, 360, -102.2, 0);

   
    line(px, yy, 0, 0);
     line(xx, py, 0, 0);// y flip due to crash
    Serial.print(xx);
    Serial.print(",");
    Serial.println(yy);
    led = 6;
    digitalWrite(11, LOW);
    //Serial.println("middle quadrant");
  }




  if (mm >= 42 && mm <= 59) {

    float hour_angle = (0.5 * (hh * 60 + mm)) - 90 ;
    float minute_angle = (6 * mm) - hour_angle - 90;
    xx = mapf(hour_angle, 0, 180, 55, 102.2);
    yy = mapf(minute_angle, 0, 360, -102.2, 0);

    line(xx, py, 0, 0);
    line(px, yy, 0, 0);

    Serial.print(xx);
    Serial.print(",");
    Serial.println(yy);
    led = 11;
    digitalWrite(6, LOW);


  }


}


// 9 - 14 o clock

void time_9_shift(int hh, int mm) {
  float xx;
  float yy;



  if (mm >= 0 && mm < 15) {
    float hour_angle = (0.5 * (hh * 60 + mm)) - 270 ;
    float minute_angle = (6 * mm) - hour_angle + 90;
    xx = mapf(hour_angle, 0, 180, 212.12, 259.25);
    yy = mapf(minute_angle, 0, 180, 0, 51.1);

    line(xx, py, 0, 0);
    line(px, yy, 0, 0);
    Serial.print(xx);
    Serial.print(",");
    Serial.println(yy);
    //Serial.println("first quadrant");
  }

  if (mm >= 15 && mm < 16) {


    float hour_angle = (6 * mm) - 90;
    float minute_angle = 0.5 * (hh * 60 + mm) + 270 - hour_angle;
    xx = mapf(hour_angle, 0, 180, 55, 102.2);
    yy = mapf(minute_angle, 0, 360, -102.2, 0);

    line(78.25, py, 0, 0);
    line(px, 51.1, 0, 0);
    Serial.print(px);
    Serial.print(",");
    Serial.println(py);

  
    line(px, yy, 0, 0);
      line(xx, py, 0, 0);// y flipped
    Serial.print(xx);
    Serial.print(",");
    Serial.println(yy);
    //Serial.println(hh);
    led = 6;




  }


  if (mm >= 16 && mm < 45) {


    float hour_angle = (6 * mm) - 90;
    float minute_angle = 0.5 * (hh * 60 + mm) + 270 - hour_angle;
    xx = mapf(hour_angle, 0, 180, 55, 102.2);
    yy = mapf(minute_angle, 0, 360, -102.2, 0);

   
    line(px, yy, 0, 0);
     line(xx, py, 0, 0);//y flipped
    Serial.print(xx);
    Serial.print(",");
    Serial.println(yy);
    led = 6;
    digitalWrite(11, LOW);
    //Serial.println("middle quadrant");
  }


  if (mm >= 45 && mm < 46) {

    float hour_angle = (0.5 * (hh * 60 + mm)) - 270 ;
    float minute_angle = (6 * mm) - hour_angle + 90;
    xx = mapf(hour_angle, 0, 180, 212.12, 259.25);
    yy = mapf(minute_angle, 0, 360, -102.2, 0);


    line(px, 0, 0, 0);
    Serial.print(px);
    Serial.print(",");
    Serial.println(py);


    line(xx, py, 0, 0);
    line(px, yy, 0, 0);

    Serial.print(xx);
    Serial.print(",");
    Serial.println(yy);
    led = 11;
    digitalWrite(6, LOW);


  }

  if (mm >= 46 && mm <= 59) {

    float hour_angle = (0.5 * (hh * 60 + mm)) - 270 ;
    float minute_angle = (6 * mm) - hour_angle + 90;
    xx = mapf(hour_angle, 0, 180, 212.12, 259.25);
    yy = mapf(minute_angle, 0, 360, -102.2, 0);

    line(xx, py, 0, 0);
    line(px, yy, 0, 0);

    Serial.print(xx);
    Serial.print(",");
    Serial.println(yy);
    led = 11;
    digitalWrite(6, LOW);


  }


}



void time_1_2(int hh, int mm) {
  float xx;
  float yy;

  hh = hh + 12;

  if (mm >= 0 && mm < 15) {
    float hour_angle = (0.5 * (hh * 60 + mm)) - 270 ;
    float minute_angle = (6 * mm) - hour_angle + 90;
    xx = mapf(hour_angle, 0, 180, 212.12, 259.25);
    yy = mapf(minute_angle, 0, 180, 0, 51.1);

    line(xx, py, 0, 0);
    line(px, yy, 0, 0);
    Serial.print(xx);
    Serial.print(",");
    Serial.println(yy);
    //Serial.println("first quadrant");
  }

  if (mm >= 15 && mm < 16) {


    float hour_angle = (6 * mm) - 90;
    float minute_angle = 0.5 * (hh * 60 + mm) + 270 - hour_angle;
    xx = mapf(hour_angle, 0, 180, 55, 102.2);
    yy = mapf(minute_angle, 0, 360, -102.2, 0);

    line(78.25, py, 0, 0);
    line(px, 51.1, 0, 0);
    Serial.print(px);
    Serial.print(",");
    Serial.println(py);

    line(xx, py, 0, 0);
    line(px, yy, 0, 0);
    Serial.print(xx);
    Serial.print(",");
    Serial.println(yy);
    //Serial.println(hh);





  }


  if (mm >= 16 && mm < 45) {


    float hour_angle = (6 * mm) - 90;
    float minute_angle = 0.5 * (hh * 60 + mm) + 270 - hour_angle;
    xx = mapf(hour_angle, 0, 180, 55, 102.2);
    yy = mapf(minute_angle, 0, 360, -102.2, 0);

    
    line(px, yy, 0, 0);
    line(xx, py, 0, 0);//flip y
    Serial.print(xx);
    Serial.print(",");
    Serial.println(yy);
    led = 6;
    digitalWrite(11, LOW);
    //Serial.println("middle quadrant");
  }


  if (mm >= 45 && mm < 46) {

    float hour_angle = (0.5 * (hh * 60 + mm)) - 270 ;
    float minute_angle = (6 * mm) - hour_angle + 90;
    xx = mapf(hour_angle, 0, 180, 212.12, 259.25);
    yy = mapf(minute_angle, 0, 360, -102.2, 0);


    line(px, 0, 0, 0);
    Serial.print(px);
    Serial.print(",");
    Serial.println(py);


    line(xx, py, 0, 0);
    line(px, yy, 0, 0);

    Serial.print(xx);
    Serial.print(",");
    Serial.println(yy);
    led = 11;
    digitalWrite(6, LOW);


  }

  if (mm >= 46 && mm <= 59) {

    float hour_angle = (0.5 * (hh * 60 + mm)) - 270 ;
    float minute_angle = (6 * mm) - hour_angle + 90;
    xx = mapf(hour_angle, 0, 180, 212.12, 259.25);
    yy = mapf(minute_angle, 0, 360, -102.2, 0);

    line(xx, py, 0, 0);
    line(px, yy, 0, 0);

    Serial.print(xx);
    Serial.print(",");
    Serial.println(yy);
    led = 11;
    digitalWrite(6, LOW);


  }


}



float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void color_2() {
  int color_y = analogRead(y3_stop);
  Serial.println(color_y);

}

void blink_me() {

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis > interval) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;
    analogWrite(led, brightness);
    // change the brightness for next time through the loop:
    brightness = brightness + fadeAmount;

    // reverse the direction of the fading at the ends of the fade:
    if (brightness == 0 || brightness == 255) {
      fadeAmount = -fadeAmount ;
    }
    // wait for 30 milliseconds to see the dimming effect
    //delay(50);
  }

}


// time display from the rtc


void print_current_time() {
  int gh = hour();
  int gm = minute();
  Serial.print(gh);
  Serial.print(":");
  Serial.println(gm);


}

//complete control

void complete_time() {

  if (tell_time == 1){

    int mm = minute();
    int chk_m = abs(disp_m - mm);
    if ( mm == 0 && hourly_home == 0){
      Serial.println("need to home ");
      motor_enable();
      home_all();
      home_y();
      position(px,0,0,0);
      hourly_home = 1;
      motor_disable();
    }

    if (mm == 59){
      hourly_home = 0;
    }
    if (chk_m > 0) {
      motor_enable();
      display_time();
      disp_m = mm;
      motor_disable();
    }

  }
  
}




void play_time() {

  if ( play_time_enable == 1) {

    int mm = minute();
    int chk_m = abs(disp_m - mm);
    if ( mm == 0 && hourly_home == 0){
      Serial.println("need to home ");
      motor_enable();
      home_all();
      home_y();
      position(px,0,0,0);
      hourly_home = 1;
      motor_disable();
    }

    if (mm == 59){
      hourly_home = 0;
    }
    if (chk_m > 0) {
      motor_enable();
      display_time();
      disp_m = mm;
      motor_disable();
    }


  }
}


void display_time() {

  int hh = hour();
  int m = minute();
  if (hh == 0){
    hh = 12;
  }
  if (hh > 12) {
    hh = hh - 12;
  }
  else hh = hh;

  if (hh >= 9 && hh <= 12) {
    time_9_shift(hh, m);
    Serial.print(hh);
    Serial.print(",");
    Serial.println(m);
  }
  if (hh == 3) {
    time_3(hh, m);
    Serial.print(hh);
    Serial.print(",");
    Serial.println(m);
  }
  if (hh == 4) {
    time_4(hh, m);
    Serial.print(hh);
    Serial.print(",");
    Serial.println(m);
  }
  if (hh == 5) {
    time_5(hh, m);
    Serial.print(hh);
    Serial.print(",");
    Serial.println(m);
  }
  if (hh == 6) {
    time_6(hh, m);
    Serial.print(hh);
    Serial.print(",");
    Serial.println(m);
  }
  if (hh == 7) {
    time_7(hh, m);
    Serial.print(hh);
    Serial.print(",");
    Serial.println(m);
  }
  if (hh == 8) {
    time_8(hh, m);
    Serial.print(hh);
    Serial.print(",");
    Serial.println(m);
  }
  if (hh == 1 || hh == 2) {
    time_1_2(hh, m);
    Serial.print(hh);
    Serial.print(",");
    Serial.println(m);
  }

}

void park(){
 int buttonstate = digitalRead(park_button);
 
 if (buttonstate == HIGH){
 
  if (park_state == 0){

    motor_enable();
      home_all();
      home_y();
     // position(px,0,0,0);
      position(0,py,0,0);

    park_state = 1;
    
  }

}

  
}


/**
 * First thing this machine does on startup.  Runs only once.
 */
void setup() {
  Serial.begin(BAUD);  // open coms
  pinMode(test_led, OUTPUT);
  pinMode(x_stop, INPUT);
  pinMode(x2_stop, INPUT);
  // pinMode(y3_stop, INPUT);
  pinMode(y2_stop, INPUT);
  pinMode(y4_stop, INPUT);
  pinMode(y5_stop, INPUT);
  pinMode(lower_led , OUTPUT);
  pinMode(upper_led , OUTPUT);
  pinMode(y_stop, INPUT);
  pinMode(park_button, INPUT);
  motor_setup();
  motor_disable();
 //time_t t = now();
 //RTC.set(t);   // set the RTC and the system time to the received value
     

  setSyncProvider(RTC.get);   // the function to get the time from the RTC
  

  where();  // for debugging purposes
  help();  // say hello
  position(0, 0, 0, 0); // set starting position
  feedrate(100);  // set default speed
  ready();
 
  //setTime(13, 05, 00, 12, 02, 2012);
  
  // disable to remove complete control
  motor_enable();
  home_all();
  home_y();
  motor_disable();
}


/**
 * After setup() this machine will repeat loop() forever.
 */
void loop() {
  // listen for serial commands
  // motor_enable(); // do something with the command
  // line (200, 200, 200, 200);

  blink_me();
  time_demo_2();
  play_time();
  complete_time();
  park();
  //onestep(0);

  while (Serial.available() > 0) { // if something is available
    char c = Serial.read(); // get it
    Serial.print(c);  // repeat it back so I know you got the message
    if (sofar < MAX_BUF - 1) buffer[sofar++] = c; // store it
    if (c == ';') {
      // entire message received
      buffer[sofar] = 0; // end the buffer so string functions work right
      Serial.print(F("\r\n"));  // echo a return character for humans
      processCommand();

      ready();
      // onestep(1);



    }
  }
}


/**
 * This file is part of GcodeCNCDemo.
 * 
 * GcodeCNCDemo is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * GcodeCNCDemo is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with Foobar. If not, see <http://www.gnu.org/licenses/>.
 */


