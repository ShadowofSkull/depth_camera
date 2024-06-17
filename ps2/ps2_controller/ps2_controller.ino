#include <CytronMotorDriver.h>
#include <math.h> //math package
#include <SoftwareSerial.h>
#include <Cytron_PS2_Shield.h>
 
Cytron_PS2_Shield ps2(10, 11); // SoftwareSerial: Rx and Tx pin
SoftwareSerial ps2Serial =  SoftwareSerial(10, 11); // Ps2 serial
 
// UNO PWM pins | 3, 5, 6, 9, 10, or 11
// MEGA PWM pins | 2 - 13, 44 - 46
// MEGA DIGITAL pins | 22 - 43, 47 - 49
 
// BASE PS2 BUTTON INPUT
int tri_up = 0;
int cross_down = 0;
int circ_right = 0;
int square_left = 0;
 
int up_button = 0;
int right_button = 0;
int down_button = 0;
int left_button = 0;
 
int L1 = 0;
int R1 = 0;
 
int L2 = 0;
int R2 = 0;

int y = 0;
int x = 0;
int rx = 0;
 
void setup()
{
  Serial.begin(9600);     
  ps2Serial.begin(9600);
  ps2.AttachPS2Serial(&ps2Serial);
}

void loop()
{
  delay(50);

  // PS2 Button Init initialisation
  tri_up = ps2.readButton(PS2_TRIANGLE);
  cross_down = ps2.readButton(PS2_CROSS);
  square_left = ps2.readButton(PS2_SQUARE);
  circ_right = ps2.readButton(PS2_CIRCLE);
 
  up_button = ps2.readButton(PS2_UP);
  right_button = ps2.readButton(PS2_RIGHT);
  down_button = ps2.readButton(PS2_DOWN);
  left_button = ps2.readButton(PS2_LEFT);
 
  L1 = ps2.readButton(PS2_LEFT_1);
  R1 = ps2.readButton(PS2_RIGHT_1);
 
  L2 = ps2.readButton(PS2_LEFT_2);
  R2 = ps2.readButton(PS2_RIGHT_2);
 
  // ===== Joystick Holonomic Movement =====
  // analog joysticks initialisation
  y = -((ps2.readButton(PS2_JOYSTICK_LEFT_Y_AXIS)) - 128);
  x = (ps2.readButton(PS2_JOYSTICK_LEFT_X_AXIS) - 128);
  rx = ps2.readButton(PS2_JOYSTICK_RIGHT_X_AXIS) - 128;
 
  // formula that moves the wheels | -128 - 128
  double frontLeftPower = (y + x + rx);
  double backLeftPower = (y - x + rx);
  double frontRightPower = (y - x - rx);
  double backRightPower = (y + x - rx);

  Serial.println(R1);
 
  // if (movementMode == 1)
  // {
  //   frontLeftPower = map(frontLeftPower, -128, 128, -maxMovement, maxMovement);
  //   frontRightPower = map(frontRightPower, -128, 128, -maxMovement, maxMovement);
  //   backLeftPower = map(backLeftPower, -128, 128, -maxMovement, maxMovement);
  //   backRightPower = map(backRightPower, -128, 128, -maxMovement, maxMovement);
  // }
 
  // // double check if correct motors
  // frontLMotor.setSpeed(frontLeftPower);  // front left
  // frontRMotor.setSpeed(frontRightPower); // back left
  // backLMotor.setSpeed(backLeftPower);    // front right
  // backRMotor.setSpeed(backRightPower);   // back right
  // // ===== Joystick Holonomic Movement =====

}