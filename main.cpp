/*
Author: Ryan - David Reyes
*/
//check updated ctrlrnix
//and replace it and stuff
#include <iostream>
#include <cmath>
#include "tcpipnix.h"
#include "ctrlrnix.h"


const int LSTICK_DEADZONE = 200;
const int LSTICK_MAX = 32767;
const int LSTICK_DEFAULT = 0;

//steering commands range from -MAX_STEER to MAX_STEER
const int MAX_STEER = 15000; 
//const int STEERING_MAX = 30000;
//const int STEERING_MIN = -30000;

/*
   const int TRIGGER_MIN = 134;
   const int TRIGGER_MAX = 65401;

*/
const int BUTTON_DEFAULT = 0;

//Throttle is always a positive integer
//based on a proportion of 0-TRIGGER_MAX
const int TRIGGER_MAX = 65535;
const int TRIGGER_OFFSET = 32767;
const int TRIGGER_DEFAULT = -32767;
const int START_BUTTON = 7;
const int LBUMPER = 4;
const int RBUMPER = 5;

const int MAX_THROTTLE = 50; 
const int MAX_BRAKE = 6656;

/*
const int RIGHT_BUMPER = 0x20;
const int LEFT_BUMPER = 0x10;
*/

const int GEAR_POSITIONS[5] = {0, -2048*7, -2048*13, -2048*16, -2048*24};

enum gearPos {PARK, REVERSE, NEUTRAL, LOW, HIGH};

const char STEERMSG = 'S';
const char GEARMSG = 'G';
const char THROTMSG = 'T';
const char BRAKEMSG = 'B';
const char STOPMSG = 'X';

const int PORT = 65534;
char * ADDRESS = "127.0.0.1"; //11.26";
char * CTRLRADDRDEFAULT = "/dev/input/js0";

/*
 * The controller returns 16 bit values for the analog sticks and the 
 * analog triggers. all other buttons return 0 when not pressed and 1 when
 * pressed.
 */
struct controllerValues
{
  int LstickXValue;
  int startButton;
  int LTrigger;
  int RTrigger;
  int LBumper;
  int RBumper;
};

struct commandValues
{
  unsigned int throttleVal;
  int steerVal;
  unsigned int brakeVal;
  int gearSetting;
  int gearVal;
  int startPressed;
  
  commandValues()
  {
    throttleVal = 0;
    steerVal = 0;
    brakeVal = 0;
    gearSetting = PARK;
    gearVal = GEAR_POSITIONS[PARK];
    startPressed = 0;
  }
};

bool init(  int argc, char ** argv, TCP & tcpConnection, Ctrlr &controller);
void readController(Ctrlr & controller, controllerValues & ctrlrValues);
void mapValues(const controllerValues & ctrlrValues, commandValues & comValues);
void sendCommands(TCP & tcpConnection, const commandValues & comValues);
void sendData(TCP & tcpConnection, char messageType, int data);
void calibrateController(Ctrlr & controller);

int main(int argc, char ** argv)
{
  bool active = true;

  Ctrlr controller;
  TCP tcpConnection;

  controllerValues ctrlrValues;
  commandValues comValues;

  //if initialization fails
  if (! init(argc, argv, tcpConnection, controller))
    return 1;
  

  while (active)
  {
    //if the controller registers a change from prec state
    readController(controller, ctrlrValues);

    //map the values and send the updated commands
    mapValues(ctrlrValues, comValues);
    sendCommands(tcpConnection, comValues);
  }
  return 0;
}

bool init(  int argc, char ** argv, TCP & tcpConnection, Ctrlr &controller)
{
  char * address = ADDRESS;
  char * ctrlr_addr = CTRLRADDRDEFAULT;

  switch (argc)
  {
    case 3:
      ctrlr_addr = argv[2];
    case 2:
      address = argv[1];
      break;

    default:
      break;
  }

  if (controller.openController(ctrlr_addr))
  {
    calibrateController(controller);
    std::cout << "Controller initialized!" << std::endl;
  }
  else 
  {
    std::cout << "Controller not found!" << std::endl;
    return false;
  }

  if (tcpConnection.connectToHost(PORT, address))
  {
    std::cout << "Connection success!" << std::endl;
    return true;
  }
  else
  {
    std::cout << "Connection Failed!" << std::endl;
    return false;
  }
}


//returns true if the controller data has changed since the last update
void readController(Ctrlr & controller, controllerValues & ctrlrValues)
{
  controller.update();
  ctrlrValues.LstickXValue = controller.getStickvalue(XAXIS);
  //get L2 
  ctrlrValues.LTrigger = controller.getStickvalue(YAXIS2);
  //get R2
  ctrlrValues.RTrigger = controller.getStickvalue(YHAT);
  //the following buttons are as of latest controller test
  ctrlrValues.startButton = controller.getButton(START_BUTTON);
  ctrlrValues.LBumper = controller.getButton(LBUMPER); 
  ctrlrValues.RBumper = controller.getButton(RBUMPER);
}

//this function translates the controller data into commands for the ATV
void mapValues(const controllerValues & ctrlrValues, commandValues & comValues)
{
  comValues.startPressed = ctrlrValues.startButton;
  static int prevLBumper = 0, prevRBumper = 0;
  //make the throttle and the brake be some proportion of
  //the max brake and max throttle
  comValues.brakeVal = (ctrlrValues.LTrigger + TRIGGER_OFFSET)* 
    static_cast<double> (MAX_BRAKE)/(TRIGGER_MAX);

  comValues.throttleVal = (ctrlrValues.RTrigger + TRIGGER_OFFSET)*
    static_cast<double> (MAX_THROTTLE)/(TRIGGER_MAX);

  //if the controller's L-stick is out of the deadzone
  //if (ctrlrValues.LstickXValue > LSTICK_DEADZONE)
  if (std::abs(ctrlrValues.LstickXValue) > LSTICK_DEADZONE)
  {
    comValues.steerVal = ctrlrValues.LstickXValue * 
      static_cast<double> (MAX_STEER) / 
      (LSTICK_MAX);
  }

  /* if the right bumper is pressed, the gear is set to some gear less
   * than high, and the throttle being sent to the ATV is zero.
   */

  if ( (ctrlrValues.RBumper != prevRBumper) || 
        (ctrlrValues.LBumper != prevLBumper ))
  {
    if (  (ctrlrValues.RBumper == 1) && 
        (comValues.gearSetting < HIGH) &&
        (comValues.throttleVal == 0) )
    {
      //the gear value is the new gear setting.
      comValues.gearVal=GEAR_POSITIONS[++comValues.gearSetting];
    }

    /* if the left bumper is pressed, and the gear is in some gear greater
     * than park, and the throttle sent to ATV = 0
     */
    else if ( (ctrlrValues.LBumper == 1) &&
        (comValues.gearSetting > PARK) &&
        (comValues.throttleVal == 0) )
    {
      //the gear value is the new gear setting.
      comValues.gearVal=GEAR_POSITIONS[--comValues.gearSetting];
    }
    prevRBumper = ctrlrValues.RBumper;
    prevLBumper = ctrlrValues.LBumper;
  }
}

void sendCommands(TCP & tcpConnection, const commandValues & comValues)
{
  static bool initialized = false;
  static commandValues prevComValues;
  if (!initialized)
  {
    prevComValues = comValues;
    initialized = true;
  }

  /* throttleVal THROTMSG
   * steerVal STEERMSG
   * brakeVal BRAKEMSG
   * gearSetting 
   * gearVal GEARMSG
   * startPressed STOPMSG
   */

  /* compare the current values to the prev values, and if they're 
   * different, send the updated commands to ATV host.
   */
  if(comValues.throttleVal != prevComValues.throttleVal)
    sendData(tcpConnection, THROTMSG, comValues.throttleVal);

  if(comValues.steerVal != prevComValues.steerVal)
    sendData(tcpConnection, STEERMSG, comValues.steerVal);

  if(comValues.brakeVal != prevComValues.brakeVal)
    sendData(tcpConnection, BRAKEMSG, comValues.brakeVal);

  //takes care of both gearsetting and gearval
  if(comValues.gearSetting != prevComValues.gearSetting)
    sendData(tcpConnection, GEARMSG, comValues.gearVal);

  if(comValues.startPressed == 1)
    sendData(tcpConnection, STOPMSG, 0x0000);

  prevComValues = comValues;
}

void sendData(TCP & tcpConnection, char messageType, int data)
{
  //send the character that identifies the data first
  tcpConnection.sendData(tcpConnection.getSocket(), 
    (char *) &messageType, sizeof(messageType));

  tcpConnection.sendData(tcpConnection.getSocket(),
    (char *) &data, sizeof(data));
}

void calibrateController(Ctrlr & controller)
{
  std::cout << "Please press/move the L Stick" <<std::endl;
  while (controller.getStickvalue(XAXIS) != LSTICK_DEFAULT)
    controller.update();
  
  //get L2 
  std::cout << "Please press the L Trigger" <<std::endl;
  while(controller.getStickvalue(YAXIS2) != TRIGGER_DEFAULT)
    controller.update();

  //get R2
  std::cout << "Please press the R Trigger" <<std::endl;
  while(controller.getStickvalue(YHAT) != TRIGGER_DEFAULT)
    controller.update();

  std::cout << "Please press the Start Button" <<std::endl;
  while (controller.getButton(START_BUTTON) != BUTTON_DEFAULT)
    controller.update();
  std::cout << "Please press the L Bumper" <<std::endl;
  while (controller.getButton(LBUMPER) != BUTTON_DEFAULT)
    controller.update();
  std::cout << "Please press the R Bumper" <<std::endl;
  while (controller.getButton(RBUMPER) != BUTTON_DEFAULT)
    controller.update();
}
