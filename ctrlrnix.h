/*
  Author:Ryan - David Reyes
*/
#ifndef CTRLRNIX_H_
#define CTRLRNIX_H_
#include <string>

//The following are handy variables to use when referring to joystick data
const int XAXIS = 0;
const int YAXIS = 1;
const int XAXIS2 = 3;
const int YAXIS2 = 2;
const int XHAT = 4;
const int YHAT = 5;

//Configure these for the properties of the gamepad
const int NUM_ANALOG_STICKS = 3; //the logitech wireless has 4
const int NUM_AXES = 2;
const int NUM_BUTTONS = 11;



const int ANALOG_EVENT = 2;
const int BUTTON_EVENT = 1;
const std::string DEFAULTADDRESS="/dev/input/js0";

struct controllerEvent
{
  unsigned int time;  //time event happened
  short value;    //value of the analog stick/buttons
  unsigned char type; //type of event (button or analog)
  unsigned char number; //number of the button or stick
};

struct controllerState
{
  int buttons[NUM_BUTTONS];
  int stick[NUM_AXES*NUM_ANALOG_STICKS];
};

class Ctrlr
{
  public:
    Ctrlr();
    Ctrlr(const std::string);
    ~Ctrlr();
    bool openController(const std::string);
    void readControllerEvent();
    void writeControllerState();
    void update();
    
    int getStickvalue(int sticknum) const 
      {return contSt->stick[sticknum];}
    int getButton(int buttonnum) const 
      {return contSt->buttons[buttonnum];}

  private:
    controllerEvent * contEv; 
    controllerState * contSt;
    int controllerfd;   //controller file descriptor
    std::string joystickAddress;
};

#endif
