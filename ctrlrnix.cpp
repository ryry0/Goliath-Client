/*
  Author:Ryan - David Reyes
*/
#include "ctrlrnix.h"
#include <iostream>
#include <iomanip>
#include <string>
#include <csignal>  //signal handling
#include <unistd.h> //file descriptor manipulation
#include <stdlib.h> //exit();
#include <fcntl.h>  //file descriptor manipulation
#include <pthread.h>

Ctrlr::Ctrlr()
{
  contEv = new controllerEvent;
  contSt = new controllerState;
}

Ctrlr::Ctrlr(const std::string joyAddressArg) 
{
  joystickAddress = joyAddressArg;
  contEv = new controllerEvent;
  contSt = new controllerState;
  openController(joystickAddress);
}

Ctrlr::~Ctrlr()
{
  delete contEv;
  delete contSt;
  close (controllerfd);
}

void Ctrlr::readControllerEvent()
{
  read(controllerfd, contEv, sizeof(contEv));
}

void Ctrlr::writeControllerState()
{
  if(contEv->type == ANALOG_EVENT)
  {
    contSt->stick[contEv->number] = contEv->value;
  }
  else //if it is button event
  {
    contSt->buttons[contEv->number] = contEv->value;
  }
}

void Ctrlr::update()
{
  readControllerEvent();
  writeControllerState();
}

bool Ctrlr::openController(std::string joyAddressArg)
{
  controllerfd = open(joyAddressArg.c_str(), O_RDONLY | O_NONBLOCK);
  if (controllerfd < 0)
    return false;

  else
    return true;
}
