/*
 * main.cpp
 * Copyright (C) 2021 dephilia <dephilia@microlabpc3.pme.nthu.edu.tw>
 *
 * Distributed under terms of the MIT license.
 */

#include "system.hpp"
#include "cmath"
#include <iostream>

System mt("/dev/ttyTHS0",115200);


int main() {
  float i = 0;

  mt.run();
  /* mt.resetSys(); */
  /* mt.setPos(1, 100); */
  /* mt.setPos(2, 100); */
  /* mt.sendCmd("Z M0 P100 M1 P200 M2 P300 "); */
  // while(mt.is_running()) {
  //   mt.setPos(0, 1000*sin(i));
  //   mt.setPos(1, 1000*sin(i));
  //   mt.setPos(2, 1000*sin(i));
  //   i+=0.05;
  //   std::cout << mt.getMotorPos(0) << " " << mt.getMotorPos(1) << " " << mt.getMotorPos(2)  << std::endl;
  //   usleep(1000);
  // }
  // mt.stop();
  std::cout<<Flow2PWM(15.7);
  return 0; // success
}


