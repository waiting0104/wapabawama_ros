/*
 * main.cpp
 * Copyright (C) 2021 dephilia <dephilia@microlabpc3.pme.nthu.edu.tw>
 *
 * Distributed under terms of the MIT license.
 */
#include "ros/ros.h"
#include "system.hpp"
#include <iostream>


int main(int argc, char **argv) {
  ros::init(argc, argv, "pwm");
  std::cout<<Flow2PWM(14.8)<<std::endl;
  std::cout<<Flow2PWM(21.6)<<std::endl;
  std::cout<<Flow2PWM(51.6)<<std::endl;
  std::cout<<Flow2PWM(41.6)<<std::endl;
  return 0; // success
}


