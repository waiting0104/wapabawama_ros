/*
 * motor.h
 * Copyright (C) 2021 dephilia <dephilia@microlabpc3.pme.nthu.edu.tw>
 *
 * Distributed under terms of the MIT license.
 */

#ifndef MOTOR_HPP
#define MOTOR_HPP
#define MAX_PWM 100
#define MIN_PWM 0
#define MOTORNUM 3
#define DCMOTORNUM 2
#define VALVENUM 3
#define MAX_POS 11000
#define MIN_POS 0

#include <thread>
#include <atomic>
#include <mutex>
#include <queue>

#include "c_serial.h"

class Smotor {
  public:
    double speed;
    int pos;
};
class Valve{
  public:
    int pwm;
    int state;
};
class System {
  public:
    System(std::string, int);
    System();
    void init(std::string, int);
    ~System();
    void run();
    void stop();
    int sendCmd(std::string);
    int sendCmd(char*);
    int setPos(int,int);
    int setnPos(int,int); // Safe
    int setGoSpeed(int);
    int resetSys();
    int resetOrig();
    int resetMotor(int);
    void print();
    bool is_running();
    int getMotorPos(int);
    double getGantrySpd(int);
    //valve function
    int setPWM(int ,int);
    int returnState(int);
    int closeValve();
  private:
    bool read_flag;
    void read_thread_fn();
    Smotor motor[MOTORNUM];
    Smotor dcmotor[DCMOTORNUM];
    Valve  valve[VALVENUM];                                                      
    std::thread read_thread;
    simp_serial serial;
    simp_serial* serialPtr;
    int bdrate;
    std::string dev_name;
    char tx_buf[128];
    std::atomic<bool> tx_send_ok;
    std::mutex tx_mutex;
    std::queue<std::string> tx_queue;

};


#endif /* !MOTOR_HPP */
