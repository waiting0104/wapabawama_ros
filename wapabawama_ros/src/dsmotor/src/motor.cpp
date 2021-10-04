/*
 * motor.cpp
 * Copyright (C) 2021 dephilia <dephilia@microlabpc3.pme.nthu.edu.tw>
 *
 * Distributed under terms of the MIT license.
 */

#include "motor.hpp"

/* #define V1 */
#define V2

System::System(std::string _dev_name, int _bdrate) {
  this->serialPtr = &this->serial;
  this->dev_name = _dev_name;
  this->bdrate = _bdrate;
  this->read_flag = false;
  memset(&this->tx_buf[0], 0, 128);
  this->tx_send_ok = false;
  /* this->rx_decoder = std::regex("M(.?) P(-?[0-9]\\d*(\\.\\d+)?)"); */
}

System::System() {
  this->serialPtr = &this->serial;
  this->read_flag = false;
  memset(&this->tx_buf[0], 0, 128);
  this->tx_send_ok = false;
  /* this->rx_decoder = std::regex("M(.?) P(-?[0-9]\\d*(\\.\\d+)?)"); */
}
void System::init(std::string _dev_name, int _bdrate) {
  this->dev_name = _dev_name;
  this->bdrate = _bdrate;
}
System::~System(){
  close_serial(this->serialPtr);
}
bool System::is_running() {
  return this->read_flag;
}
void System::run() {
  this->read_flag = true;
  /* init_serial(this->serialPtr, this->dev_name.c_str(), this->bdrate); */
  this->read_thread = std::thread{ &System::read_thread_fn, this };
  this->read_thread.detach();
  /* usleep(1000); */
  /* this->read_thread.join(); */
}
void System::stop() {
  this->read_flag = false;
}
void System::read_thread_fn() {
  int result = 0;

  // For decode
#ifdef V1
  int is_num = 0;
  char temp_int[8];
  int temp_value = 0;
  int index = 0;
  char last_cmd;
  int select_motor = 0;
#endif // V1

#ifdef V2
#define RXBUFSIZE 64
  char rx_buf[RXBUFSIZE] = {0};
  unsigned int rx_index = 0;
#endif

  /*
   * Init Serial
   */
  result = init_serial(this->serialPtr, this->dev_name.c_str(), this->bdrate);

  if (result) {
    this->read_flag = false;
    printf("Initial serial failed.\n");
  }

  /*
   * Receive msg
   */
  while(this->read_flag) {
    if ( read_once(this->serialPtr) ) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      continue;
    }
    // decoder
    for (char& cmd: this->serialPtr->read_buf) {

#ifdef V1
      switch (cmd) {
        case '0' ... '9': case '-':
          if (is_num){
            temp_int[index] = cmd;
            index++;
          }
          break;
        case 'M': // Select Motor
          last_cmd = cmd;
          is_num = 1;
          break;
        case 'P':
          last_cmd = cmd;
          is_num = 1;
          break;
        case ' ': case '\n': // End commend
          index = 0;
          is_num = 0;
          sscanf(temp_int, "%d", &temp_value); // Char to int
          memset(&temp_int[0], 0, sizeof(8)); // Clear number temp
          if ( last_cmd == 'M') {
            select_motor = temp_value;
          } else if ( last_cmd == 'P' ) {
            this->motor[select_motor].pos = temp_value;
          }
          break;
        default:
          break;
      }
#endif // V1

#ifdef V2
      switch (cmd) {
        case '\r': case '\n': // End of line
          sscanf(rx_buf, "M0 P%d M1 P%d M2 P%d L%lf R%lf", 
              &this->motor[0].pos,
              &this->motor[1].pos,
              &this->motor[2].pos,
              &this->dcmotor[0].speed,
              &this->dcmotor[1].speed);

          memset(rx_buf, 0, RXBUFSIZE);
          rx_index = 0;
          break;
        case 0:
          break;
        default:
          rx_buf[rx_index++] = cmd;
          break;
      }
#endif

    }

    /* 
     * Transmit message if queue not empty
     */
    if ( !this->tx_queue.empty() ) {
      write_serial(this->serialPtr, this->tx_queue.front().c_str());
      this->tx_queue.pop();
    }
  }
}
int System::sendCmd(std::string _cmd) {
  this->tx_queue.push(_cmd);
  return 0;
}
int System::sendCmd(char* _cmd) {
  this->tx_queue.push(std::string(_cmd));
  return 0;
}

int System::setPos(int m, int value) {
  char cmd[12];
  if (value >= 0 ){
    snprintf(cmd,12,"M%i P%i ", m, value);
  } else {
    snprintf(cmd,12,"M%i N%i ", m, -value);
  }
  this->tx_queue.push(std::string(cmd));
  return 0;
}
int System::setnPos(int m, int value) {
  value = value < MIN_POS ? MIN_POS : value;
  value = value > MAX_POS ? MAX_POS : value;
  char cmd[12];
  snprintf(cmd,12,"M%i P%i ", m, value);
  this->tx_queue.push(std::string(cmd));
  return 0;
}
int System::setGoSpeed(int value) {
  char cmd[12];
  snprintf(cmd, 12, "A%i ", value);
  this->tx_queue.push(std::string(cmd));
  return 0;
}
int System::resetSys() {
  this->tx_queue.push(std::string("Z "));
  return 0;
}
int System::resetOrig() {
  this->tx_queue.push(std::string("O "));
  return 0;
}
int System::resetMotor(int m) {
  char cmd[12];
  snprintf(cmd,12,"M%i R ", m);
  this->tx_queue.push(std::string(cmd));
  return 0;
}
void System::print() {
  printf("M0 P%d M1 P%d M2 P%d L%4.4lf R%4.4lf\r\n",
              this->motor[0].pos,
              this->motor[1].pos,
              this->motor[2].pos,
              this->dcmotor[0].speed,
              this->dcmotor[1].speed);
}

int System::getMotorPos(int m) {
  return this->motor[m].pos;
}
double System::getGantrySpd(int m) {
  return this->dcmotor[m].speed;
}
//Valve
int System::returnState(int v){
  return this->valve[v].state;
}
int System::setPWM(int v , int pwm){
  pwm = pwm < MIN_PWM ? MIN_PWM : pwm;
  pwm = pwm > MAX_PWM ? MAX_PWM : pwm;
  this->valve[v].pwm = pwm;
  this->valve[v].state = 1;r
  char cmd[12];
  snprintf(cmd, 12, "X%i V%i ", v, pwm);
  // std::cout <<"PWM is "<<pwm<<std::endl;
  this->tx_queue.push(std::string(cmd));
  return 0;
}
int System::closeValve(){
  char cmd[12]="Y ";
  this->valve[0].state=0;
  this->valve[1].state=0;
  this->valve[2].state=0;
  this->tx_queue.push(std::string(cmd));                                          
  return 0;
}
