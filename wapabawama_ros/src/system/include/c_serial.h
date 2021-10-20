/*
 * c_serial.h
 * Copyright (C) 2021 dephilia <dephilia@microlabpc3.pme.nthu.edu.tw>
 *
 * Distributed under terms of the MIT license.
 */

#ifndef C_SERIAL_H
#define C_SERIAL_H

#ifdef __cplusplus
extern "C" {
#endif

// C library headers
#include <stdio.h>
#include <string.h>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

typedef struct _simp_serial {
  int serial_port;
  char read_buf [256];
} simp_serial;

int setTTY(simp_serial *ss, int _br) ;
int init_serial(simp_serial* ss, const char* dev_name, int baudrate) ;
int close_serial(simp_serial* ss) ;
int read_once(simp_serial* ss) ;
int write_serial(simp_serial* ss, const char* msg) ;

#ifdef __cplusplus
}
#endif

#endif /* !C_SERIAL_H */
