#include "c_serial.h"

int setTTY(simp_serial *ss, int _br) {

  // Create new termios struc, we call it 'tty' for convention
  struct termios tty;

  // Read in existing settings, and handle any error
  if(tcgetattr(ss->serial_port, &tty) != 0) {
      printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
      return 1;
  }

  tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
  tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
  tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size 
  tty.c_cflag |= CS8; // 8 bits per byte (most common)
  tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
  tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO; // Disable echo
  tty.c_lflag &= ~ECHOE; // Disable erasure
  tty.c_lflag &= ~ECHONL; // Disable new-line echo
  tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

  tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
  tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
  // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
  // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

  tty.c_cc[VTIME] = 1;
  tty.c_cc[VMIN] = 0;

  speed_t br;
  switch (_br) {
    case 0:
      br = B0;
      break;
    case 50:
      br = B50;
      break;
    case 75:
      br = B75;
      break;
    case 110:
      br = B110;
      break;
    case 134:
      br = B134;
      break;
    case 150:
      br = B150;
      break;
    case 200:
      br = B200;
      break;
    case 300:
      br = B300;
      break;
    case 600:
      br = B600;
      break;
    case 1200:
      br = B1200;
      break;
    case 2400:
      br = B2400;
      break;
    case 4800:
      br = B4800;
      break;
    case 9600:
      br = B9600;
      break;
    case 19200:
      br = B19200;
      break;
    case 38400:
      br = B38400;
      break;
    case 57600:
      br = B57600;
      break;
    case 115200:
      br = B115200;
      break;
    case 230400:
      br = B230400;
      break;
    default:
      printf("Error from set baud rate to: %i\n", _br);
      return 1;
      break;
  }

  // Set in/out baud rate
  cfsetispeed(&tty, br);
  cfsetospeed(&tty, br);

  // Save tty settings, also checking for error
  if (tcsetattr(ss->serial_port, TCSANOW, &tty) != 0) {
      printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
      return 1;
  }
  return 0;
}

int init_serial(simp_serial* ss, const char* dev_name, int baudrate) {
  int result = 0;
  /* ss->serial_port = open(dev_name, O_RDWR); */
  ss->serial_port = open(dev_name, O_RDWR | O_NOCTTY );

  if (ss->serial_port < 0) {
      printf("Error %d opening %s: %s\n", errno, dev_name, strerror(errno));
      return 1;
  }
  // Acquire non-blocking exclusive lock
  if(flock(ss->serial_port, LOCK_EX | LOCK_NB) == -1) {
      printf("Serial port with file descriptor %i is already locked by another process.\n", ss->serial_port);
      return 1;
  }

  result |= setTTY(ss, baudrate);
  return result;
}

int close_serial(simp_serial* ss) {
  close(ss->serial_port);
  return 0;
}

int read_once(simp_serial* ss) {
  // Normally you wouldn't do this memset() call, but since we will just receive
  // ASCII data for this example, we'll set everything to 0 so we can
  // call printf() easily.
  memset(&ss->read_buf, '\0', sizeof(ss->read_buf));

  // Read bytes. The behaviour of read() (e.g. does it block?,
  // how long does it block for?) depends on the configuration
  // settings above, specifically VMIN and VTIME
  int num_bytes = read(ss->serial_port, &ss->read_buf, sizeof(ss->read_buf));

  // n is the number of bytes read. n may be 0 if no bytes were received, and can also be -1 to signal an error.
  if (num_bytes < 0) {
      printf("Error reading: %s", strerror(errno));
      return 1;
  }

  return 0;
}

int write_serial(simp_serial* ss, const char* msg) {
  // Write to serial port
  int n_written = write(ss->serial_port, msg, strlen(msg));
  /* printf("Write %i bytes: [%s]\n", n_written, msg); */
  usleep((strlen(msg) + 25) * 100);
  return 0;
}

