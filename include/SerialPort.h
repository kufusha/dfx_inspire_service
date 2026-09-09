#ifndef SERIAL_PORT_H
#define SERIAL_PORT_H

#include <termios.h>
#include <sys/select.h>
#include <string>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/serial.h>
#include <unistd.h>
#include <iostream>
#include <memory>
#include <chrono>
#include <queue>

class SerialPort
{
public:
  using SharedPtr = std::shared_ptr<SerialPort>;

  SerialPort(std::string port, speed_t baudrate, int timeout_ms = 2)
  {
    set_timeout(timeout_ms);
    Init(port, baudrate);
  }

  ~SerialPort()
  {
    close(fd_);
  }

  ssize_t send(const uint8_t* data, size_t len)
  {
    ssize_t ret = ::write(fd_, data, len);
    return ret;
  }

  ssize_t recv(uint8_t* data, size_t len)
  {
    const auto timeout_duration = std::chrono::seconds(timeout_.tv_sec) +
        std::chrono::microseconds(timeout_.tv_usec);
    const auto deadline = std::chrono::steady_clock::now() + timeout_duration;
    size_t total = 0;

    while (total < len)
    {
      const auto now = std::chrono::steady_clock::now();
      if (now >= deadline)
        break;
      const auto remaining =
          std::chrono::duration_cast<std::chrono::microseconds>(deadline - now);
      timeval timeout;
      timeout.tv_sec = remaining.count() / 1000000;
      timeout.tv_usec = remaining.count() % 1000000;

      FD_ZERO(&rSet_);
      FD_SET(fd_, &rSet_);
      const int ready = select(fd_ + 1, &rSet_, NULL, NULL, &timeout);
      if (ready <= 0)
        break;

      const ssize_t received = ::read(fd_, data + total, len - total);
      if (received <= 0)
        break;
      total += static_cast<size_t>(received);
    }
    return static_cast<ssize_t>(total);
  }

  void set_timeout(int timeout_ms)
  {
    timeout_.tv_sec = timeout_ms / 1000;
    timeout_.tv_usec = (timeout_ms % 1000) * 1000;
  }

private:
  void Init(std::string port, speed_t baudrate)
  {
    int ret;
    // Open serial port
    fd_ = open(port.c_str(), O_RDWR | O_NOCTTY);
    if (fd_ < 0)
    {
      printf("Open serial port %s failed\n", port.c_str());
      exit(-1);
    }

    // Set attributes
    struct termios option;
    memset(&option, 0, sizeof(option));
    ret = tcgetattr(fd_, &option);

    option.c_oflag = 0;
    option.c_lflag = 0;
    option.c_iflag = 0;

    cfsetispeed(&option, baudrate);
    cfsetospeed(&option, baudrate);

    option.c_cflag &= ~CSIZE;
    option.c_cflag |= CS8; // 8
    option.c_cflag &= ~PARENB; // no parity
    option.c_iflag &= ~INPCK; // no parity
    option.c_cflag &= ~CSTOPB; // 1 stop bit

    option.c_cc[VTIME] = 0;
    option.c_cc[VMIN] = 0;
    option.c_lflag |= CBAUDEX;

    ret = tcflush(fd_, TCIFLUSH);
    ret = tcsetattr(fd_, TCSANOW, &option);
  }

  int fd_;
	fd_set rSet_;
  timeval timeout_;

  std::queue<uint8_t> recv_queue;
  std::array<uint8_t, 1024> recv_buf;
};

#endif // SERIAL_PORT_H
