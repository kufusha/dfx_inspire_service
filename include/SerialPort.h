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
#include <deque>

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
    FD_ZERO(&rSet_);
    FD_SET(fd_, &rSet_);
    // select(2) may modify the timeval. Use a fresh copy for every call.
    timeval timeout = timeout_;
    ssize_t received = 0;
    switch (select(fd_ + 1, &rSet_, NULL, NULL, &timeout))
    {
    case -1:
    case 0:
      break;
    default:
      received = ::read(fd_, data, len);
      break;
    }
    return received;
  }

  ssize_t recvFrame(uint8_t* data, size_t expected_len, int timeout_ms = 50)
  {
    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline)
    {
      while (recv_queue.size() >= 2 &&
             !((recv_queue[0] == 0x90 && recv_queue[1] == 0xEB) ||
               (recv_queue[0] == 0xEB && recv_queue[1] == 0x90)))
        recv_queue.pop_front();

      if (recv_queue.size() >= 4)
      {
        const size_t frame_len = static_cast<size_t>(recv_queue[3]) + 5;
        if (frame_len < 9 || frame_len > recv_buf.size())
        {
          recv_queue.pop_front();
          continue;
        }
        if (recv_queue.size() >= frame_len)
        {
          uint8_t checksum = 0;
          for (size_t i = 2; i + 1 < frame_len; ++i)
            checksum += recv_queue[i];
          const bool valid = checksum == recv_queue[frame_len - 1];
          if (valid && frame_len == expected_len)
          {
            for (size_t i = 0; i < frame_len; ++i)
            {
              data[i] = recv_queue.front();
              recv_queue.pop_front();
            }
            return static_cast<ssize_t>(frame_len);
          }
          for (size_t i = 0; i < frame_len; ++i)
            recv_queue.pop_front();
          continue;
        }
      }

      const auto now = std::chrono::steady_clock::now();
      const auto remaining =
          std::chrono::duration_cast<std::chrono::microseconds>(deadline - now);
      timeval timeout;
      timeout.tv_sec = remaining.count() / 1000000;
      timeout.tv_usec = remaining.count() % 1000000;
      FD_ZERO(&rSet_);
      FD_SET(fd_, &rSet_);
      if (select(fd_ + 1, &rSet_, NULL, NULL, &timeout) <= 0)
        break;
      const ssize_t received = ::read(fd_, recv_buf.data(), recv_buf.size());
      if (received <= 0)
        break;
      recv_queue.insert(
          recv_queue.end(), recv_buf.begin(), recv_buf.begin() + received);
    }
    return 0;
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

  std::deque<uint8_t> recv_queue;
  std::array<uint8_t, 1024> recv_buf;
};

#endif // SERIAL_PORT_H
