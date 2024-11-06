#include "tfmini_ros2/TFmini.hpp"

namespace benewake
{
  TFmini::TFmini(std::string _name, int _baudRate) :
    portName_(_name), baudRate_(_baudRate)
  {
    serial_ = open(_name.c_str(), O_RDWR);
    if(serial_ == -1){
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to open serial port %s!", _name.c_str());
      exit(0);
    } else {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Serial port %s opened successfully", _name.c_str());
    }

    struct termios options;
    if(tcgetattr(serial_, &options) != 0){
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Can't get serial port settings for %s!", _name.c_str());
      exit(0);
    }
    tcflush(serial_, TCIFLUSH);

    switch(_baudRate)
    {
      case 921600:
        cfsetispeed(&options, B921600);
        cfsetospeed(&options, B921600);
        break;
      case 576000:
        cfsetispeed(&options, B576000);
        cfsetospeed(&options, B576000);
        break;
      case 500000:
        cfsetispeed(&options, B500000);
        cfsetospeed(&options, B500000);
        break;
      case 460800:
        cfsetispeed(&options, B460800);
        cfsetospeed(&options, B460800);
        break;
      case 230400:
        cfsetispeed(&options, B230400);
        cfsetospeed(&options, B230400);
        break;
      case 115200:
        cfsetispeed(&options, B115200);
        cfsetospeed(&options, B115200);
        break;
      case 57600:
        cfsetispeed(&options, B57600);
        cfsetospeed(&options, B57600);
        break;
      case 38400:
        cfsetispeed(&options, B38400);
        cfsetospeed(&options, B38400);
        break;
      case 19200:
        cfsetispeed(&options, B19200);
        cfsetospeed(&options, B19200);
        break;
      case 9600:
        cfsetispeed(&options, B9600);
        cfsetospeed(&options, B9600);
        break;
      case 4800:
        cfsetispeed(&options, B4800);
        cfsetospeed(&options, B4800);
        break;
      default:
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Unsupported baud rate: %d", _baudRate);
        exit(0);
    }

    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_oflag &= ~OPOST;
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_iflag &= ~(INLCR | ICRNL | IGNCR);
    options.c_oflag &= ~(ONLCR | OCRNL);

    if(tcsetattr(serial_, TCSANOW, &options) != 0){
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Can't set serial port options for %s!", _name.c_str());
      exit(0);
    } else {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Serial port options set successfully for %s", _name.c_str());
    }
  }

  bool TFmini::readData(unsigned char *_buf, int _nRead)
  {
    int total = 0, ret = 0;

    while(total != _nRead){
      ret = read(serial_, _buf + total, (_nRead - total));
      if(ret < 0){
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to read data from serial port.");
        return false;
      }
      total += ret;
    }

    return true;
  }

  float TFmini::getDist()
  {
    while(true){
      if(readData(dataBuf, 2)){
        if(dataBuf[0] == 0x59 && dataBuf[1] == 0x59) {
          break;
        } else {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid data header received: %02x %02x", dataBuf[0], dataBuf[1]);
        }
      } else {
        return -1.0;
      }
    }

    if(readData(dataBuf, 7)){
      int sumCheck = (0x59 + 0x59) % 256;
      for(int i = 0; i < 6; i++){
        sumCheck = (sumCheck + dataBuf[i]) % 256;
      }

      if(sumCheck == dataBuf[6]){
        float distance = (float)(dataBuf[1] << 8 | dataBuf[0]) / 100.0;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Distance read: %f", distance);
        return distance;
      } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Checksum failed.");
        return 0.0;
      }
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to read complete data from sensor.");
      return -1.0;
    }
  }

  void TFmini::closePort()
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Closing serial port %s", portName_.c_str());
    close(serial_);
  }
}
