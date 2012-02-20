/*

Copyright (c) 2011, Markus Achtelik, ASL, ETH Zurich, Switzerland
You can contact the author at <markus dot achtelik at mavt dot ethz dot ch>

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
* Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.
* Neither the name of ETHZ-ASL nor the
names of its contributors may be used to endorse or promote products
derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ETHZ-ASL BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "serial_dump.h"
#include <ros/ros.h>

Comm::Comm() :
  rx_timeout_(uart_service_), rx_timeout_occurred_(false), rx_state_(1)
{

}

Comm::~Comm()
{
  this->close();
}

bool Comm::connect(SerialPortPtr & serial_port, const std::string & port, uint32_t baudrate)
{
  try
  {
    serial_port.reset(new SerialPort(uart_service_));
    serial_port->open(port);

    if(!configurePort(serial_port, &baudrate))
      return false;

    ROS_INFO_STREAM("INFO: opened serial port " << port << " with baudrate " << baudrate);
    return true;
  }
  catch (boost::system::system_error::exception e)
  {
    ROS_ERROR_STREAM("ERROR: could not open serial port " << port << " reason: " << e.what());
    return false;
  }
}

bool Comm::connect(const std::string & port_rx, uint32_t baudrate)
{
  bool success = false;
  port_rx_name_ = port_rx;

  success = connect(port_rx_, port_rx, baudrate);

  if (!success)
    return false;

  // run uart worker threads
  rxReadStart(1, 1000);
  uart_thread_[0] = boost::thread(boost::bind(&boost::asio::io_service::run, &uart_service_));
  uart_thread_[1] = boost::thread(boost::bind(&boost::asio::io_service::run, &uart_service_));

  return true;
}

void Comm::rxReadStart(uint32_t length, uint32_t timeout)
{
  boost::asio::async_read(*port_rx_, boost::asio::buffer(rx_buffer_, length),
                          boost::bind(&Comm::rxReadCallback, this, boost::asio::placeholders::error,
                                      boost::asio::placeholders::bytes_transferred));

  if (timeout != 0)
  {
    rx_timeout_.expires_from_now(boost::posix_time::milliseconds(timeout));
    rx_timeout_.async_wait(boost::bind(&Comm::rxTimeoutCallback, this, boost::asio::placeholders::error));
  }
}

bool Comm::configurePort(SerialPortPtr & serial_port, uint32_t * baudrate)
{
  int32_t baudrates[] = {9600, 14400, 19200, 38400, 57600, 115200, 230400, 460800, 921600};
  uint32_t best_baudrate = 57600;
  uint32_t min_diff = 1e6;

  for (uint32_t i = 0; i < sizeof(baudrates) / sizeof(uint32_t); i++)
  {
    uint32_t diff = abs(baudrates[i] - *baudrate);
    if (diff < min_diff)
    {
      min_diff = diff;
      best_baudrate = baudrates[i];
    }
  }

  if (best_baudrate != *baudrate)
    ROS_WARN("Unsupported baudrate, choosing closest supported baudrate (%d)", best_baudrate);

  *baudrate = best_baudrate;

  try
  {
    serial_port->set_option(boost::asio::serial_port_base::baud_rate(best_baudrate));
    serial_port->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
    serial_port->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    serial_port->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    serial_port->set_option(boost::asio::serial_port_base::character_size(8));
    return true;
  }
  catch (boost::system::system_error::exception e)
  {
    ROS_ERROR_STREAM("configuring serial port failed: " << e.what());
    return false;
  }
}

void Comm::close(){
  uart_service_.post(boost::bind(&boost::asio::deadline_timer::cancel, &rx_timeout_));
  uart_service_.post(boost::bind(&boost::asio::serial_port::close, port_rx_));

  uart_thread_[0].join();
  uart_thread_[1].join();
}

void Comm::rxTimeoutCallback(const boost::system::error_code & error)
{
  if (!error)
  {
    port_rx_->cancel();
    rx_timeout_occurred_ = true;
    printf("rx timeout\n");
  }
}

void Comm::rxReadCallback(const boost::system::error_code& error, size_t bytes_transferred)
{
  static uint32_t bytes_to_read = 1;

  static uint32_t err_cnt = 0;

  if (error)
  {
    if (error == boost::asio::error::operation_aborted)
    {
      // cancel came from timer? -> try again
      if (rx_timeout_occurred_)
      {
        rx_timeout_occurred_ = false;
        rx_state_ = 1;
        rxReadStart(1, 1000);
      }
      return;
    }

    std::cerr << "read error: " << error.message() << std::endl;
    if(err_cnt < 10){
      rx_state_ = 1;
      usleep(1e5);
      rxReadStart(1, 1000);
    }
    else{
      ROS_ERROR("Too many read errors");
      ros::shutdown();
    }
    err_cnt ++;
    return;
  }

  rx_timeout_.cancel();

  for(unsigned int i=0; i<bytes_transferred; i++)
    fprintf(stderr,"%02x ", rx_buffer_[i]);

  rxReadStart(bytes_to_read, 1000);
}

int main(int argc, char** argv){
  Comm comm;

  std::string port(argv[1]);
  int baudrate = atoi(argv[2]);

  comm.connect(port, baudrate);

  while(1)
    usleep(1000);
}
