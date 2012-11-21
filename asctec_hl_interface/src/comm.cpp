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

#include "comm.h"
#include <ros/ros.h>

Comm::Comm() :
  rx_timeout_(uart_service_), rx_timeout_occurred_(false), rx_state_(1), TX_HEADER_SIZE(6)// 3 preamble + size + type + flag
{
  rx_packet_cnt_ = 0;
  rx_packet_good_cnt_ = 0;
  registerCallback(HLI_PACKET_ID_ACK, &Comm::processPacketAck, this);
  tx_buffer_[0] = 'a';
  tx_buffer_[1] = '*';
  tx_buffer_[2] = '>';
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
  catch (boost::system::system_error::exception &e)
  {
    ROS_ERROR_STREAM("ERROR: could not open serial port " << port << " reason: " << e.what());
    return false;
  }
  return false;
}

bool Comm::connect(const std::string & port_rx, const std::string & port_tx, uint32_t baudrate)
{
  bool success = false;
  port_tx_name_ = port_tx;
  port_rx_name_ = port_rx;

  if (port_rx == port_tx)
  {
    success = connect(port_rx_, port_rx, baudrate);
    port_tx_ = port_rx_;
  }
  else
  {
    if (connect(port_rx_, port_rx, baudrate))
    {
      success = connect(port_tx_, port_tx, baudrate);
    }
  }

  if (!success)
    return false;

  // write packet that configures autobaud. LSB needs to be 1, LSB+1 needs to be 0
  char buf = 'a';
  boost::asio::write(*port_tx_, boost::asio::buffer(&buf, 1));

  // run uart worker threads
  rxReadStart(1, 1000);
  uart_thread_[0] = boost::thread(boost::bind(&boost::asio::io_service::run, &uart_service_));
  uart_thread_[1] = boost::thread(boost::bind(&boost::asio::io_service::run, &uart_service_));

  // this doesn't do anything on the HLP right now, just a dummy package to check successful connection
  HLI_BAUDRATE dummy;
  ROS_INFO("configured serial port(s), checking connection ... ");
  for (int i = 0; i < 5; i++)
  {
    if (sendPacketAck(HLI_PACKET_ID_BAUDRATE, dummy, 0.5))
    {
      ROS_INFO("ok");
      return true;
    }
  }
  ROS_ERROR("failed");
  return false;
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
  catch (boost::system::system_error::exception &e)
  {
    ROS_ERROR_STREAM("configuring serial port failed: " << e.what());
    return false;
  }
  return false;
}

void Comm::close(){
  uart_service_.post(boost::bind(&boost::asio::deadline_timer::cancel, &rx_timeout_));
  uart_service_.post(boost::bind(&boost::asio::serial_port::close, port_rx_));
  if (port_rx_name_ != port_tx_name_)
    uart_service_.post(boost::bind(&boost::asio::serial_port::close, port_tx_));

  uart_thread_[0].join();
  uart_thread_[1].join();
}

bool Comm::validateChecksum(uint8_t * data, uint32_t length, uint16_t checksum)
{
  return crc16(data, length) == checksum;
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
  const int CHECKSUM_SIZE = 2;

  static uint8_t packet_type = 0;
  static uint8_t packet_size = 0;

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

  if (rx_state_ == 1)
  {
    packet_type = 0;
    packet_size = 0;
    if (rx_buffer_[0] == 0xff)
      rx_state_++;
  }
  else if (rx_state_ == 2)
  {
    if (rx_buffer_[0] == 0x09/*recvPreamble_[1]*/)
    {
      rx_state_ = 3;
      bytes_to_read = 2;
    }
    else
    {
      rx_state_ = 1;
      bytes_to_read = 1;
    }
  }
  else if (rx_state_ == 3)
  {
    // read packet size
    packet_size = rx_buffer_[0];
    // determine packet type
    packet_type = rx_buffer_[1];
    bytes_to_read = packet_size + CHECKSUM_SIZE; // read packet and checksum at once
    rx_state_ = 4;
  }
  else if (rx_state_ == 4)
  {
    // message body is read - validate checksum now
    uint16_t checksum_received = 0;
    uint16_t checksum_computed = 0;
    rx_packet_cnt_++;
    checksum_received = *((uint16_t*)(&rx_buffer_[packet_size]));
    checksum_computed = crc16(&packet_type, sizeof(packet_type));
    checksum_computed = crc16(rx_buffer_, packet_size, checksum_computed);
    if (checksum_received != checksum_computed)
    {
      std::cerr << "checksum error for packet " << (int)packet_type << " ,resyncing " << std::endl;
      bytes_to_read = 1;
      rx_state_ = 1;
    }
    else
    {
      rx_packet_good_cnt_++;
      // look for registered callback and call it if available
      CommCallbackMap::iterator callback_it;
      callback_it = callbacks_.find(packet_type);
      if (callback_it != callbacks_.end())
        callback_it->second(rx_buffer_, packet_size);

      // everything is successfully finished and synchronized, read header in one step now
      bytes_to_read = 4;
      rx_state_ = 5;
    }
  }
  else if (rx_state_ == 5)
  {
    // read packet size
    packet_size = rx_buffer_[2];
    // determine packet type
    packet_type = rx_buffer_[3];
    bytes_to_read = packet_size + CHECKSUM_SIZE; // read packet and checksum at once
    rx_state_ = 4;
  }

  rxReadStart(bytes_to_read, 1000);
}

void Comm::serializePacket(uint8_t packet_id, const void * data, const int32_t & packet_size, uint8_t flag)
{
  assert((packet_size + TX_HEADER_SIZE) <= 256);
  tx_bytes2send_ = TX_HEADER_SIZE + packet_size + sizeof(uint16_t);
  tx_buffer_[3] = packet_size;
  tx_buffer_[4] = packet_id;
  tx_buffer_[5] = flag;

  memcpy(&tx_buffer_[6], data, packet_size);
  uint16_t chk = crc16(&tx_buffer_[4], packet_size + 2); // datasize + descriptor + flag
  tx_buffer_[packet_size + TX_HEADER_SIZE] = (uint8_t)(chk & 0xff);
  tx_buffer_[packet_size + TX_HEADER_SIZE + 1] = (uint8_t)(chk >> 8);
}

void Comm::processPacketAck(uint8_t * buf, uint32_t length)
{
  HLI_ACK *packet_ack = (HLI_ACK*)buf;
  boost::mutex::scoped_lock lock(tx_mutex_);
  packet_acks_.push_back(packet_ack->ack_packet);
//  ROS_INFO("packet ack %d", packet_ack->ack_packet);
}

bool Comm::waitForPacketAck(uint8_t ack_id, const double & timeout)
{
  int sleeptime = 1e5; // 100ms
  int wait_count = static_cast<int> (timeout * 1e6 / sleeptime);

  for (int i = 0; i < wait_count; i++)
  {
    boost::mutex::scoped_lock lock(tx_mutex_);
    for (std::vector<uint8_t>::iterator it = packet_acks_.begin(); it < packet_acks_.end(); it++)
    {
      if (*it == ack_id)
      {
        packet_acks_.erase(it);
        return true;
      }
    }
    lock.unlock();
    usleep(sleeptime);
  }
  std::cerr << "waiting for acknowledged packet timed out" << std::endl;
  return false;
}

uint16_t Comm::crc_update(uint16_t crc, uint8_t data)
{
  data ^= (crc & 0xff);
  data ^= data << 4;

  return ((((uint16_t)data << 8) | ((crc >> 8) & 0xff)) ^ (uint8_t)(data >> 4) ^ ((uint16_t)data << 3));
}

uint16_t Comm::crc16(void* data, uint16_t cnt, uint16_t crc)
{
  uint8_t * ptr = (uint8_t *)data;
  int i;

  for (i = 0; i < cnt; i++)
  {
    crc = crc_update(crc, *ptr);
    ptr++;
  }
  return crc;
}
