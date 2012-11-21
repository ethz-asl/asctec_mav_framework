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

#ifndef __COMM_H__
#define __COMM_H__

#include <iostream>
#include <map>
#include <vector>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/asio.hpp>

#include <ros/ros.h>

// communication definitions
#include <asctec_hl_comm/HL_interface.h>

typedef boost::function<void(uint8_t*, int32_t)> CommCallbackType;
typedef std::map<uint8_t, CommCallbackType> CommCallbackMap;

class Comm;
typedef boost::shared_ptr<Comm> CommPtr;

/// Handles the communication between host computer and the HighLevel Processor (HLP) of the AscTec Autopilot board.
/**
 * Can connect to either one or two (e.g. for separate rx/tx wireless links) serial ports. 
 * The default baudrate can be changed up to 921600 baud. Messages can be sent with 
 * Comm::sendPacket or Comm::sendPacketAck . Messages are received by setting a callback function 
 * with Comm::registerCallback for the message id to receive.
 */
class Comm
{
  typedef boost::asio::serial_port SerialPort;
  typedef boost::shared_ptr<SerialPort> SerialPortPtr;

private:

  SerialPortPtr port_rx_;
  SerialPortPtr port_tx_;
  boost::asio::io_service uart_service_;
  boost::asio::deadline_timer rx_timeout_;
  bool rx_timeout_occurred_;
  boost::thread uart_thread_[2];

  std::string port_rx_name_;
  std::string port_tx_name_;

  uint32_t rx_packet_cnt_;
  uint32_t rx_packet_good_cnt_;

  uint8_t tx_buffer_[256];
  uint8_t rx_buffer_[256];
  uint32_t tx_bytes2send_;
  int rx_state_;
  std::vector<uint8_t> packet_acks_;
  boost::mutex tx_mutex_;
  const int TX_HEADER_SIZE;

  CommCallbackMap callbacks_;

  void rxReadStart(uint32_t length, uint32_t timeout = 0);
  void rxReadCallback(const boost::system::error_code& error, size_t bytes_transferred);
  void rxTimeoutCallback(const boost::system::error_code & error);
  bool configurePort(SerialPortPtr & serial_port, uint32_t *baudrate);
  bool connect(SerialPortPtr & serial_port, const std::string & port, uint32_t baudrate);

  void processPacketAck(uint8_t * buf, uint32_t length);
  bool validateChecksum(uint8_t * buf, uint32_t length, uint16_t checksum);

  bool waitForPacketAck(uint8_t ack_id, const double & timeout = 1.0);

  inline uint16_t crc_update(uint16_t crc, uint8_t data);
  uint16_t crc16(void* data, uint16_t cnt, uint16_t crc = 0xff);

  void serializePacket(uint8_t packet_id, const void * data, const int32_t & packet_size, uint8_t flag = 0);

public:
  Comm();
//  Comm(const std::string & port, uint32_t baudrate);
  ~Comm();

  /// connects to the specified serial port(s) with the given baudrate. The HLP sets it's baudrate automatically.
  /**
   * The port names can be equal, then it connects only to one serial port (most common operation).
   * It can also connect to different ports for rx and tx. This is useful when e.g. two wireless modules (one for rx, one for tx) such as XBee are used to connect
   * to the helicopter and bandwidth of one link is to low. rx/tx is seen from the host computer running the communication node. Any baudrate can be set. If it doesn't
   * match a standard baudrate, the closest standard baudrate is chosen. If the HLP doesn't "hear" anything from the node for ~10s, it will go into autobaud mode. During connection
   * setup, the node sends 'a' to the HLP, which will then configure its baudrate automatically.
   * @param port_rx port to use for rx
   * @param port_tx port to use for tx
   * @param baudrate baudrate to connect with. equal for both ports. The baudrate finally chosen is written to baudrate.
   * @return connection successful
   */
  bool connect(const std::string & port_rx, const std::string & port_tx, uint32_t baudrate);

  /// closes the serial port(s)
  void close();

  uint32_t getRxPackets()
  {
    return rx_packet_cnt_;
  }

  uint32_t getRxPacketsGood()
  {
    return rx_packet_good_cnt_;
  }

  /// sends a packet to the HLP
  template<typename T>
    bool sendPacket(uint8_t packet_id, const T & data)
    {
      serializePacket(packet_id, &data, sizeof(T));
//      return port_tx_->write_block(tx_buffer_, tx_bytes2send_);
      return tx_bytes2send_ == boost::asio::write(*port_tx_, boost::asio::buffer(tx_buffer_, tx_bytes2send_));
    }

  /// sends a packet to the HLP and blocks until it got acknowledged or until the timeout is reached
  template<typename T>
    bool sendPacketAck(uint8_t packet_id, const T & data, const double & timeout = 1.0)
    {
      // generate some "random" value to determine acknowledged packet afterwards. This is only meant for some configuration packets, so it should be random enough ;-)
      ros::Time time_start = ros::Time::now();
      uint8_t ack_id = (uint8_t)(time_start.toNSec() & 0xff) | HLI_COMM_ACK;
//      static uint8_t cnt = 1;
//      uint8_t ack_id = cnt | HLI_COMM_ACK;
//      cnt++;

      serializePacket(packet_id, &data, sizeof(T), ack_id);
      if (tx_bytes2send_ != boost::asio::write(*port_tx_, boost::asio::buffer(tx_buffer_, tx_bytes2send_)))
        return false;

      return waitForPacketAck(ack_id, timeout);
    }

  /// registers a class method that gets called when the packet specified by id was sent from the HLP
  /**
   * Currently, only one method/function can be registered per message id. The method has to be prototyped as follows (see e.g. Comm::processBaudrate):
   * \n \code void class::method(uint8_t *buffer, uint32_t length) \endcode
   * @param id the message id to subscribe to
   * @param cb_func the method that gets called
   * @param p_obj pointer to the object whose cb_func should get called
   */
  template<class T>
    void registerCallback(uint8_t id, void(T::*cb_func)(uint8_t*, uint32_t), T* p_obj)
    {
      if (callbacks_.find(id) != callbacks_.end())
        std::cerr<<"message "<< id << "is already registered"<<std::endl;
      else
        callbacks_[id] = boost::bind(cb_func, p_obj, _1, _2);
    }
void loopOnce(){uart_service_.run_one();};
};

#endif /* ASCTECCOMM_H_ */
