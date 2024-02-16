#ifndef REMOTE_CAPTURE_HPP_
#define REMOTE_CAPTURE_HPP_

#include <vector>
#include <string>
#include <iostream>
#include <mutex>

#include "boost/bind/bind.hpp"
#include "boost/asio.hpp"
#include "boost/array.hpp"
#include "opencv2/core/mat.hpp"

#include "image_tools_sdfr/visibility_control.h"

namespace remote
{

#define BUFFER_SIZE 65536
#define CHUNK_SIZE 9216
#define IO_SERVICE_TIMEOUT 7

class RemoteCapture
{
public:
  IMAGE_TOOLS_SDFR_PUBLIC
  RemoteCapture();
  IMAGE_TOOLS_SDFR_PUBLIC
  ~RemoteCapture();
  IMAGE_TOOLS_SDFR_PUBLIC
  cv::Mat & get_frame();
  //cv::Mat & get_frame(size_t width, size_t height);

  IMAGE_TOOLS_SDFR_PUBLIC
  void set_ip(std::string ip_, int port_);
  IMAGE_TOOLS_SDFR_PUBLIC
  void initialize(size_t width, size_t height);
  IMAGE_TOOLS_SDFR_PUBLIC
  void shutdown() {webcam_down = true;};
  IMAGE_TOOLS_SDFR_PUBLIC
  bool webcam_is_down() { return webcam_down;};

private:
  using udp     = boost::asio::ip::udp;
  using address = boost::asio::ip::address;

  IMAGE_TOOLS_SDFR_LOCAL
  void add_handler();
  IMAGE_TOOLS_SDFR_LOCAL
  void handle_receive(const boost::system::error_code& error, size_t bytes_transferred);
  IMAGE_TOOLS_SDFR_LOCAL
  void run_receiver();

  boost::asio::io_service io_service;
  udp::socket socket{io_service, udp::v4()};
  boost::array<char, BUFFER_SIZE> recv_buffer;
  std::string frame_buffer;
  udp::endpoint remote_endpoint;
  cv::Mat remote_buf;
  std::string ip;
  int port;
  bool webcam_down;
  bool initialized;
};


}  // namespace remote

#endif  // REMOTE_CAPTURE_HPP_