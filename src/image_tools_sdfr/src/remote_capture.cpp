#include <thread>
#include <chrono>

#include "base64.hpp"
#include "remote_capture.hpp"

#include "opencv2/imgcodecs.hpp"
#include "rclcpp/rclcpp.hpp"

using remote::RemoteCapture;

std::mutex m;

RemoteCapture::RemoteCapture() {
    initialized = false;
    webcam_down = false;
}

RemoteCapture::~RemoteCapture() {
    if (socket.is_open() && initialized) {
        RCLCPP_INFO(rclcpp::get_logger("remote_cap"), "Closing socket");
        socket.close();
    }
}

void RemoteCapture::set_ip(std::string ip_, int port_) {
    ip = ip_;
    port = port_;
    remote_endpoint = udp::endpoint(address::from_string(ip), port);

    RCLCPP_INFO(rclcpp::get_logger("remote_cap"), "Remote capture endpoint set to: %s:%d", ip.c_str(), port);
}

void RemoteCapture::initialize(size_t width, size_t height) {
    if (!socket.is_open()) {
        socket.open(boost::asio::ip::udp::v4());
        RCLCPP_INFO(rclcpp::get_logger("remote_cap"), "Opening socket");
    } else {
        RCLCPP_INFO(rclcpp::get_logger("remote_cap"), "Socket is already open, using this instead");
    }

    std::string identifier = "{\"width\": " + std::to_string((int)width) + ", \"height\": " + std::to_string((int)height) + "}";
    std::string enc_ident = base64::to_base64(identifier);

    boost::system::error_code err;
    try {
        socket.send_to(boost::asio::buffer(enc_ident.c_str(), enc_ident.size()), remote_endpoint, 0, err);
        RCLCPP_INFO(rclcpp::get_logger("remote_cap"), "Sent camera size to endpoint");
    } catch (const boost::system::system_error& ex) {
        RCLCPP_ERROR(rclcpp::get_logger("remote_cap"), "Sending failed: %s", ex.what());
        return;
    }

    initialized = true;
}

void RemoteCapture::add_handler() {
    RCLCPP_DEBUG(rclcpp::get_logger("remote_cap"), "Adding handler to queue");
    socket.async_receive_from(boost::asio::buffer(recv_buffer),
        remote_endpoint,
        boost::bind(&RemoteCapture::handle_receive, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
}

void RemoteCapture::handle_receive(const boost::system::error_code& error, size_t bytes_transferred) {
    if (error) {
        RCLCPP_ERROR(rclcpp::get_logger("remote_cap"), "Receive failed: %s", error.message().c_str());
        return;
    }

    RCLCPP_DEBUG(rclcpp::get_logger("remote_cap"), "Received %ld bytes", bytes_transferred);
    std::string received_str = std::string(recv_buffer.begin(), recv_buffer.begin()+bytes_transferred);

    if (received_str == std::string("EOS")) {
        RCLCPP_INFO(rclcpp::get_logger("remote_cap"), "Received EndOfStream ('EOS'). Shutting down...");
        webcam_down = true;
        return;
    } else if (received_str == std::string("EOF")) {
        RCLCPP_DEBUG(rclcpp::get_logger("remote_cap"), "Received EndOfFrame ('EOF'). Processing frame...");
        auto bytes = base64::decode_into<std::vector<char>>(frame_buffer);
        m.lock();
        if (!bytes.empty()) {
            remote_buf = cv::imdecode(bytes, cv::IMREAD_COLOR);
        }
        m.unlock();
        frame_buffer = "";
    } else {
        frame_buffer += received_str;
        add_handler();
    }
}

void RemoteCapture::run_receiver() {
    using namespace std::chrono_literals;
    add_handler();

    RCLCPP_DEBUG(rclcpp::get_logger("remote_cap"), "Start receiving");

    std::size_t handlers_executed = io_service.run_for(IO_SERVICE_TIMEOUT * 1s);
    if (handlers_executed == 0) {
        RCLCPP_WARN(rclcpp::get_logger("remote_cap"), "No handlers were executed within timeout period of %d seconds.", IO_SERVICE_TIMEOUT);
        RCLCPP_WARN(rclcpp::get_logger("remote_cap"), "Assuming connection has been lost. Shutting down...");
        webcam_down = true;
    }

    RCLCPP_DEBUG(rclcpp::get_logger("remote_cap"), "Stop receiving");
}

cv::Mat & RemoteCapture::get_frame() {
    if (io_service.stopped()) {
        RCLCPP_DEBUG(rclcpp::get_logger("remote_cap"), "Restarting IO service");
        io_service.restart();
    }

    RCLCPP_DEBUG(rclcpp::get_logger("remote_cap"), "Starting thread");
    std::thread r(&RemoteCapture::run_receiver, this);
    r.join();

    return remote_buf;
}