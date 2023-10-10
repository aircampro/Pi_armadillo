//
// Boost Serial communicator class - this speaks to gimbal
//
#include <iostream>
#include <boost/asio/serial_port.hpp>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include "blocking_reader.h"
#include <chrono>
#include <thread>
#include <boost/asio/steady_timer.hpp>
#include <boost/thread.hpp>
#include <exception>

#ifndef __serial_class_boost_
#define __serial_class_boost_

using namespace std;
//using namespace boost;

#define GIMBAL_USB_CONNECTION "/dev/ttyACM1" // may be "/dev/ttyACM1" "/dev/ttyACM0" "/dev/ttyUSB0" "/dev/ttyAMA0"
#define GREMSY_GIMBAL_BAUD "57600"

// if you want to include the timeout reader from ridgesolutions include this below
//
#include "blocking_reader.h"

// boost asio serial port communicator class
//
class serialport
{
public:
    typedef std::vector<std::uint8_t> frame_type;
private:
    bool isFinished;
    bool isTimeout;
    unsigned int timeout_sec;
    std::string port;
    std::string speed;
    std::size_t transferred;
    frame_type recvdata;
    boost::asio::io_service& io;
    //boost::asio::serial_port sp(io, GIMBAL_USB_CONNECTION);
    boost::asio::serial_port sp;
    boost::asio::steady_timer timer;
    boost::system::error_code err;
    boost::asio::streambuf sbuf;

    void WriteHandler(const boost::system::error_code& ec, std::size_t bytes_transferred)
    {
        isFinished = true;
        if (ec == boost::asio::error::operation_aborted)
        {
            isTimeout = true;
        }
        else
        {
            isTimeout = false;
            timer.cancel();
        }
        return;
    };
    void ReadHandler(const boost::system::error_code& ec, std::size_t bytes_transferred)
    {
        transferred = bytes_transferred;
        isFinished = true;
        if (ec == boost::asio::error::operation_aborted)
        {
            isTimeout = true;
        }
        else
        {
            isTimeout = false;
            timer.cancel();

            const uint8_t* buffer_ptr = boost::asio::buffer_cast<const uint8_t*>(sbuf.data());
            recvdata.assign(buffer_ptr, buffer_ptr + bytes_transferred);
            sbuf.consume(bytes_transferred);
        }
        return;
    };
    void on_timer(const boost::system::error_code& ec)
    {
        sp.cancel();
    };
public:
    serialport(boost::asio::io_service& ios)
        :sp(ios), io(ios), timer(ios), isFinished(false), isTimeout(false), timeout_sec(5), speed("57600"), port(GIMBAL_USB_CONNECTION)
    {};
    void setPort(const std::string PORT)
    {
        port = PORT;
        return;
    }
    std::string getPort()
    {
        return port;
    }
    void setPort_to_gimbal()
    {
        port = GIMBAL_USB_CONNECTION;
        return;
    }
    void setSpeed(const std::string SPEED)
    {
        speed = SPEED;
        return;
    }
    void reset_serial_set_up() {
        sp.set_option(boost::asio::serial_port::baud_rate(57600));
        sp.set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
        sp.set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
        sp.set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
        sp.set_option(boost::asio::serial_port::character_size(8));
        std::this_thread::sleep_for(std::chrono::seconds(5));
    }
    const char* read_serial_until_eol() {
        boost::asio::streambuf response_buf;
        boost::asio::read_until(sp, response_buf, '\n');
        return boost::asio::buffer_cast<const char*>(response_buf.data());
    };
    const char* to_char_star() {
        return boost::asio::buffer_cast<const char*>(sbuf.data());
    };
    void write_string_serial(std::string str_to_write) {
        boost::asio::write(sp, boost::asio::buffer(str_to_write, str_to_write.length()));
        io.reset();
        io.run();
        return;
    };
    bool open(void)
    {
        if (sp.is_open())  return false;
        else
        {
            sp.open(port, err);
            if (err)
            {
                return false;
            }
            else
            {
                unsigned int baudrate = 0;
                try
                {
                    std::stringstream(speed) >> baudrate;
                }
                catch (std::exception& e)
                {
                    baudrate = 57600;                                  // default baud rate to this if you made an error
                }
                sp.set_option(boost::asio::serial_port_base::baud_rate(baudrate), err);
                sp.set_option(boost::asio::serial_port::character_size(8));
                sp.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none), err);
                sp.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none), err);
                sp.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one), err);
                std::this_thread::sleep_for(std::chrono::seconds(5));
            }
            return true;
        }
    };
    bool close(void)
    {
        if (!sp.is_open()) return false;
        else
        {
            sp.cancel(err);
            if (err)
                return false;
            else
            {
                sp.close(err);
                if (err)
                    return false;
                else
                    return true;
            }
        }
    };
    void write(const frame_type& data)
    {
        boost::asio::async_write(
            sp,
            boost::asio::buffer(data, data.size()),
            boost::asio::transfer_all(),
            boost::bind(&serialport::WriteHandler, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred)
        );
        timer.expires_from_now(std::chrono::seconds(timeout_sec));
        timer.async_wait(boost::bind(&serialport::on_timer, this, boost::asio::placeholders::error));
        isFinished = false;

        io.reset();
        io.run();

        return;
    };
    frame_type convert_char_star_to_frame(char* pData, std::size_t pd_len) {
        frame_type hoge(pData, pData + pd_len);
        return hoge;
    };
    frame_type convert_byte_star_to_frame(std::uint8_t* pData, std::size_t pd_len) {
        frame_type hoge(pData, pData + pd_len);
        return hoge;
    };
    frame_type convert_string_to_frame(std::string ss) {
        char* cstr = new char[ss.size() + 1];
        std::char_traits<char>::copy(cstr, ss.c_str(), ss.size() + 1);
        frame_type hoge(cstr, cstr + ss.length());
        return hoge;
    };
    std::string convert_frame_to_string(frame_type char_array1) {
        std::string output_string(char_array1.begin(), char_array1.end());
        return output_string;
    };
    char* convert_frame_to_char_star(frame_type char_array1) {
        std::string output_string(char_array1.begin(), char_array1.end());
        char* cstr = new char[output_string.size() + 1];                                                                      // allocate char* size of string
        std::char_traits<char>::copy(cstr, output_string.c_str(), output_string.size() + 1);                                  // copy the string to the char*
        return cstr;
    };
#if defined(__serial_block_reader_)                                                                                                 // if we have the ridge solutions library allow its use
    std::string read_response_serial_br() {

        // A blocking reader for this port that
        // will time out a read after time_out_ms e.g. 500 milliseconds.
        blocking_reader reader(sp, timeout_sec, io);

        char c;
        std::string rsp;

        // read from the serial port until we get a
        // \n or until a read times-out time_out_ms e.g. (500ms)
        io.reset();
        while (reader.read_char(c) && c != '\n') {
            rsp += c;
        }

        if (c != '\n') {
            // it must have timed out.
            throw std::runtime_error("Read timed out!");
        }

        io.run();

        return rsp;
    };
#endif
    void write_len(const frame_type& data, const std::size_t length)
    {
        boost::asio::async_write(
            sp,
            boost::asio::buffer(data, data.size()),
            boost::asio::transfer_exactly(((length < data.size()) ? length : data.size())),
            boost::bind(&serialport::WriteHandler, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred)
        );
        timer.expires_from_now(std::chrono::seconds(timeout_sec));
        timer.async_wait(boost::bind(&serialport::on_timer, this, boost::asio::placeholders::error));
        isFinished = false;

        io.reset();
        io.run();

        return;
    };
    void write_char_array(const char* px)
    {
        boost::asio::async_write(
            sp,
            boost::asio::buffer(px, sizeof(px)),
            boost::asio::transfer_exactly((sizeof(px))),
            boost::bind(&serialport::WriteHandler, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred)
        );
        timer.expires_from_now(std::chrono::seconds(timeout_sec));
        timer.async_wait(boost::bind(&serialport::on_timer, this, boost::asio::placeholders::error));
        isFinished = false;

        io.reset();
        io.run();

        return;
    };
    void write_byte_array(const std::uint8_t* px)
    {
        boost::asio::async_write(
            sp,
            boost::asio::buffer(px, sizeof(px)),
            boost::asio::transfer_exactly((sizeof(px))),
            boost::bind(&serialport::WriteHandler, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred)
        );
        timer.expires_from_now(std::chrono::seconds(timeout_sec));
        timer.async_wait(boost::bind(&serialport::on_timer, this, boost::asio::placeholders::error));
        isFinished = false;

        io.reset();
        io.run();

        return;
    };
    void read(void)
    {
        boost::asio::async_read(
            sp,
            sbuf,
            boost::asio::transfer_at_least(4),
            boost::bind(&serialport::ReadHandler, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred)
        );
        timer.expires_from_now(std::chrono::seconds(timeout_sec));
        timer.async_wait(boost::bind(&serialport::on_timer, this, boost::asio::placeholders::error));
        isFinished = false;

        io.reset();
        io.run();

        return;
    }
    void read_len(const std::size_t length)
    {
        sbuf.prepare(length);

        boost::asio::async_read(
            sp,
            sbuf,
            boost::asio::transfer_exactly(length),
            boost::bind(&serialport::ReadHandler, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred)
        );
        timer.expires_from_now(std::chrono::seconds(timeout_sec));
        timer.async_wait(boost::bind(&serialport::on_timer, this, boost::asio::placeholders::error));
        isFinished = false;

        io.reset();
        io.run();

        return;
    }
    frame_type GetRecvData(void)
    {
        if (isFinished)
            return recvdata;
        else
            return frame_type();
    };
    bool isProcessFinished(void) const
    {
        return isFinished;
    };
    bool isProcessTimeout(void) const
    {
        return isTimeout;
    };
    std::string GetLastErrorMessage(void) const
    {
        return err.message();
    };
    int setTimeout_sec(const unsigned int new_timeout_sec)
    {
        unsigned int prev = timeout_sec;
        timeout_sec = new_timeout_sec;

        return prev;
    }
};

#endif