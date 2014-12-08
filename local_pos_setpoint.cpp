#include <sstream>
#include "mavlink.h"

#include "Poco/Timestamp.h"
#include "April_Tag_Demo/vision_position_estimate.hpp"
#include "Lcm.h"


#include "Poco/Logger.h"
#include "Poco/FileChannel.h"
#include "Poco/AutoPtr.h"
#include "Poco/FormattingChannel.h"
#include "Poco/PatternFormatter.h"

using Poco::Logger;
using Poco::FileChannel;
using Poco::FormattingChannel;
using Poco::PatternFormatter;
using Poco::AutoPtr;

void checkLCM_nonblocking(lcm::LCM &lcm)
{
  // setup the LCM file descriptor for waiting.
  int lcm_fd = lcm.getFileno();
  fd_set fds;
  FD_ZERO(&fds);
  FD_SET(lcm_fd, &fds);

  // wait a limited amount of time for an incoming message
  struct timeval timeout = {
    0,  // seconds
    10   // microseconds
  };

  int status = select(lcm_fd + 1, &fds, 0, 0, &timeout);

  if(0 == status) {
    // no messages
  } else if(FD_ISSET(lcm_fd, &fds)) {
    // LCM has events ready to be processed.
    lcm.handle();
  }
}

int open_uart(int baud, const char *uart_name, struct termios *uart_config_original)
{
    /* process baud rate */
    int speed;

    switch (baud) {
    case 0:      speed = B0;      break;
    case 50:     speed = B50;     break;
    case 75:     speed = B75;     break;
    case 110:    speed = B110;    break;
    case 134:    speed = B134;    break;
    case 150:    speed = B150;    break;
    case 200:    speed = B200;    break;
    case 300:    speed = B300;    break;
    case 600:    speed = B600;    break;
    case 1200:   speed = B1200;   break;
    case 1800:   speed = B1800;   break;
    case 2400:   speed = B2400;   break;
    case 4800:   speed = B4800;   break;
    case 9600:   speed = B9600;   break;
    case 19200:  speed = B19200;  break;
    case 38400:  speed = B38400;  break;
    case 57600:  speed = B57600;  break;
    case 115200: speed = B115200; break;
    case 230400: speed = B230400; break;

    default:
        return -EINVAL;
    }

    /* open uart */
    fflush(stdout);
    int uart_fd = open(uart_name, O_RDWR | O_NOCTTY);

    if (uart_fd < 0) {
        return uart_fd;
    }

    /* Try to set baud rate */
    struct termios uart_config;
    int termios_state;

    /* Back up the original uart configuration to restore it after exit */
    termios_state = tcgetattr(uart_fd, uart_config_original);
    if (termios_state < 0) {
        printf("tcgetattr error\n");
        fflush(stdout);
        close(uart_fd);
        return -1;
    }

    /* Fill the struct for the new configuration */
    bzero(&uart_config, sizeof(uart_config));

    uart_config.c_iflag = IGNPAR; // ignore framing and parity errors

    // CLOCAL : ignore modem control lines
    uart_config.c_cflag = speed | CS8 | CREAD | CLOCAL;

    // all reads should be satisfied immediately
    uart_config.c_cc[VTIME] = 0;
    uart_config.c_cc[VMIN]  = 0;

    tcflush(uart_fd, TCIFLUSH); 
    if ((termios_state = tcsetattr(uart_fd, TCSANOW, &uart_config)) < 0) {
        // g_logger.warning("ERR SET CONF %s\n", uart_name);
        printf("tcsetattr error\n");
        close(uart_fd);
        return -1;
    }

    tcgetattr(uart_fd, &uart_config);

    return uart_fd;
}

void set_offboard(int uart_fd)
{
    mavlink_message_t message;
    char buf[300];
    unsigned len = 0;

    mavlink_command_long_t cmd_long;
    cmd_long.target_system=(uint8_t)1;
    cmd_long.target_component=(uint8_t)50;
    cmd_long.command=(uint16_t)176;
    cmd_long.confirmation=(uint8_t)1;
    cmd_long.param1=(float)209;//Base mode on arming via QGC. 29: disarms on mode switch, 209: stays armed
    cmd_long.param2=(float)6; //Custom mode - Offboard
    cmd_long.param3=(float)0;
    cmd_long.param4=(float)0;
    cmd_long.param5=(float)0;
    cmd_long.param6=(float)0;
    cmd_long.param7=(float)0;

    mavlink_msg_command_long_encode(200, 0, &message, &cmd_long);
    len = mavlink_msg_to_send_buffer((uint8_t*)buf, &message);
    write(uart_fd, buf, len);
}

void set_posControl(int uart_fd)
{
    mavlink_message_t message;
    char buf[300];
    unsigned len = 0;

    mavlink_command_long_t cmd_long;
    cmd_long.target_system=(uint8_t)1;
    cmd_long.target_component=(uint8_t)50;
    cmd_long.command=(uint16_t)176;
    cmd_long.confirmation=(uint8_t)1;
    cmd_long.param1=(float)1; //Base mode on arming via QGC. 29: disarms on mode switch, 209: stays armed
    cmd_long.param2=(float)3; //Custom mode - position control
    cmd_long.param3=(float)0;
    cmd_long.param4=(float)0;
    cmd_long.param5=(float)0;
    cmd_long.param6=(float)0;
    cmd_long.param7=(float)0;


    mavlink_msg_command_long_encode(200, 0, &message, &cmd_long);
    len = mavlink_msg_to_send_buffer((uint8_t*)buf, &message);
    write(uart_fd, buf, len);
}

void set_mav_guided_enabled(int uart_fd)
{
    mavlink_message_t message;
    char buf[300];
    unsigned len = 0;

    mavlink_command_long_t cmd_long;
    cmd_long.target_system=(uint8_t)1;
    cmd_long.target_component=(uint8_t)50;
    cmd_long.command=(uint16_t)MAV_CMD_NAV_GUIDED_ENABLE;
    cmd_long.confirmation=(uint8_t)1;
    cmd_long.param1=(float)0;
    cmd_long.param2=(float)0;
    cmd_long.param3=(float)0;
    cmd_long.param4=(float)0;
    cmd_long.param5=(float)0;
    cmd_long.param6=(float)0;
    cmd_long.param7=(float)0;

    mavlink_msg_command_long_encode(200, 0, &message, &cmd_long);
    len = mavlink_msg_to_send_buffer((uint8_t*)buf, &message);
    write(uart_fd, buf, len);
}

int main(int argc, char** argv)
{
    std::stringstream message_to_log;

    AutoPtr<FileChannel> peChannel(new FileChannel);
    peChannel->setProperty("path", "posEst.log");
    peChannel->setProperty("archive", "timestamp");
    AutoPtr<PatternFormatter> pePF(new PatternFormatter);
    pePF->setProperty("pattern", "%Y-%m-%d %H:%M:%S %s: %t");
    AutoPtr<FormattingChannel> peFC(new FormattingChannel(pePF, peChannel));
    Logger::root().setChannel(peFC);

    Logger& posEstLogger = Logger::create("posEst", peFC);
    posEstLogger.information("*****************************************************");

    int uart_fd;
    int baudrate;
    const char * device_name;
    struct termios uart_config_original;

    baudrate = DEFAULT_BAUD;
    device_name = NULL;

    bool err_flag = false;
    int ch;
    while ((ch = getopt(argc, argv, "b:d:")) != EOF) {
        switch (ch) {
        case 'b':
            baudrate = strtoul(optarg, NULL, 10);
            break;

        case 'd':
            device_name = optarg;
            break;

        default:
            err_flag = true;
            break;
        }
    }

    if (NULL == device_name) {
        device_name = DEFAULT_DEVICE;
    }

    if (err_flag) {
        printf("usage: [-d device] [-b baudrate]");
        return ERROR;
    }

    printf("opening: %s, baud: %i\n", device_name, baudrate);
    message_to_log.str("");
    message_to_log << "opening: " << device_name << ", baud: " << baudrate;
    posEstLogger.information(message_to_log.str());
    uart_fd = open_uart(baudrate, device_name, &uart_config_original);

    if (uart_fd < 0) 
    {
        printf("could not open %s\n", device_name);
        return ERROR;
    }
    printf("successfully opened port\n");
    posEstLogger.information("successfully opened port\n");

    mavlink_message_t message;
    uint32_t drain_delay_us = 1e6 / 200; // delay after writing to serial port
    unsigned len = 0;

    lcm::LCM lcm;
    if(!lcm.good())
    {
        printf("LCM IS NOT GOOD!\n");
        posEstLogger.information("LCM is NOT GOOD\n");
        return 1;
    }
                  

    set_offboard(uart_fd);

    Poco::Timestamp timer;
    Poco::Timestamp epoch_timer;

    printf("starting loop\n");
    posEstLogger.information("starting loop\n");

    static uint8_t              buf[100];

    // main loop
    while(1)
    {
        uint8_t system_id = 1;
        uint8_t comp_id = 50;   // ?not sure if this matters?

        checkLCM_nonblocking(lcm);
        if( timer.elapsed() < (1.0 / 3.0) * 1e6)
        {
            timer.update();
            /* Send setpoint */
            // params in this order
            float x = 1; ///< X Position in NED frame in meters
            float y = 1; ///< Y Position in NED frame in meters
            float z = 1; ///< Z Position in NED frame in meters (note, altitude is negative in NED)
            float vx = 0.0; ///< X velocity in NED frame in meter / s
            float vy = 0.0; ///< Y velocity in NED frame in meter / s
            float vz = 0.0; ///< Z velocity in NED frame in meter / s
            float afx = 0; ///< X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
            float afy = 0; ///< Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
            float afz = 0; ///< Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
            float yaw = 0;//3.14 / 4; ///< yaw setpoint in rad
            float yaw_rate = 0.0; ///< yaw rate setpoint in rad/s
            uint16_t type_mask = 0b0000100111111000; ///< Bitmask to indicate which dimensions should be 
                // ignored by the vehicle: a value of 0b0000000000000000 or 0b0000001000000000 
                // indicates that none of the setpoint dimensions should be ignored. If bit 10
                // is set the floats afx afy afz should be interpreted as force instead of acceleration. 
                // Mapping: bit 1: x, bit 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, 
                // bit 9: az, bit 10: is force setpoint, bit 11: yaw, bit 12: yaw rate
            uint8_t target_system = system_id; ///< System ID
            uint8_t target_component = comp_id; ///< Component ID
            uint8_t coordinate_frame = 1; ///< Valid options are: 
                // MAV_FRAME_LOCAL_NED = 1, MAV_FRAME_LOCAL_OFFSET_NED = 7, 
                // MAV_FRAME_BODY_NED = 8, MAV_FRAME_BODY_OFFSET_NED = 9

            // printf("x: %f y: %f z: %f yaw: %f\n", x,y,z, yaw);
            message_to_log.str("");
            message_to_log << "SetPoint: x: " << x << " y: " << y << " z: " << z;
            posEstLogger.information(message_to_log.str());

            epoch_timer.update();
            mavlink_msg_set_position_target_local_ned_pack(system_id, comp_id, &message,
                            epoch_timer.epochMicroseconds(), target_system, target_component, coordinate_frame, type_mask, 
                            x, y, z, vx, vy, vz, afx, afy, afz, yaw, yaw_rate);

            len = mavlink_msg_to_send_buffer((uint8_t*)buf, &message);
            write(uart_fd, buf, len);

            usleep(drain_delay_us);
            tcdrain(uart_fd);
        }
    }
    return OK;
}
