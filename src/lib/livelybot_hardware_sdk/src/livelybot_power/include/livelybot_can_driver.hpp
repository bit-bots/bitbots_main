#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <thread>

typedef void (*callback_func)(int, unsigned char*,unsigned char);

namespace livelybot_can
{
    class CAN_Driver
    {
    public:
        CAN_Driver(const char* can_dev);
        ~CAN_Driver(void);
        void start(void);
        void start_callback(callback_func func);
        void send(unsigned int can_id, unsigned char* data, unsigned char len);
        bool run_flag;
    private:
        const char* can_dev;
        int can_socket;
        struct can_frame frame_tx;
        struct can_frame frame_rx;        
        struct sockaddr_can addr;
        struct ifreq ifr;
        std::thread receive_thread;
        int exit_flag = 0;
        void init(void);
        void receive(void);
        void receive_callback(callback_func func);
    };
}