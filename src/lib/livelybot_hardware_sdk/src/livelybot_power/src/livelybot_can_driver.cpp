#include "livelybot_can_driver.hpp"
#include <thread>

using namespace livelybot_can;

CAN_Driver::CAN_Driver(const char* can_dev)
{
    this->can_dev = can_dev;
    this->init();    
    this->run_flag = 1;
}

CAN_Driver::~CAN_Driver(void)
{       
    this->run_flag = 0;
    if(receive_thread.joinable())
    {
        receive_thread.join();
    }
    close(this->can_socket);
}

void CAN_Driver::start(void)
{
    receive_thread = std::thread(&CAN_Driver::receive, this);
}

void CAN_Driver::start_callback(callback_func func)
{
    receive_thread = std::thread(&CAN_Driver::receive_callback, this, func);
}

void CAN_Driver::init(void)
{
    this->can_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (this->can_socket == -1) {
        perror("Error creating can socket");
    }

    strcpy(this->ifr.ifr_name, this->can_dev);
    ioctl(this->can_socket, SIOCGIFINDEX, &this->ifr);

    this->addr.can_family = AF_CAN;
    this->addr.can_ifindex = this->ifr.ifr_ifindex;

    if (bind(this->can_socket, (struct sockaddr*)&this->addr, sizeof(this->addr)) == -1) {
        perror("Error binding socket");
        close(this->can_socket);
    }
}

void CAN_Driver::send(unsigned int can_id, unsigned char* data, unsigned char len)
{
    this->frame_tx.can_id = can_id;
    this->frame_tx.can_dlc = len;
    memcpy(this->frame_tx.data, data, len);

    int retval = write(this->can_socket, &this->frame_tx, sizeof(this->frame_tx));

    if(retval == -1)
    {
        perror("Error sending CAN frame");
    }
}

void CAN_Driver::receive(void)
{
    while (1)
    {
        int n_bytes;

        n_bytes = read(this->can_socket, &this->frame_rx, sizeof(struct can_frame));

        if(n_bytes < 0)
        {
            perror("Error receiving CAN frame");
            break;
        }

        printf("Received CAN frame - ID: 0x%03X, Length: %d, Data: ", this->frame_rx.can_id, this->frame_rx.can_dlc);
        for (int i = 0; i < this->frame_rx.can_dlc; i++) {
            printf("0x%02X ", this->frame_rx.data[i]);
        }
        printf("\n");

        usleep(10000);
    }
    pthread_exit(NULL);
}

void CAN_Driver::receive_callback(callback_func recv_callback)
{
    while (run_flag)
    {
        int n_bytes;

        n_bytes = read(this->can_socket, &this->frame_rx, sizeof(struct can_frame));

        if(n_bytes < 0)
        {
            perror("Error receiving CAN frame");
            break;
        }

        recv_callback(this->frame_rx.can_id, this->frame_rx.data, this->frame_rx.can_dlc);
        
        usleep(10000);
    }
    pthread_exit(NULL);
}

