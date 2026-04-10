#include "lively_serial.h"


lively_serial::lively_serial(std::string *port, uint32_t baudrate)
{
    init_flag = false;
    error_flag = false;
    _ser.setPort(*port); // 设置打开的串口名称
    _ser.setBaudrate(baudrate);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000); // 创建timeout
    _ser.setTimeout(to);                                       // 设置串口的timeout

    // 打开串口
    try
    {
        _ser.open(); // 打开串口
    }
    catch (const std::exception &e)
    {
        ROS_ERROR_STREAM("Motor Unable to open port "); // 打开串口失败，打印信息
        this->error_flag = true;
    }
    if (_ser.isOpen())
    {
        // ROS_INFO_STREAM("Motor Serial Port initialized."); // 成功打开串口，打印信息
        init_flag = true;
    }
    else
    {
    }
    init_flag = true;
}


lively_serial::~lively_serial()
{
    if(_ser.isOpen())
    {
        _ser.close();
        init_flag = false;
    }
}

void lively_serial::recv_1for6_42()
{
    uint8_t CRC8 = 0;
    uint16_t CRC16 = 0;
    cdc_tr_message_head_data_s SOF = {0};
    cdc_tr_message_data_s cdc_rx_message_data = {0};
    while (ros::ok() && init_flag)
    {
        try
        {
            _ser.read(&(SOF.head), 1); 
            if (SOF.head == 0xF7)      //  head
            {
                _ser.read(&(SOF.cmd), 4);
                if (SOF.crc8 == Get_CRC8_Check_Sum((uint8_t *)&(SOF.cmd), 3, 0xFF)) // cmd_id
                {
                    _ser.read((uint8_t *)&CRC16, 2);
                    _ser.read((uint8_t *)&cdc_rx_message_data, SOF.len);
                    if (CRC16 != crc_ccitt(0xFFFF, (const uint8_t *)&cdc_rx_message_data, SOF.len))
                    {
                        memset(&cdc_rx_message_data, 0, sizeof(cdc_rx_message_data) / sizeof(int));
                    }
                    else
                    {
                        // printf("cmd %02X  ", SOF.cmd);
                        // for (int i = 0; i < SOF.len; i++)
                        // {
                        //     printf("0x%02X ", cdc_rx_message_data.data[i]);
                        // }
                        // printf("\n");

                        switch (SOF.cmd)
                        {
                        case (MODE_RESET_ZERO):
                        case (MODE_CONF_WRITE):
                        case (MODE_CONF_LOAD):
                            *p_mode_flag = SOF.cmd;
                            for (int i = 0; i < SOF.len; i++)
                            {
                                // printf("0x%02X ", cdc_rx_message_data.data[i]);
                                p_motor_id->insert(cdc_rx_message_data.data[i]);
                            }
                            break;

                        case(MODE_SET_NUM):
                            *p_port_version = cdc_rx_message_data.data[2];
                            *p_port_version += (float)cdc_rx_message_data.data[3] * 0.1f;
                            break;
                        case(MODE_FUN_V):
                            *p_fun_v = (fun_version)cdc_rx_message_data.data[0];
                            break;
                        case(MODE_MOTOR_VERSION):
                            for (size_t i = 0; i < SOF.len / sizeof(cdc_rx_motor_version_s); i++)
                            {
                                auto it = Map_Motors_p.find(cdc_rx_message_data.motor_version[i].id);
                                if (it != Map_Motors_p.end())
                                {
                                    it->second->set_version(cdc_rx_message_data.motor_version[i]);
                                }
                            }
                            break;

                        case(MODE_MOTOR_STATE):
                            for (size_t i = 0; i < SOF.len / sizeof(cdc_rx_motor_state_s); i++)
                            {
                                auto it = Map_Motors_p.find(cdc_rx_message_data.motor_state[i].id);
                                if (it != Map_Motors_p.end())
                                {
                                    it->second->fresh_data(0, 0, 
                                                    cdc_rx_message_data.motor_state[i].pos,
                                                    cdc_rx_message_data.motor_state[i].val,
                                                    cdc_rx_message_data.motor_state[i].tqe);
                                }
                            }
                            break;
                        case(MODE_MOTOR_STATE2):
                            for (size_t i = 0; i < SOF.len / sizeof(cdc_rx_motor_state2_s); i++)
                            {
                                auto it = Map_Motors_p.find(cdc_rx_message_data.motor_state2[i].id);
                                if (it != Map_Motors_p.end())
                                {
                                    it->second->fresh_data(
                                                    cdc_rx_message_data.motor_state2[i].mode,
                                                    cdc_rx_message_data.motor_state2[i].fault,
                                                    cdc_rx_message_data.motor_state2[i].pos,
                                                    cdc_rx_message_data.motor_state2[i].val,
                                                    cdc_rx_message_data.motor_state2[i].tqe);
                                }
                            }
                            break;
                        }
                    }
                }
                else
                {
                    // ROS_ERROR("clcl");
                }
            }
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            _ser.close();
            error_flag = true;
            break;
        }
    }
}

bool lively_serial::is_serial_error(void)
{
    return this->error_flag;
}

void lively_serial::set_run_flag(bool flag)
{
    this->init_flag = flag;
}

bool lively_serial::get_run_flag(void)
{
    return this->init_flag;
}

void lively_serial::close(void)
{
    try
    {
        /* code */
        if(_ser.isOpen())
        {
            _ser.flush();
        }
        _ser.close();
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
    
}

void lively_serial::send_2(cdc_tr_message_s *cdc_tr_message)
{
    cdc_tr_message->head.s.crc8 = Get_CRC8_Check_Sum(&(cdc_tr_message->head.data[1]), 3, 0xFF);
    cdc_tr_message->head.s.crc16 = crc_ccitt(0xFFFF, &(cdc_tr_message->data.data[0]), cdc_tr_message->head.s.len);

    // uint8_t *byte_ptr = (uint8_t *)&cdc_tr_message->head.s.head;
    // printf("send:\n");
    // for (size_t i = 0; i < cdc_tr_message->head.s.len + 7; i++)
    // {
    //     printf("0x%.2X ", byte_ptr[i]);
    // }
    // printf("\n\n");

    try
    {
        if(_ser.isOpen())
        {
            _ser.write((const uint8_t *)&cdc_tr_message->head.s.head, cdc_tr_message->head.s.len + sizeof(cdc_tr_message_head_s));
        }
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        _ser.close();
        error_flag = true;
    }
}


void lively_serial::port_version_init(float *p)
{
    p_port_version = p;
}


void lively_serial::port_motors_id_init(std::unordered_set<int> *_p_motor_id, int *_p_mode_flag)
{
    p_motor_id = _p_motor_id;
    p_mode_flag = _p_mode_flag;
}


void lively_serial::init_map_motor(std::map<int, motor *> *_Map_Motors_p)
{
    Map_Motors_p = *_Map_Motors_p;
}


void lively_serial::port_fun_v_init(fun_version *_p_fun_v)
{
    p_fun_v = _p_fun_v;
}
