#ifndef _CANPORT_H_
#define _CANPORT_H_
#include "ros/ros.h"
#include "motor.h"
#include <condition_variable>
#include <thread>
#include "../lively_serial.h"
#include <unordered_set>

class canport
{
private:
    int motor_num;
    int CANport_num;
    ros::NodeHandle n;
    std::vector<motor *> Motors;
    std::map<int, motor *> Map_Motors_p;
    bool sendEnabled;
    int canboard_id, canport_id;
    std::vector<motor_back_t *> Motor_data;
    lively_serial *ser;
    cdc_tr_message_s cdc_tr_message;
    int id_max = 0;
    int control_type = 0;
    int motor_num_max = 30;
    float port_version = 0.0f;
    std::unordered_set<int> motors_id;
    int mode_flag = 0;
    std::vector<int> port_motor_id;

public:
    canport(int _CANport_num, int _CANboard_num, lively_serial *_ser) : ser(_ser)
    {
        canboard_id = _CANboard_num;
        canport_id = _CANport_num;
        if (n.getParam("robot/CANboard/No_" + std::to_string(_CANboard_num) + "_CANboard/CANport/CANport_" + std::to_string(_CANport_num) + "/motor_num", motor_num))
        {
            // ROS_INFO("Got params motor_num: %d",motor_num);
        }
        else
        {
            ROS_ERROR("Faile to get params motor_num");
        }

        if (motor_num_max < motor_num)
        {
            ROS_ERROR("Too many motors, Supports up to %d motors, but there are actually %d motors", motor_num_max, motor_num);
            exit(-1);
        }
        

        for (int i = 1; i <= motor_num; i++)
        {
            int temp_id = 0;
            if (n.getParam("robot/CANboard/No_" + std::to_string(_CANboard_num) + "_CANboard/CANport/CANport_" + std::to_string(_CANport_num) + "/motor/motor" + std::to_string(i) + "/id", temp_id))
            {
                port_motor_id.push_back(temp_id);
                if (id_max < temp_id)
                {
                    id_max = temp_id;
                }
            }
            else
            {
                ROS_ERROR("Faile to get params id");
            }
        }
        for (size_t i = 1; i <= motor_num; i++)
        {
            Motors.push_back(new motor(i, _CANport_num, _CANboard_num, &cdc_tr_message, id_max));
        }
        for (motor *m : Motors)
        {
            Map_Motors_p.insert(std::pair<int, motor *>(m->get_motor_id(), m));
        }
        ser->init_map_motor(&Map_Motors_p);
        ser->port_version_init(&port_version);
        ser->port_motors_id_init(&motors_id, &mode_flag);
        // ser->test_ser_motor();
        sendEnabled = false;
        // std::thread(&canport::send, this);
    }
    ~canport()
    {
        // for (motor_back_t* m:Motor_data)
        // {
        //     delete m;
        // }
    }
    float set_motor_num()
    {
        if (cdc_tr_message.head.s.cmd != MODE_SET_NUM)
        {
            cdc_tr_message.head.s.head = 0XF7;
            cdc_tr_message.head.s.cmd = MODE_SET_NUM;
            cdc_tr_message.head.s.len = 1;
            memset(&cdc_tr_message.data, 0, cdc_tr_message.head.s.len);
        }

        cdc_tr_message.data.data[0] = motor_num;
        
        int t = 0;
        #define MAX_DALAY 10000  // 单位ms
        while (t++ < MAX_DALAY)
        {
            motor_send_2();
            ros::Duration(0.001).sleep();
            if (port_version >= 2)
            {
                // ROS_INFO("\033[1;32m ttt %d\033[0m", t);
                break;
            }
        }

        if (t < MAX_DALAY)
        {
            ROS_INFO("\033[1;32mCANboard(%d) version is: %.1fv\033[0m", canboard_id, port_version);
        }
        else
        {
            ROS_ERROR("CANboard(%d) CANport(%d) Connection disconnected!!!", canboard_id, canport_id);
        }

        return port_version;
    }

    int set_conf_load()
    {
        if (cdc_tr_message.head.s.cmd != MODE_CONF_LOAD)
        {
            cdc_tr_message.head.s.head = 0XF7;
            cdc_tr_message.head.s.cmd = MODE_CONF_LOAD;
            cdc_tr_message.head.s.len = 1;
            memset(&cdc_tr_message.data, 0, cdc_tr_message.head.s.len);
        }
        cdc_tr_message.data.data[0] = 0x7f;

        int t = 0;
        int num = 0;
        int max_delay = 10000;
        motors_id.clear();
        mode_flag = 0;
        while (t++ < max_delay)
        {
            motor_send_2();
            ros::Duration(0.01).sleep();
            num = 0;
            if (mode_flag == MODE_CONF_LOAD)
            {
                for (int i = 1; i <= motor_num; i++)
                {
                    int id = port_motor_id[i - 1];
                    if (motors_id.count(id) == 1)
                    {
                        ++num;
                    }
                }
            }

            if (num == motor_num)
            {
                break;
            }
        }

        if (num == motor_num)
        {
            ROS_INFO("\033[1;32mSettings have been restored. Initiating motor zero point reset.\033[0m");
            return 0;
        }
        else 
        {
            ROS_ERROR("Restoration of settings failed.");
            return 1;
        }
    }

    int set_conf_load(int id)
    {
        
        if (cdc_tr_message.head.s.cmd != MODE_CONF_LOAD)
        {
            cdc_tr_message.head.s.head = 0XF7;
            cdc_tr_message.head.s.cmd = MODE_CONF_LOAD;
            cdc_tr_message.head.s.len = 1;
            memset(&cdc_tr_message.data, 0, cdc_tr_message.head.s.len);
        }
        cdc_tr_message.data.data[0] = id;

        int t = 0;
        int max_delay = 10000;
        motors_id.clear();
        mode_flag = 0;
        while (t++ < max_delay)
        {
            motor_send_2();
            ros::Duration(0.01).sleep();
            if (mode_flag == MODE_CONF_LOAD && motors_id.count(id) == 1)
            {
                return 0;
            }
        }

        return 1;
    }
    int set_reset_zero()
    {
        if (cdc_tr_message.head.s.cmd != MODE_RESET_ZERO)
        {
            cdc_tr_message.head.s.head = 0XF7;
            cdc_tr_message.head.s.cmd = MODE_RESET_ZERO;
            cdc_tr_message.head.s.len = 1;
            memset(&cdc_tr_message.data, 0, cdc_tr_message.head.s.len);
        }
        cdc_tr_message.data.data[0] = 0x7f;

        int t = 0;
        int num = 0;
        int max_delay = 10000;
        motors_id.clear();
        mode_flag = 0;
        while (t++ < max_delay)
        {
            motor_send_2();
            ros::Duration(0.001).sleep();
            num = 0;
            if (mode_flag == MODE_RESET_ZERO)
            {
                for (int i = 1; i <= motor_num; i++)
                {
                    // if (motors_id.count(i) == 1)
                    int id = port_motor_id[i - 1];
                    if (motors_id.count(id) == 1)
                    {
                        ++num;
                    }
                }
            }

            if (num == motor_num)
            {
                break;
            }
        }

        if (num == motor_num)
        {
            ROS_INFO("\033[1;32mMotor zero position reset successfully, waiting for the motor to save the settings.\033[0m");
            return 0;
        }
        else 
        {
            ROS_ERROR("Motor reset to zero position failed.");
            return 1;
        }
        
    }

    int set_reset_zero(int id)
    {
        
        if (cdc_tr_message.head.s.cmd != MODE_RESET_ZERO)
        {
            cdc_tr_message.head.s.head = 0XF7;
            cdc_tr_message.head.s.cmd = MODE_RESET_ZERO;
            cdc_tr_message.head.s.len = 1;
            memset(&cdc_tr_message.data, 0, cdc_tr_message.head.s.len);
        }
        cdc_tr_message.data.data[0] = id;

        int t = 0;
        int max_delay = 10000;
        motors_id.clear();
        mode_flag = 0;
        while (t++ < max_delay)
        {
            motor_send_2();
            ros::Duration(0.001).sleep();
            if (mode_flag == MODE_RESET_ZERO && motors_id.count(id) == 1)
            {
                return 0;
            }
        }

        return 1;
    }

    void set_stop()
    {
        if (cdc_tr_message.head.s.cmd != MODE_STOP)
        {
            cdc_tr_message.head.s.head = 0XF7;
            cdc_tr_message.head.s.cmd = MODE_STOP;
            cdc_tr_message.head.s.len = 1;
            memset(&cdc_tr_message.data, 0, cdc_tr_message.head.s.len);
        }
        cdc_tr_message.data.data[0] = 0x7f;
    }

    void set_reset()
    {
        if (cdc_tr_message.head.s.cmd != MODE_RESET)
        {
            cdc_tr_message.head.s.head = 0XF7;
            cdc_tr_message.head.s.cmd = MODE_RESET;
            cdc_tr_message.head.s.len = 1;
            memset(&cdc_tr_message.data, 0, cdc_tr_message.head.s.len);
        }
        cdc_tr_message.data.data[0] = 0x7f;
    }

    void set_conf_write()
    {
        
        if (cdc_tr_message.head.s.cmd != MODE_CONF_WRITE)
        {
            cdc_tr_message.head.s.head = 0XF7;
            cdc_tr_message.head.s.cmd = MODE_CONF_WRITE;
            cdc_tr_message.head.s.len = 1;
            memset(&cdc_tr_message.data, 0, cdc_tr_message.head.s.len);
        }
        cdc_tr_message.data.data[0] = 0x7f;

        int t = 0;
        int num = 0;
        int max_delay = 10000;
        motors_id.clear();
        mode_flag = 0;
        while (t++ < max_delay)
        {
            motor_send_2();
            ros::Duration(0.001).sleep();
            num = 0;
            if (mode_flag == MODE_CONF_WRITE)
            {
                for (int i = 1; i <= motor_num; i++)
                {
                    int id = port_motor_id[i - 1];
                    if (motors_id.count(id) == 1)
                    {
                        ++num;
                    }
                }
            }

            if (num == motor_num)
            {
                break;
            }
        }

        if (num == motor_num)
        {
            ROS_INFO("\033[1;32mSettings saved successfully.\033[0m");
        }
        else 
        {
            ROS_INFO("\033[1;32mFailed to save settings.\033[0m");
            exit(-1);
        }
    }
    int set_conf_write(int id)
    {
        
        if (cdc_tr_message.head.s.cmd != MODE_CONF_WRITE)
        {
            cdc_tr_message.head.s.head = 0XF7;
            cdc_tr_message.head.s.cmd = MODE_CONF_WRITE;
            cdc_tr_message.head.s.len = 1;
            memset(&cdc_tr_message.data, 0, cdc_tr_message.head.s.len);
        }
        cdc_tr_message.data.data[0] = id;

        int t = 0;
        int max_delay = 10000;
        motors_id.clear();
        mode_flag = 0;
        while (t++ < max_delay)
        {
            motor_send_2();
            ros::Duration(0.001).sleep();
            if (mode_flag == MODE_CONF_WRITE && motors_id.count(id) == 1)
            {
                return 0;
            }
        }

        return 1;
    }
    void send_get_motor_state_cmd()
    {
        if (cdc_tr_message.head.s.cmd != MODE_MOTOR_STATE)
        {
            cdc_tr_message.head.s.head = 0XF7;
            cdc_tr_message.head.s.cmd = MODE_MOTOR_STATE;
            cdc_tr_message.head.s.len = 1;
            memset(&cdc_tr_message.data, 0, cdc_tr_message.head.s.len);
        }
        cdc_tr_message.data.data[0] = 0x7f;
        motor_send_2();
    }
    void puch_motor(std::vector<motor *> *_Motors)
    {
        for (motor *m : Motors)
        {
            _Motors->push_back(m);
        }
    }
    void push_motor_data()
    {
        for (motor_back_t *r : Motor_data)
        {
        }
    }
    void motor_send()
    {
        for (motor *m : Motors)
        {
            ser->send(m->return_cmd_p());
        }
    }

    void motor_send_2()
    {
        ser->send_2(&cdc_tr_message);
    }
    int get_motor_num() { return motor_num; }
    int get_canboard_id() { return canboard_id; }
    int get_canport_id() { return canport_id; }
};
#endif