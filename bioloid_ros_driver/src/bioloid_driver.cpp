#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "bioloid_ros_driver/allcmd.h"
#include "zigbee.h"
#include <termios.h>
#include <unistd.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>

#define ACK (short)0
#define RDY (short)-1

#define MAX_LOOP_COUNT 5
#define MAX_INIT_LOOP_COUNT 100

boost::mutex radio_in_use;

bool allcmdCallback(bioloid_ros_driver::allcmd::Request &req, bioloid_ros_driver::allcmd::Response &res)
{
  boost::mutex::scoped_lock lock(radio_in_use);
  short RxData, TxData;
  char type, id;
  int i;
  // std_msgs::Int32 x;
  // testpub.publish(x);
  ROS_INFO("Got Request: %s\n, DevID: %d\n", req.command_type.c_str(), req.device_id);
  ros::Rate r(200);
  int flag = 0;
  if (req.command_type == "GetMotorTargetPosition")
  {
    int loop_count = 0;
    type = 'p';
    id = req.device_id;
    TxData = ((short)type << 8) | ((short)id & 0xff);
    while (loop_count < MAX_LOOP_COUNT)
    {
      ROS_INFO("Sending data");
      zgb_tx_data(TxData);
      for (i = 0; i < 300; i++)
      {
        if (zgb_rx_check() == 1)
        {
          RxData = zgb_rx_data();
          ROS_INFO("got get_motor_target_position response %d\n", RxData);
          res.val = RxData;
          flag = 1;
          break;
        }
        r.sleep();
      }
      if (flag)
        break;

      loop_count++;
    }

    if (loop_count == MAX_LOOP_COUNT)
    {
      ROS_ERROR("Response from robot timed out");
    }
    loop_count = 0;

    if (flag)
    {
      TxData = ACK;
      zgb_tx_data(TxData);
      ROS_INFO("successfully acknowledged get_motor_target_position response \n");
    }
    else
    {
       ROS_WARN("failed to get motor target position");
    }
  }
  else if (req.command_type == "GetMotorCurrentPosition")
  {
    int loop_count = 0;
    type = 'q';
    id = req.device_id;
    TxData = ((short)type << 8) | ((short)id & 0xff);
    while (loop_count < MAX_LOOP_COUNT)
    {
      ROS_INFO("Sending data");
      zgb_tx_data(TxData);
      for (i = 0; i < 300; i++)
      {
        if (zgb_rx_check() == 1)
        {
          RxData = zgb_rx_data();
          ROS_INFO("got get_motor_current_position response %d\n", RxData);
          res.val = RxData;
          flag = 1;
          break;
        }
        r.sleep();
      }
      if (flag)
        break;

      loop_count++;
    }

    if (loop_count == MAX_LOOP_COUNT)
    {
      ROS_ERROR("Response from robot timed out");
    }
    loop_count = 0;

    if (flag)
    {
      TxData = ACK;
      zgb_tx_data(TxData);
      ROS_INFO("successfully acknowledged get_motor_current_position response \n");
    }
    else
    {
       ROS_WARN("failed to get motor current position");
    }
  }
  else if (req.command_type == "GetIsMotorMoving")
  {
    int loop_count = 0;
    type = 'm';
    id = req.device_id;
    TxData = ((short)type << 8) | ((short)id & 0xff);
    while (loop_count < MAX_LOOP_COUNT)
    {
      ROS_INFO("Sending data");
      zgb_tx_data(TxData);
      for (i = 0; i < 300; i++)
      {
        if (zgb_rx_check() == 1)
        {
          RxData = zgb_rx_data();
          ROS_INFO("got get_is_motor_moving response %d\n", RxData);
          res.val = RxData;
          flag = 1;
          break;
        }
        r.sleep();
      }
      if (flag)
          break;

      loop_count++;
    }

    if (loop_count == MAX_LOOP_COUNT)
    {
      ROS_ERROR("Response from robot timed out");
    }
    loop_count = 0;

    if (flag)
    {
      TxData = ACK;
      zgb_tx_data(TxData);
      ROS_INFO("successfully acknowledged get_is_motor_moving response \n");
    }
    else
    {
       ROS_WARN("failed to get motor is motor moving response");
    }
  }
  else if (req.command_type == "GetSensorValue")
  {
    int loop_count = 0;
    type = 'v';
    id = req.device_id;
    TxData = ((short)type << 8) | ((short)id & 0xff);
    while (loop_count < MAX_LOOP_COUNT)
    {
      ROS_INFO("Sending data");
      zgb_tx_data(TxData);
      for (i = 0; i < 300; i++)
      {
        if (zgb_rx_check() == 1)
        {
          RxData = zgb_rx_data();
          ROS_INFO("got get_sensor_value response %d\n", RxData);
          res.val = RxData;
          flag = 1;
          break;
        }
        r.sleep();
      }
      if (flag)
        break;

      loop_count++;
    }

    if (loop_count == MAX_LOOP_COUNT)
    {
      ROS_ERROR("Response from robot timed out");
    }
    loop_count = 0;

    if (flag)
    {
      TxData = ACK;
      zgb_tx_data(TxData);
      ROS_INFO("successfully acknowledged get_sensor_value response \n");
    }
    else
    {
       ROS_WARN("failed to get sensor value");
    }
  }
  else if (req.command_type == "GetMotorWheelSpeed")
  {
    int loop_count = 0;
    type = 's';
    id = req.device_id;
    TxData = ((short)type << 8) | ((short)id & 0xff);
    while (loop_count < MAX_LOOP_COUNT)
    {
      ROS_INFO("Sending data");
      zgb_tx_data(TxData);
      for (i = 0; i < 300; i++)
      {
        if (zgb_rx_check() == 1)
        {
          RxData = zgb_rx_data();
          ROS_INFO("got get_motor_wheel_speed response %d\n", RxData);
          res.val = RxData;
          flag = 1;
          break;
        }
        r.sleep();
      }
      if (flag)
        break;

      loop_count++;
    }

    if (loop_count == MAX_LOOP_COUNT)
    {
      ROS_ERROR("Response from robot timed out");
    }
    loop_count = 0;

    if (flag)
    {
      TxData = ACK;
      zgb_tx_data(TxData);
      ROS_INFO("successfully acknowledged get_motor_wheel_speed response \n");
    }
    else
    {
       ROS_WARN("failed to get motor wheel speed");
    }
  }
  else if (req.command_type == "SetMotorTargetPosition")
  {
    type = 'P';
    id = req.device_id;
    TxData = ((short)type << 8) | ((short)id & 0xff);

    int loop_count_one = 0;
    int flag_one = 0;
    int loop_count_two = 0;
    int flag_two = 0;

    while (flag_one == 0 && loop_count_one < MAX_LOOP_COUNT)
    {
      zgb_tx_data(TxData);
      usleep(5);
      for (int idx = 0; idx < 500; idx++)
      {
        if (zgb_rx_check() == 1)
        {
          RxData = zgb_rx_data();
          if (RxData == ACK)
          {
            ROS_INFO("Got first Motor Position ACK back\n");
            flag_one = 1;
            break;
          }
          else
          {
            ROS_WARN("FIRST Motor Position response is not an ACK, as it should be!, it is instead: %d \n", RxData);
            usleep(5);
          }
        }
        r.sleep();
      }
      if (flag_one == 1)
      {
        break;
      }
      loop_count_one++;
    }
    if (flag_one == 1)
    {
      TxData = req.target_val;
      while (flag_two == 0 && loop_count_two < MAX_LOOP_COUNT)
      {
        zgb_tx_data(TxData);
        for (int idx = 0; idx < 300; idx++)
        {
          if (zgb_rx_check() == 1)
          {
            RxData = zgb_rx_data();
            if (RxData == ACK)
            {
              ROS_INFO("Got second Motor Position ACK back\n");
              flag_two = 1;
              break;
            }
            else
            {
              ROS_WARN("SECOND Motor Position response is not an ACK, as it should be!\n");
            }
          }
          r.sleep();
        }
        if (flag_two == 1)
        {
          break;
        }
        loop_count_two++;
      }
    }
    else
    {
        // This is the same as if flag_one was still set to 0
        ROS_ERROR("Couldn't successfully send type/id information!");
    }
    if (flag_two == 0)
    {
        ROS_ERROR("Couldn't successfully send motor position information!");
    }
    res.val = 0;
  }
  else if (req.command_type == "SetMotorTargetSpeed")
  {
    type = 'S';
    id = req.device_id;
    TxData = ((short)type << 8) | ((short)id & 0xff);

    int loop_count_one = 0;
    int flag_one = 0;
    int loop_count_two = 0;
    int flag_two = 0;

    while (flag_one == 0 && loop_count_one < MAX_LOOP_COUNT)
    {
        zgb_tx_data(TxData);
        usleep(5);
        for (int idx = 0; idx < 500; idx++)
        {
            if (zgb_rx_check() == 1)
            {
                RxData = zgb_rx_data();
                if (RxData == ACK)
                {
                    ROS_INFO("Got first Motor Speed ACK back\n");
                    flag_one = 1;
                    break;
                }
                else
                {
                    ROS_WARN("FIRST Motor Speed response is not an ACK, as it should be!, it is instead: %d \n", RxData);
                    usleep(5);
                }
            }
            r.sleep();
        }
        if (flag_one == 1)
        {
            break;
        }
        loop_count_one++;
    }
    if (flag_one == 1)
    {
      TxData = req.target_val;
      while (flag_two == 0 && loop_count_two < MAX_LOOP_COUNT)
      {
        zgb_tx_data(TxData);
        for (int idx = 0; idx < 300; idx++)
        {
          if (zgb_rx_check() == 1)
          {
            RxData = zgb_rx_data();
            if (RxData == ACK)
            {
                ROS_INFO("Got second Motor Speed ACK back\n");
                flag_two = 1;
                break;
            }
            else
            {
                ROS_WARN("SECOND Motor Speed response is not an ACK, as it should be!\n");
            }
          }
          r.sleep();
        }
        if (flag_two == 1)
        {
          break;
        }
        loop_count_two++;
      }
    }
    else
    {
        // This is the same as if flag_one was still set to 0
        ROS_ERROR("Couldn't successfully send type/id information!");
    }
    if (flag_two == 0)
    {
        ROS_ERROR("Couldn't successfully send motor speed information!");
    }
    res.val = 0;
  }
  else if (req.command_type == "SetMotorTargetPositionsSync")
  {
    ROS_INFO("Motor position sync, num ids [%d], (%d: %d), (%d: %d)", req.n_dev, req.dev_ids[0], req.target_vals[0], req.dev_ids[1], req.target_vals[1]);
    type = 'C';
    id = req.device_id;
    TxData = ((short)type << 8) | ((short)req.n_dev & 0xff);

    int loop_count_one = 0;
    int flag_one = 0;
    int loop_count_two = 0;
    int flag_two = 0;

    while (flag_one == 0 && loop_count_one < MAX_LOOP_COUNT)
    {
      zgb_tx_data(TxData);
      usleep(5);
      for (int idx = 0; idx < 500; idx++)
      {
        if (zgb_rx_check() == 1)
        {
          RxData = zgb_rx_data();
          if (RxData == ACK)
          {
            ROS_INFO("Got first data ACK back\n");
            flag_one = 1;
            break;
          }
          else
          {
            ROS_WARN("FIRST Motor Position Sync response is not an ACK, as it should be!, it is instead: %d \n", RxData);
            usleep(5);
          }
        }
        r.sleep();
      }
      if (flag_one == 1)
      {
        break;
      }
      loop_count_one++;
    }
    if (flag_one == 1)
    {
      int n = req.n_dev/2;
      if (req.n_dev != n*2)
        n = n + 1;
      int cnt = 0;

      for (int j = 0; j < n; j++)
      {
        short hi = req.dev_ids[cnt];
        cnt = cnt+1;
        short lo = -1;
        if (cnt < req.n_dev)
        {
          lo = req.dev_ids[cnt];
          cnt = cnt+1;
        }
        TxData = (hi << 8) | (lo & 0xff);
        flag_two = 0;
        loop_count_two = 0;
        while (flag_two == 0 && loop_count_two < MAX_LOOP_COUNT)
        {
          zgb_tx_data(TxData);
          for (int idx = 0; idx < 300; idx++)
          {
            if (zgb_rx_check() == 1)
            {
              RxData = zgb_rx_data();
              if (RxData == ACK)
              {
                ROS_INFO("Got Motor id list ACK back\n");
                flag_two = 1;
                break;
              }
              else
              {
                ROS_WARN("Motor id list response is not an ACK, as it should be!\n");
              }
            }
            r.sleep();
          }
          if (flag_two == 1)
          {
            break;
          }
          loop_count_two++;
        }
      }
      if (flag_two == 0)
      {
        ROS_ERROR("Couldn't successfully send motor id list information!");
        res.val = 0;
        return true;
      }
      loop_count_two = 0;
      flag_two = 0;
      ROS_INFO("sent id list\n");
      for (int j = 0; j < req.n_dev; j++)
      {
        TxData = req.target_vals[j];
        flag_two = 0;
        loop_count_two = 0;
        while (flag_two == 0 && loop_count_two < MAX_LOOP_COUNT)
        {
          zgb_tx_data(TxData);
          for (int idx = 0; idx < 300; idx++)
          {
            if (zgb_rx_check() == 1)
            {
              RxData = zgb_rx_data();
              if (RxData == ACK)
              {
                ROS_INFO("Got Motor target position list ACK back\n");
                flag_two = 1;
                break;
              }
              else
              {
                ROS_WARN("Motor target position list response is not an ACK, as it should be!\n");
              }
            }
            r.sleep();
          }
          if (flag_two == 1)
          {
            break;
          }
          loop_count_two++;
        }
        if (flag_two == 0)
        {
          ROS_ERROR("Couldn't successfully send motor position list information!");
          res.val = 0;
          return true;
        }
      }
      ROS_INFO("sent target values\n");
    }
    else
    {
      // This is the same as if flag_one was still set to 0
      ROS_ERROR("Couldn't successfully send type/id information!");
    }
    if (flag_two == 0)
    {
      ROS_ERROR("Couldn't successfully send motor position list information!");
    }
    // ALEX : reset res.val ? just took this from existing code
    res.val = 0;
  }
  else if (req.command_type == "SetWheelSpeedSync")
  {
    ROS_INFO("Wheel Speed sync, num ids [%d], (%d: %d), (%d: %d)", req.n_dev, req.dev_ids[0], req.target_vals[0], req.dev_ids[1], req.target_vals[1]);
    type = 'Y';
    id = req.device_id;
    TxData = ((short)type << 8) | ((short)req.n_dev & 0xff);

    int loop_count_one = 0;
    int flag_one = 0;
    int loop_count_two = 0;
    int flag_two = 0;

    while (flag_one == 0 && loop_count_one < MAX_LOOP_COUNT)
    {
      zgb_tx_data(TxData);
      usleep(5);
      for (int idx = 0; idx < 500; idx++)
      {
        if (zgb_rx_check() == 1)
        {
          RxData = zgb_rx_data();
          if (RxData == ACK)
          {
            ROS_INFO("Got first data ACK back\n");
            flag_one = 1;
            break;
          }
          else
          {
            ROS_WARN("FIRST Wheel Speed Sync response is not an ACK, as it should be!, it is instead: %d \n", RxData);
            usleep(5);
          }
        }
        r.sleep();
      }
      if (flag_one == 1)
      {
        break;
      }
      loop_count_one++;
    }
    if (flag_one == 1)
    {
      int n = req.n_dev/2;
      if (req.n_dev != n*2)
        n = n + 1;
      int cnt = 0;

      for (int j = 0; j < n; j++)
      {
        short hi = req.dev_ids[cnt];
        cnt = cnt+1;
        short lo = -1;
        if (cnt < req.n_dev)
        {
          lo = req.dev_ids[cnt];
          cnt = cnt+1;
        }
        TxData = (hi << 8) | (lo & 0xff);
        flag_two = 0;
        loop_count_two = 0;
        while (flag_two == 0 && loop_count_two < MAX_LOOP_COUNT)
        {
          zgb_tx_data(TxData);
          for (int idx = 0; idx < 300; idx++)
          {
            if (zgb_rx_check() == 1)
            {
              RxData = zgb_rx_data();
              if (RxData == ACK)
              {
                ROS_INFO("Got Motor id list ACK back\n");
                flag_two = 1;
                break;
              }
              else
              {
                ROS_WARN("Motor id list response is not an ACK, as it should be!\n");
              }
            }
            r.sleep();
          }
          if (flag_two == 1)
          {
            break;
          }
          loop_count_two++;
        }
      }
      if (flag_two == 0)
      {
        ROS_ERROR("Couldn't successfully send motor id list information!");
        res.val = 0;
        return true;
      }
      loop_count_two = 0;
      flag_two = 0;
      ROS_INFO("sent id list\n");
      for (int j = 0; j < req.n_dev; j++)
      {
        TxData = req.target_vals[j];
        flag_two = 0;
        loop_count_two = 0;
        while (flag_two == 0 && loop_count_two < MAX_LOOP_COUNT)
        {
          zgb_tx_data(TxData);
          for (int idx = 0; idx < 300; idx++)
          {
            if (zgb_rx_check() == 1)
            {
              RxData = zgb_rx_data();
              if (RxData == ACK)
              {
                ROS_INFO("Got Wheel Speed list ACK back\n");
                flag_two = 1;
                break;
              }
              else
              {
                ROS_WARN("Wheel Speed  list response is not an ACK, as it should be!\n");
              }
            }
            r.sleep();
          }
          if (flag_two == 1)
          {
            break;
          }
          loop_count_two++;
        }
        if (flag_two == 0)
        {
          ROS_ERROR("Couldn't successfully send Wheel Speed list information!");
          res.val = 0;
          return true;
        }
      }
      ROS_INFO("sent target values\n");
    }
    else
    {
      // This is the same as if flag_one was still set to 0
      ROS_ERROR("Couldn't successfully send type/id information!");
    }
    if (flag_two == 0)
    {
      ROS_ERROR("Couldn't successfully send Wheel Speed list information!");
    }
    // ALEX : reset res.val ? just took this from existing code
    res.val = 0;
  }
  else if (req.command_type == "SetMotorMode")
  {
    type = 'M';
    id = req.device_id;
    TxData = ((short)type << 8) | ((short)id & 0xff);

    int loop_count_one = 0;
    int flag_one = 0;
    int loop_count_two = 0;
    int flag_two = 0;

    while (flag_one == 0 && loop_count_one < MAX_LOOP_COUNT)
    {
        zgb_tx_data(TxData);
        usleep(5);
        for (int idx = 0; idx < 500; idx++)
        {
            if (zgb_rx_check() == 1)
            {
                RxData = zgb_rx_data();
                if (RxData == ACK)
                {
                    ROS_INFO("Got first Motor Mode ACK back\n");
                    flag_one = 1;
                    break;
                }
                else
                {
                    ROS_WARN("FIRST Motor Mode response is not an ACK, as it should be!, it is instead: %d \n", RxData);
                    usleep(5);
                }
            }
            r.sleep();
        }
        if (flag_one == 1)
        {
            break;
        }
        loop_count_one++;
    }
    if (flag_one == 1)
    {
      TxData = req.target_val;
      while (flag_two == 0 && loop_count_two < MAX_LOOP_COUNT)
      {
        zgb_tx_data(TxData);
        for (int idx = 0; idx < 300; idx++)
        {
            if (zgb_rx_check() == 1)
            {
                RxData = zgb_rx_data();
                if (RxData == ACK)
                {
                    ROS_INFO("Got second Motor Mode ACK back\n");
                    flag_two = 1;
                    break;
                }
                else
                {
                    ROS_WARN("SECOND Motor Mode response is not an ACK, as it should be!\n");
                }
            }
            r.sleep();
        }
        if (flag_two == 1)
        {
            break;
        }
        loop_count_two++;
      }
    }
    else
    {
      // This is the same as if flag_one was still set to 0
      ROS_ERROR("Couldn't successfully send type/id information!");
    }
    if (flag_two == 0)
    {
      ROS_ERROR("Couldn't successfully send motor mode information!");
    }
    res.val = 0;
  }
  else if (req.command_type == "SetMotorWheelSpeed") {
    type = 'W';
    id = req.device_id;
    TxData = ((short)type << 8) | ((short)id & 0xff);

    int loop_count_one = 0;
    int flag_one = 0;
    int loop_count_two = 0;
    int flag_two = 0;

    while (flag_one == 0 && loop_count_one < MAX_LOOP_COUNT)
    {
        zgb_tx_data(TxData);
        usleep(5);
        for (int idx = 0; idx < 500; idx++)
        {
            if (zgb_rx_check() == 1)
            {
                RxData = zgb_rx_data();
                if (RxData == ACK)
                {
                    ROS_INFO("Got first Motor Wheel Speed ACK back\n");
                    flag_one = 1;
                    break;
                }
                else
                {
                    ROS_WARN("FIRST Motor Wheel Speed response is not an ACK, as it should be!, it is instead: %d \n", RxData);
                    usleep(5);
                }
            }
            r.sleep();
        }
        if (flag_one == 1) 
        {
            break;
        }
        loop_count_one++;
    }
    if (flag_one == 1)
    {
    TxData = req.target_val;
        while (flag_two == 0 && loop_count_two < MAX_LOOP_COUNT)
        {
            zgb_tx_data(TxData);
            for (int idx = 0; idx < 300; idx++)
            {
                if (zgb_rx_check() == 1)
                {
                    RxData = zgb_rx_data();
                    if (RxData == ACK)
                    {
                        ROS_INFO("Got second Motor Wheel Speed ACK back\n");
                        flag_two = 1;
                        break;
                    }
                    else
                    {
                        ROS_WARN("SECOND Motor Wheel Speed response is not an ACK, as it should be!\n");
                    }
                }
                r.sleep();
            }
            if (flag_two == 1)
            {
                break;
            }
            loop_count_two++;
       }
    }
    else
    {
        // This is the same as if flag_one was still set to 0
        ROS_ERROR("Couldn't successfully send type/id information!");
    }
    if (flag_two == 0)
    {
        ROS_ERROR("Couldn't successfully send motor wheel speed information!");
    }
    res.val = 0;
  }
  else
  {
    ROS_INFO("Unrecognized Command Type\n");
  }
  return true;
}

int main(int argc, char **argv)
{
  short RxData;
  ros::init(argc, argv, "bioloid_driver");
  ros::NodeHandle n;
  int init_loop_count = 0;
  if (zgb_initialize(0) == 0)
  {
    ROS_INFO("Failed to open Serial Connection!\n");
    ROS_INFO("Quitting...\n");
    return 0;
  }
  else
  {
    ROS_INFO("Succeeded in openning Serial Connection!\n");
  }
  ros::Rate rate(50);
  zgb_tx_data(RDY);
  while (init_loop_count < MAX_INIT_LOOP_COUNT)
  {
    if (zgb_rx_check() == 1)
    {
      RxData = zgb_rx_data();
      if (RxData == ACK)
        break;
    }
    init_loop_count++;
    rate.sleep();
  }

  if (init_loop_count == MAX_INIT_LOOP_COUNT)
  {
    ROS_ERROR("Could not establish connection with robot, please restart the robot controller, check the serial connection is made and then restart this node.");
    return 0;
  }

  ROS_INFO("Robot is ready to take commands\n");

  ros::ServiceServer serv = n.advertiseService("allcmd", allcmdCallback);

  ros::spin();
  return 0;
}
