
#include <SerialPort.h>

#include <cstring>
#include <cstdlib>
#include <iostream>
#include <unistd.h>
#include <sstream>
#include <algorithm>
#include <chrono>
#include <thread>
#include <pthread.h>
#include <mutex>
#include <geometry_msgs/Twist.h> 
#include <tf/transform_broadcaster.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include "base_controller.h"
#include "data_type.h"
#include "crc.h"
#include "imu_data_decode.h"
#include "packet.h"

#define DEGREES_TO_RADIANS(angle) ((angle) / 180.0 * M_PI)

using namespace LibSerial;
using namespace std;

// data definition
SerialPort base_serial_port;
SerialPort imu_serial_port;

ros::Publisher pubImu;
ros::Publisher pubOdom;
ros::Subscriber subCmd;
std::mutex serial_mutex;
double max_linear_velocity = 2.0;
double max_angluar_velocity = 10.0;
int encoder_resolution = 4096;
int cmd_time_out_counter_limit = 20;
double cmd_time_out = 100.0;
double publish_rate = 50.0;
unsigned long cmd_time_couter = 0;
bool start_cmd = false;
double controller_rate  = 20;
bool publish_tf = true;
namespace base_controller
{


Imu_t imu_info;
motor_cmd_t motor_cmd;
DataBuffer cmd_buffer;
geometry_msgs::Twist motorSpeedSend;

DataBuffer motor_cmd_buffer;

BaseController base_control;

void init_cmd_variable();
void init_serial(const char *fd);
void send_velocity_to_base(const double v, const double w);
void send_cmd_to_base( const uint16 frame_type, const uint16 cmd);
void motion_dmd_callback(const ros::TimerEvent&);
void odometry_data_processing(const uint8 *buff,const uint32 len);
void mcu_cmd_processing(const uint8 *buff,const uint32 len);
void mcu_ack_processing(const uint8 *buff,const uint32 len);

void init_cmd_variable()
{
    motor_cmd.sync1 = SYNC1;
    motor_cmd.sync2 = SYNC2;
    motor_cmd.frame_type = MOTOR_CMD_TYPE;
    motor_cmd.seq = 0;
    motor_cmd.left_motor = 0;
    motor_cmd.right_motor= 0;
    motor_cmd.crc =0;
    motor_cmd.tail1 = TAIL1;
    motor_cmd.tail2 = TAIL2;
    motor_cmd_buffer.resize(MOTOR_CMD_LENGTH);

    cmd_time_out_counter_limit =(int)(cmd_time_out * controller_rate/1000.0 );
    send_cmd_to_base(CMD_BASE_TO_MCU_FRAME, CMD_RESET_ODOM);
    send_cmd_to_base(CMD_BASE_TO_MCU_FRAME, CMD_UNLOCK_MOTOR);
}

void init_serial(const char *fd, SerialPort& serial_port)

{
    // Open the Serial Port at the desired hardware port.
    serial_port.Open(fd);

    // Set the baud rate of the serial port.
    serial_port.SetBaudRate(BaudRate::BAUD_115200);

    // Set the number of data bits.
    serial_port.SetCharacterSize(CharacterSize::CHAR_SIZE_8);

    // Turn off hardware flow control.
    serial_port.SetFlowControl(FlowControl::FLOW_CONTROL_NONE);

    // Disable parity.
    serial_port.SetParity(Parity::PARITY_NONE);
    
    // Set the number of stop bits.
    serial_port.SetStopBits(StopBits::STOP_BITS_1);
    serial_port.FlushIOBuffers();
}
void send_velocity_to_base(const double v, const double w)
{
    static uint32 seq = 0;
    motor_cmd.seq = seq++;
    double left_motor_rpm_speed =0., right_motor_rpm_speed = 0.;
    base_control.computeMotorCmd(v, w,left_motor_rpm_speed,right_motor_rpm_speed);
    uint16 crc = getCrc16((uint8 *)&motor_cmd.frame_type,(MOTOR_CMD_LENGTH -6));
    motor_cmd.left_motor = (int16) left_motor_rpm_speed;
    motor_cmd.right_motor = (int16) right_motor_rpm_speed;
    motor_cmd.crc = crc;
    memcpy((uint8 *)&motor_cmd_buffer[0],(uint8 *)&motor_cmd, MOTOR_CMD_LENGTH);
    {
        //unique_lock<std::mutex> lock(serial_mutex);
        base_serial_port.Write(motor_cmd_buffer);
        // Wait until the data has actually been transmitted.
        base_serial_port.DrainWriteBuffer();
        //lock.unlock();
        printf("cmd %d  %d  %d  \n",seq, motor_cmd.left_motor, motor_cmd.right_motor);
    }

}

void send_cmd_to_base( const uint16 frame_type, const uint16 cmd)
{
    DataBuffer cmd_buffer;
    cmd_buffer.resize(CMD_LENGTH);
    cmd_t base_cmd;
    base_cmd.sync1 = SYNC1;
    base_cmd. sync2 = SYNC2;
    base_cmd. frame_type = frame_type;
    base_cmd. cmd = cmd;
    base_cmd. length = CMD_DATA_LENGTH;
    uint16 crc = getCrc16((uint8 *)&base_cmd.frame_type,(CMD_LENGTH -6));
    base_cmd. crc = crc;
    base_cmd. tail1 = TAIL1;
    base_cmd. tail2 = TAIL2;
    memcpy((uint8 *)&cmd_buffer[0],(uint8 *)&base_cmd, CMD_LENGTH);
   // for(int i= 0;i<CMD_LENGTH;i++)
   //     printf("%x ",cmd_buffer[i]);
   // printf("crc %x  \n",crc);
    {
        unique_lock<std::mutex> lock(serial_mutex);
        base_serial_port.Write(cmd_buffer);
        // Wait until the data has actually been transmitted.
        base_serial_port.DrainWriteBuffer();
        lock.unlock();
        usleep(1000);

    }
}


void motion_dmd_callback(const ros::TimerEvent&)
{
    if(start_cmd)
    {
        if(cmd_time_couter <=cmd_time_out_counter_limit)
        {
            double v = motorSpeedSend.linear.x, w = motorSpeedSend.angular.z;
            //check velocity limits
            if( v > max_linear_velocity )
            {
                v = max_linear_velocity;
            }
            else if( v < - max_linear_velocity)
            {
                v = -max_linear_velocity;
            }

            if(w > max_angluar_velocity)
            {
                w = max_angluar_velocity;
            }
            else if(w < - max_angluar_velocity)
            {
                w = -max_angluar_velocity;
            }
            send_velocity_to_base(v,w);

        }
        else if(cmd_time_out_counter_limit< cmd_time_couter&&
                cmd_time_couter <=(cmd_time_out_counter_limit + controller_rate))
        {
            //timeout processing, reset the speed and send 0 to base
            motorSpeedSend.linear.x = 0.0;
            motorSpeedSend.angular.z = 0.0;
            send_velocity_to_base(0.0,0.0);

        }
        else if(cmd_time_couter >(cmd_time_out_counter_limit + controller_rate)&&
		cmd_time_couter <=(cmd_time_out_counter_limit + controller_rate + 5))
        {
            send_cmd_to_base(CMD_BASE_TO_MCU_FRAME, CMD_UNLOCK_MOTOR);

        }
        else
        {
            // send heartbeaten clock
            
        }
     //   printf("counter %ld  %d", cmd_time_couter, cmd_time_out_counter_limit);

        cmd_time_couter++;
    }
}
//recv cmd_vel topic and write it to serial imdeaitely.
void cmd_vel_call_back(const geometry_msgs::Twist::ConstPtr& motorSpeed)
{
    cmd_time_couter = 0;
    start_cmd = true;
    motorSpeedSend.linear.x = motorSpeed->linear.x;
    motorSpeedSend.angular.z = motorSpeed->angular.z;


}
// process odomety data
bool imu_data_processing(const double& acc_x, const double& acc_y, const double& acc_z,
                         const double& gyro_x, const double& gyro_y, const double& gyro_z,
                         const double&w, const double&x, const double& y, const double &z)
{
    const double orientation_covariance[9] = {DEGREES_TO_RADIANS(0.01), 0, 0,
                                       0, DEGREES_TO_RADIANS(0.01), 0,
                                       0, 0., DEGREES_TO_RADIANS(0.09)};
    const double angular_velocity_covariance[9] = { DEGREES_TO_RADIANS(0.0025), 0, 0,
                                             0, DEGREES_TO_RADIANS(0.0025), 0,
                                             0, 0, DEGREES_TO_RADIANS(0.0025)};
    const double linear_acceleration_covariance[9] = {0.0004, 0, 0,
                                            0, 0.0004, 0,
                                            0, 0, 0.0004};

    sensor_msgs::Imu imu_msg;
    imu_msg.header.stamp = ros::Time::now();
    imu_msg.header.frame_id = "imu_link";
    imu_msg.orientation.x = x;
    imu_msg.orientation.y = y;
    imu_msg.orientation.z = z;
    imu_msg.orientation.w = w;

    imu_msg.linear_acceleration.x = acc_x;
    imu_msg.linear_acceleration.y = acc_y;
    imu_msg.linear_acceleration.z = acc_z;

    imu_msg.angular_velocity.x= gyro_x;
    imu_msg.angular_velocity.y= gyro_y;
    imu_msg.angular_velocity.z= gyro_z;

    for(int i=0; i< sizeof(orientation_covariance)/sizeof(double); i++)
    {
        imu_msg.orientation_covariance[i] = orientation_covariance[i];
    }

    for(int i=0; i< sizeof(angular_velocity_covariance)/sizeof(double); i++)
    {
        imu_msg.angular_velocity_covariance[i] = angular_velocity_covariance[i];
    }
    for(int i=0; i< sizeof(linear_acceleration_covariance)/sizeof(double); i++)
    {
        imu_msg.linear_acceleration_covariance[i] = linear_acceleration_covariance[i];
    }

    pubImu.publish(imu_msg);

}

void mcu_cmd_processing(const uint8 *buff,const uint32 len)
{
    const uint16 cmd =buff[5]*256 + buff[4];
    printf("%x \n",cmd);

    cmd_t mcu_cmd;
    if(len == sizeof(mcu_cmd)){
        memcpy((void *)&mcu_cmd, buff, len);
    }
   // else
    {
        for(int i=0;i<len;i++)
        {
            printf("%x ",buff[i]);
        }
        printf("\n");
        std::cout << "len " << len << "size " << sizeof(mcu_cmd) <<std::endl;
    //    return;
    
    }
    switch(cmd)
    {
    case CMD_POWER_ON:
        //send acklowedgement
        std::cout << " CMD_POWER_ON " <<std::endl;

        send_cmd_to_base(CMD_BASE_TO_MCU_ACK_FRAME, CMD_POWER_ON);
        break;

    case CMD_POWER_OFF:
        std::cout << " CMD_POWER_OFF " <<std::endl;

        send_cmd_to_base(CMD_BASE_TO_MCU_ACK_FRAME, CMD_POWER_OFF);

        break;
    case CMD_EMERGENCY_ON:
        std::cout << " CMD_EMERGENCY_ON " <<std::endl;

        send_cmd_to_base(CMD_BASE_TO_MCU_ACK_FRAME, CMD_EMERGENCY_ON);

        break;
    case CMD_EMERGENCY_OFF:
        std::cout << " CMD_EMERGENCY_OFF " <<std::endl;
        send_cmd_to_base(CMD_BASE_TO_MCU_ACK_FRAME, CMD_EMERGENCY_OFF);
        //TO DO: cancel navigation task and send remove emergency status to mcu
        send_cmd_to_base(CMD_BASE_TO_MCU_FRAME, CMD_REMOVE_EMERENCY);
	break;
    case CMD_BUMPER_SWITCH_ON:
        std::cout << " CMD_BUMPER_SWITCH_ON " <<std::endl;
        send_cmd_to_base(CMD_BASE_TO_MCU_ACK_FRAME, CMD_BUMPER_SWITCH_ON);
	break;
    case CMD_BATTERY:
        std::cout << " CMD_BATTERY " <<std::endl;
        send_cmd_to_base(CMD_BASE_TO_MCU_ACK_FRAME, CMD_BATTERY);
	break;       
    case CMD_MOTOR_STATUS:
        std::cout << " CMD_MOTOR_STATUS " <<std::endl;
        send_cmd_to_base(CMD_BASE_TO_MCU_ACK_FRAME, CMD_MOTOR_STATUS);        

        break;
    default:
        break;
    }

}
void mcu_ack_processing(const uint8 *buff,const uint32 len)
{
    const uint16 cmd =buff[5]*256 + buff[4];

    cmd_t mcu_ack_cmd;
    if(len == sizeof(mcu_ack_cmd)){
        memcpy((void *)&mcu_ack_cmd, buff, len);
    }
    else
    {
        return;
    
    }

    switch(cmd)
    {
    case CMD_LOCK_MOTOR:
        //send acklowedgement
        std::cout << " CMD_LOCK_MOTOR ACK" <<std::endl;
        break;
    case CMD_UNLOCK_MOTOR:
        std::cout << " CMD_UNLOCK_MOTOR ACK" <<std::endl;
        break;
    case CMD_RESET_ODOM:
        std::cout << " CMD_RESET_ODOM ACK" <<std::endl;
        break;
    case CMD_CLEAR_ALARM:
        std::cout << " CMD_CLEAR_ALARM ACK " <<std::endl;
        break;
    case CMD_REMOVE_EMERENCY:
        std::cout << " CMD_REMOVE_EMERENCY ACK " <<std::endl;
        break;
    default:
        break;
    }



}
// convert raw encoder data to pose in rad
void convert_encoder_to_pose(const int32& left_encoder, const int32& right_encoder, double *pose)
{  
    int32 delta_encoder[2]={0,0};
    int32 current_encoder[2]={left_encoder, right_encoder};
    //when encoder counter overflow, cycle_pose will increase
    static double cycle_pose[2]={0., 0.};
    //store last encoder to detect encoder counter overflow
    static int32 last_encoder[2] = {0};
    for(int i = 0;i < 2;i++)
    {
        delta_encoder[i] = current_encoder[i] -last_encoder[i];
        //encoder from max to  begin
        if(delta_encoder[i] < 0 && abs(delta_encoder[i])>(ENCODER_MAX / 2.0))
        {

            cycle_pose[i] += (double)ENCODER_MAX / encoder_resolution * 2 * PI;

        }
        //encoder from min to begin
        else if(delta_encoder[i]>0 && abs(delta_encoder[i])>(-ENCODER_MIN /2.0))
        {
            cycle_pose[i] += (double)ENCODER_MIN / encoder_resolution * 2 * PI;
        }
        // the total distance is cycle_pose + current
        pose[i] = cycle_pose[i] + ((double)current_encoder[i]) / encoder_resolution * 2 * PI;
        last_encoder[i] = current_encoder[i];
    }

    //

}
// convert gps utc time to rostime
void convert_utctime_to_rostime(ros::Time &ros_time)
{
    //temp
    ros_time = ros::Time::now();

}
void odometry_data_processing(const uint8 *buff,const uint32 len)
{
    Odometry_t odom;
    if(len == ODOM_TOTAL_LENGTH){
       memcpy((void *)&odom, buff, len);     
    }
    else
    {
        return;
    }
    //std::cout<< "encoder " << ' '<< odom.seq<<' '<< odom.timestamp<<' '
    //         <<odom.millisecs<<' '<< odom.encoder_A<<' '<< odom.encoder_B<<std::endl;
    nav_msgs::Odometry odom_msg;
    odom_msg.pose.pose.orientation.w =1.0;
    double pose[2]={0,0};
    ros::Time timestamp;
    //duration
    const ros::Duration period(0.02);
    //convert encoder counter to pose in rad
    convert_encoder_to_pose((const int)(odom.encoder_A), (const int )(odom.encoder_B), pose);

    //convert utc timestamp to ros time
    convert_utctime_to_rostime(timestamp);
    //
    //update odometry
    base_control.update(timestamp, period , pose[0], pose[1]);
    //get odometry
    base_control.getOdometryMsg(odom_msg);
    odom_msg.header.seq = odom.seq;
    odom_msg.header.frame_id ="odom";
    odom_msg.child_frame_id="base_link";


    //publish msg
    pubOdom.publish(odom_msg);
    if(publish_tf){
        //publish tf
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, 0.0) );
        tf::Quaternion q (odom_msg.pose.pose.orientation.x,
                          odom_msg.pose.pose.orientation.y,
                          odom_msg.pose.pose.orientation.z,
                          odom_msg.pose.pose.orientation.w);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, timestamp, "odom", "base_link"));
    }

}


// thread to recv serial data 
void *serial_recv_thread(void *arg)
{
    // Timeout value in milliseconds to wait for data being read.
    size_t ms_timeout = 1;
    const int max_buffer_size = 50;
    uint8 buffer[max_buffer_size];//={0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08};
    memset(buffer,0,max_buffer_size);
    uint8 *ptrHead = &buffer[0];
    uint8 *ptrTail= &buffer[0];
    uint8 *ptr = &buffer[0];
    base_control.starting(ros::Time::now());
    while(1)
    {
        // Read a single byte of data from the serial port.
        try
        {
            unique_lock<std::mutex> lock(serial_mutex);
            base_serial_port.ReadByte(*ptr, ms_timeout);
            //lock.unlock();
        }
        catch (ReadTimeout)
        {

            continue;
        }
        //serial_port.Read(read_buffer[0], nRead, buffer_size, ms_timeout);
        if(*ptr == SYNC2 && *(ptrHead-1) == SYNC1)
        {
            ptrHead = ptr -1;
        }
        if(*ptr == TAIL2 && *(ptr -1)== TAIL1)
        {
            ptrTail = ptr - 1;
        }
        if(ptrHead[1] == SYNC2 && ptrTail[1] == TAIL2 &&
                *ptr == TAIL2)
        {
            const uint8 frame_type = ptrHead[2];
            const uint8 ckA = *(ptrTail -2);
            const uint8 ckB = *(ptrTail -1);
            const uint32 data_len = (ptrTail-2 - (ptrHead+2));
            const uint32 frame_len = ptrTail + 2 - ptrHead;
            //check crc
            bool ret = CheckCrc16(ptrHead+2, data_len, ckA , ckB);
            if(ret)
            {
                switch(frame_type)
                {
                case ODOM_ID:
                    //process pipeline
                    //imdeiately copy the data to odometry data and then reset the buffer.
                    odometry_data_processing(ptrHead,frame_len);
                    break;
                case IMU_ID:
                   // memcpy((void *)&imu_info, ptrHead, IMU_TOTAL_LENGTH);
                   // imu_data_processing(&imu_info);
                    break;
                case CMD_MCU_TO_BASE_ACK_FRAME:
                    mcu_ack_processing(ptrHead,frame_len);
                    break;
                case CMD_MCU_TO_BASE_FRAME:
                    //receive ack
                    std::cout << "receive mcu cmd \n" <<std::endl;
                    //receive cmd from mcu to base
                    mcu_cmd_processing(ptrHead,frame_len);
                    break;

                default:
                    memset(&buffer[0], 0, max_buffer_size);
                    ptr = &buffer[0];                    
                    break;

                }
                memset(&buffer[0], 0, max_buffer_size);
                ptr = &buffer[0];
                continue;
            }

        }
        ptr++;
        // to avoid overflow
        if((ptr - buffer) >= (max_buffer_size -1))
        {
            memset(&buffer[0], 0, max_buffer_size);
            ptr = &buffer[0];
        }
        //10us to receive the next byte
        usleep(10);


    }//end of while
    // Successful program completion.
    std::cout << "The program successfully completed!" << std::endl;

}


// thread to recv serial data 
void *imu_serial_recv_thread(void *arg)
{
    // Timeout value in milliseconds to wait for data being read.
    size_t ms_timeout = 1;
    const int max_buffer_size = 100;
    uint8 buffer[max_buffer_size];//={0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08};
    memset(buffer,0,max_buffer_size);
    uint8 *ptrHead = &buffer[0];
    uint8 *ptrTail= &buffer[0];
    uint8 *ptr = &buffer[0];
    static int16 acc[3];
    double acc_data[3];
    static int16 gyro[3];
    double gyro_data[3];
    static int16 mag[3];
    static float eular[3];
    static float quat[4];
    static uint8 id;
    imu_data_decode_init();
    while(1)
    {
        // Read a single byte of data from the serial port.
        try
        {
            imu_serial_port.ReadByte(*ptr, ms_timeout);
            //printf("%x ",ch );
        }
        catch (ReadTimeout)
        {

            continue;
        }
        if(*ptr == 0xA5 &&(*(ptr-1) ==0x5A))
        {
            ptrHead = ptr -1;
        }
        if(*ptr == 0xD1&&(ptr -ptrHead)==29)
        {
            ptrTail = ptr;
        }
        if(ptrHead[1] == 0xA5&&(ptrTail -ptrHead)==29 && (ptr - ptrTail)==16)
        {   
            int len = ptr - ptrHead;
            //cout<< "imu len " << len<<std::endl;
            for(int i =0; i<=len; i++){
                uint8 ch = ptrHead[i];
                //printf("%x ",ch);
                Packet_Decode(ch);
            }
            //printf("\n");
            get_raw_acc(acc);
            get_raw_gyo(gyro);
            get_raw_mag(mag);
            get_eular(eular);
            get_quat(quat);
            get_id(&id);
            
            //printf("id:%d\r\n", ID);
            for(int i =0; i<3; i++)
            {
                acc_data[i] = acc[i] * 0.001f * GRAVITY;
                gyro_data[i] = gyro[i] * 0.1f * PI /180.0f; 
            }
            //printf("Acc: %.5f %.5f %.5f\r\n", acc_data[0], acc_data[1], acc_data[2]);
           // printf("Gyo: %.5f %.5f %.5f\r\n", gyro_data[0], gyro_data[1], gyro_data[2]);
            //printf("Mag: %d %d %d\r\n", mag[0], mag[1], mag[2]);
           // printf("Eular(P R Y):    %0.2f %0.2f %0.2f\r\n", eular[0], eular[1], eular[2]);
           // printf("Quat(W X Y Z):   %0.3f %0.3f %0.3f %0.3f\r\n", quat[0], quat[1], quat[2], quat[3]);
            imu_data_processing(acc_data[0], acc_data[1], acc_data[2],
                                gyro_data[0], gyro_data[1], gyro_data[2],
                                quat[0],quat[1],quat[2],quat[3]);
            memset(&buffer[0], 0, max_buffer_size);
            ptr = &buffer[0];
            continue;


        }
        ptr++;
        // to avoid overflow
        if((ptr - buffer) >= (max_buffer_size -1))
        {
            memset(&buffer[0], 0, max_buffer_size);
            ptr = &buffer[0];
        }
        //10us to receive the next byte
        usleep(10);


    }//end of while
    // Successful program completion.

}

} //end of namespace


int main(int argc, char **argv)
{
    ros::init(argc, argv, "base_node");
    ros::NodeHandle nh("base_control");

    subCmd = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 10, base_controller::cmd_vel_call_back);
    pubImu = nh.advertise<sensor_msgs::Imu>("/imu_raw", 30);
    pubOdom = nh.advertise<nav_msgs::Odometry>("/odom", 5);
    
    std::string port_name;
    std::string imu_port_name;
    bool use_base = true;
    bool use_imu = false;
    nh.param<std::string>("/imu_port_name", imu_port_name, "/dev/ttyUSB0");
    nh.param<std::string>("/base_port_name", port_name, "/dev/ttyUSB0");
    nh.param<double>("/max_linear_velocity", max_linear_velocity, 1.5);
    nh.param<double>("/max_angular_velocity", max_angluar_velocity, 5.0);
    nh.param<int>("/encoder_resolution", encoder_resolution, 4096);
    nh.param<double>("/cmd_time_out", cmd_time_out, 1000);
    nh.param<double>("/publish_rate", publish_rate, 50);
    nh.param<double>("/controller_rate", controller_rate, 20);
    nh.param<double>("/controller_rate", controller_rate, 20);
    nh.param<bool>("/use_base", use_base, true);
    nh.param<bool>("/use_imu", use_imu, false);
    nh.param<bool>("/publish_tf", publish_tf, true);


    base_controller::base_control.init(nh);
    ROS_INFO("encoder_resolution  %d ",encoder_resolution);

    ROS_INFO("max_linear_velocity  %f ",max_linear_velocity);
    ROS_INFO("max_angluar_velocity  %f ",max_angluar_velocity);

    ROS_INFO("open device %s ",port_name.c_str());
    ros::Timer timer = nh.createTimer(ros::Duration(1.0/controller_rate), base_controller::motion_dmd_callback);

    //inital serial and recv thread
    if(use_base){
        base_controller::init_serial(port_name.c_str(),base_serial_port);
        base_controller::init_cmd_variable();
        pthread_t Serial_thread;
        int ret = pthread_create(&Serial_thread, NULL,base_controller::serial_recv_thread, NULL);
        if (ret) {
            std::cout<<"ERROR; return code from pthread_create() is %d\n"<< ret<<std::endl;
        }
    }

    if(use_imu){
        base_controller::init_serial(imu_port_name.c_str(),imu_serial_port);

        pthread_t Imu_Serial_thread;
        int ret_imu = pthread_create(&Imu_Serial_thread, NULL,base_controller::imu_serial_recv_thread, NULL);
        if (ret_imu) {
            std::cout<<"ERROR; return code from pthread_create() is %d\n"<< ret_imu<<std::endl;
        }
    }
    ros::AsyncSpinner ros_spinner(4);
    ros_spinner.start();

    while(ros::ok())
    {
        //processing
        // base_controller::send_cmd_to_base(CMD_BASE_TO_MCU_FRAME, CMD_RESET_ODOM);

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    ros_spinner.stop();
    base_serial_port.Close();
    imu_serial_port.Close();

    return 0;

}

