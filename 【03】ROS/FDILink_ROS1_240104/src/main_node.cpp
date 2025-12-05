#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>
#include "fdilink_decode.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include <string>
#include <iomanip>

extern "C" int InuptChar(uint8_t c);

extern FDILink_IMUData_Packet_t IMU_Recive;
extern FDILink_INSGPSData_Packet_t INSGPS_Recive;
extern FDILink_AHRSData_Packet_t AHRS_Recive;
extern Raw_GNSS_Packet_t Raw_GNSS_Recive;
extern GNSS_DUAL_ANT_Data_Packet_t GNSS_DUAL_ANT_Data_Recive;

ros::Publisher pub_imu; //imu msg publisher
ros::Publisher pub_mag; //magnetometer msg publisher

int main(int argc,char** argv)
{
    std::cout << "========================"  << std::endl
              << "Ros wrapper for FDI Sensor"<< std::endl
              << "it contains a accelerameter, gyroscope and magnetometer" << std::endl
              << "========================" << std::endl;

    ros::init(argc, argv, "launch_imu_node");
    ros::NodeHandle nh("~"); 

    //read serial port param
    std::string IMU_SERIAL_PORT("/dev/ttyUSB0");
    std::string FDI_IMU_TOPIC("/fdi_imu");
    std::string FDI_MAG_TOPIC("/fdi_mag");

    /* //Get Param的三种方法
    //① ros::param::get()获取参数“param1”的value，写入到param1上
    bool test1 = ros::param::get("param1", param1);
    
    //② ros::NodeHandle::getParam()获取参数，与①作用相同
    bool test2 = nh.getParam("param2",param2);
    
    //③ ros::NodeHandle::param()类似于①和②
    //但如果get不到指定的param，它可以给param指定一个默认值(如1)
    nh.param("param3", param3, 1); */

    nh.param("FDI_IMU_TOPIC",FDI_IMU_TOPIC);
    nh.param("FDI_MAG_TOPIC",FDI_MAG_TOPIC);

    
    if(!nh.getParam("imu_serial_port",IMU_SERIAL_PORT))
    {
        ROS_WARN_STREAM("The imu serial port not set, use default: /dev/ttyUSB0");
    }

    pub_imu = nh.advertise<sensor_msgs::Imu>(FDI_IMU_TOPIC,1000);
    pub_mag = nh.advertise<sensor_msgs::MagneticField>(FDI_MAG_TOPIC,1000);
    
    serial::Serial sp; //声明串口对象
    serial::Timeout to = serial::Timeout::simpleTimeout(100);
    sp.setPort(IMU_SERIAL_PORT); //此处需要根据imu的接的串口来进行修改
    sp.setBaudrate(921600);//波特率
    sp.setTimeout(to);
    try
    {
        sp.open();
    }
    catch(serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port " << IMU_SERIAL_PORT << ", may be a wrong port number or permission denied");
        return -1;
    }
    if(sp.isOpen()) //检测串口是否已经打开，并给出提示
    {
        ROS_INFO_STREAM( 
            IMU_SERIAL_PORT << " is opened. ");
        
    }
    else
    {
        return -1;
    }
    ros::Rate loop_rate(100);
    int in;
    while(ros::ok())
    {
        size_t n = sp.available();
        if(n!=0)
        {
            uint8_t buffer[256];
            n = n > 256 ? 256 : n;
            n = sp.read(buffer, n);
            for(int i=0;i<n;i++)
            {
                //0x40
                in = InuptChar(buffer[i]);
                if(in == FDILINK_IMUDATA_PACKET_ID)
                {
                    std::cout 
                      << "[" <<IMU_Recive.Accelerometer_X 
                      << "," <<IMU_Recive.Accelerometer_Y 
                      << "," <<IMU_Recive.Accelerometer_Z 
                      << "," <<IMU_Recive.Gyroscope_X
                      << "," <<IMU_Recive.Gyroscope_Y
                      << "," <<IMU_Recive.Gyroscope_Z
                      << "," <<IMU_Recive.Magnetometer_X
                      << "," <<IMU_Recive.Magnetometer_Y
                      << "," <<IMU_Recive.Magnetometer_Z
                      << "]" << std::endl;

                    //assemble ros msgs 
                    sensor_msgs::Imu imu_msg;
                    sensor_msgs::MagneticField mag_msg;
                    ros::Time time = ros::Time::now();
                    //double secs = time.toSec();
                    //std::cout << std::setprecision(20) << "shijian" << secs << std::endl;
                    imu_msg.header.stamp = time;
                    imu_msg.header.frame_id = "fdi_imu";
                    imu_msg.angular_velocity.x = IMU_Recive.Gyroscope_X;
                    imu_msg.angular_velocity.y = IMU_Recive.Gyroscope_Y;
                    imu_msg.angular_velocity.z = IMU_Recive.Gyroscope_Z;
                    imu_msg.linear_acceleration.x = IMU_Recive.Accelerometer_X;
                    imu_msg.linear_acceleration.y = IMU_Recive.Accelerometer_Y;
                    imu_msg.linear_acceleration.z = IMU_Recive.Accelerometer_Z;
                    
                    mag_msg.header = imu_msg.header;
                    mag_msg.magnetic_field.x = IMU_Recive.Magnetometer_X;
                    mag_msg.magnetic_field.y = IMU_Recive.Magnetometer_Y;
                    mag_msg.magnetic_field.z = IMU_Recive.Magnetometer_Z;

                    pub_imu.publish(imu_msg);
                    pub_mag.publish(mag_msg);
                    
                }

                //0x42
                else if(in == FDILINK_INSGPSDATA_PACKET_ID)
                {
                    std::cout 
                      << "[" <<INSGPS_Recive.BodyVelocity_X
                      << "," <<INSGPS_Recive.BodyVelocity_Y
                      << "," <<INSGPS_Recive.BodyVelocity_Z
                      << "," <<INSGPS_Recive.BodyAcceleration_X
                      << "," <<INSGPS_Recive.BodyAcceleration_Y
                      << "," <<INSGPS_Recive.BodyAcceleration_Z
                      << "," <<INSGPS_Recive.Location_North
                      << "," <<INSGPS_Recive.Location_East
                      << "," <<INSGPS_Recive.Velocity_Down
                      << "," <<INSGPS_Recive.Velocity_North
                      << "," <<INSGPS_Recive.Velocity_East
                      << "," <<INSGPS_Recive.Velocity_Down
                      << "," <<INSGPS_Recive.Acceleration_North
                      << "," <<INSGPS_Recive.Acceleration_East
                      << "," <<INSGPS_Recive.Acceleration_Down
                      << "," <<INSGPS_Recive.Pressure_Altitude
                      << "," <<INSGPS_Recive.Timestamp
                      << "]" << std::endl;
                }

                //0x59
                else if(in == Raw_GNSS_Packet_ID)
                {
                    std::cout 
                      << "[" <<Raw_GNSS_Recive.Unix_time_stamp
                      << "," <<Raw_GNSS_Recive.Microseconds
                      << "," <<Raw_GNSS_Recive.Latitude
                      << "," <<Raw_GNSS_Recive.Longitude
                      << "," <<Raw_GNSS_Recive.Height
                      << "," <<Raw_GNSS_Recive.Velocity_north
                      << "," <<Raw_GNSS_Recive.Velocity_east
                      << "," <<Raw_GNSS_Recive.Velocity_down
                      << "," <<Raw_GNSS_Recive.Latitude_standard_deviation
                      << "," <<Raw_GNSS_Recive.Longitude_standard_deviation
                      << "," <<Raw_GNSS_Recive.Height_standard_deviation
                      << "," <<Raw_GNSS_Recive.Reserved1
                      << "," <<Raw_GNSS_Recive.Reserved2
                      << "," <<Raw_GNSS_Recive.Reserved3
                      << "," <<Raw_GNSS_Recive.Reserved4
                      << "," <<Raw_GNSS_Recive.Status
                      << "]" << std::endl;
                }
                //0x78
                else if(in == GNSS_DUAL_ANT_Data_Packet_ID)
                {
                    std::cout 
                      << "[" <<GNSS_DUAL_ANT_Data_Recive.Microseconds
                      << "," <<GNSS_DUAL_ANT_Data_Recive.RoverRtkRefPosN
                      << "," <<GNSS_DUAL_ANT_Data_Recive.RoverRtkRefPosE
                      << "," <<GNSS_DUAL_ANT_Data_Recive.RoverRtkRefPosD
                      << "," <<GNSS_DUAL_ANT_Data_Recive.MBRtkRefPosN
                      << "," <<GNSS_DUAL_ANT_Data_Recive.MBRtkRefPosE
                      << "," <<GNSS_DUAL_ANT_Data_Recive.MBRtkRefPosD
                      << "," <<GNSS_DUAL_ANT_Data_Recive.RoverLat
                      << "," <<GNSS_DUAL_ANT_Data_Recive.RoverLon
                      << "," <<GNSS_DUAL_ANT_Data_Recive.RoverAlt
                      << "," <<GNSS_DUAL_ANT_Data_Recive.Rover_hAcc
                      << "," <<GNSS_DUAL_ANT_Data_Recive.Rover_vAcc
                      << "," <<GNSS_DUAL_ANT_Data_Recive.MBLat
                      << "," <<GNSS_DUAL_ANT_Data_Recive.MBLon
                      << "," <<GNSS_DUAL_ANT_Data_Recive.MBAlt
                      << "," <<GNSS_DUAL_ANT_Data_Recive.MBhAcc
                      << "," <<GNSS_DUAL_ANT_Data_Recive.MBvAcc
                      << "," <<GNSS_DUAL_ANT_Data_Recive.RoverRtkPosLength
                      << "," <<GNSS_DUAL_ANT_Data_Recive.RoverRtkAccuracyLength
                      << "," <<GNSS_DUAL_ANT_Data_Recive.RoverRtkPosHeading
                      << "," <<GNSS_DUAL_ANT_Data_Recive.RoverRtkAccuracyHeading
                      << "," <<GNSS_DUAL_ANT_Data_Recive.MBfixtype
                      << "," <<GNSS_DUAL_ANT_Data_Recive.Roverfixtype
                      << "]" << std::endl;
                }
                
            }
        }
        loop_rate.sleep();
    }

    return 0;
}