#include <string>
#include <ros/ros.h>
#include <serial/serial.h>

#include <sensor_msgs/Imu.h>
#include <std_msgs/Header.h>

#include <boost/algorithm/string/predicate.hpp>
#include <boost/foreach.hpp>
#include <boost/timer.hpp>
#include <arpa/inet.h>

const int DATA_LENGTH = 16;
const uint8_t GYRO_DREG_ADDR = 0x61;
const uint8_t ACC_DREG_ADDR  = 0x65;
const double GYRO_SCALE = 0.0174533;
const double ACC_SCALE  = 9.80665;

//params
int baud_rate_;
int data_rate_;
std::string port_;
bool is_ned_to_nwu_;//North_East_Down to North_West_Up

//flags and global containers
double time_offset_;
bool is_set_offset_ = false;
uint8_t ignore_begin_num_ = 100, ignore_begin_cnt_ = 0;

uint32_t imu_data_[8];//[gyro_x, gyro_y, gyro_z, gyro_stamp, acc_x, acc_y, acc_z, acc_stamp]
float float_imu_data_[8];
unsigned char  char_imu_data_[16];

serial::Serial ser_;
ros::Publisher imu_pub_;


void read_param(ros::NodeHandle &nh)
{
	nh.param("port", port_, std::string("/dev/ttyUSB0"));
	nh.param("baud_rate", baud_rate_, 115200);
	nh.param("data_rate", data_rate_, 100);
    nh.param("is_ned_to_nwu", is_ned_to_nwu_, true);
}

void init_serial()
{
	ser_.setPort(port_);
	ser_.setBaudrate(baud_rate_);
	serial::Timeout time_out = serial::Timeout(50,50,0,50,50);
	ser_.setTimeout(time_out);
}

void configure_sensor()
{

}

bool sensor_rcv()
{
    try
    {
        static bool acc_flag = false; // publish after rcv acc
        while(!ser_.available()){
            ros::Duration(0.0001).sleep();
        }
        if (ser_.available() < 23){
            ros::Duration(0.0001).sleep();
            //std::cout << "Why no more?" << std::endl;
            //ser_.flushInput();
            return false;
        }
        boost::timer timer_prog;
        uint8_t header_bytes[5];
        ser_.read(header_bytes, 5);
        uint8_t type, address;

        //! 1. Locate "snp", then acquire Packet Type and Address (see sensor datasheet for details)
        if (memcmp(header_bytes, "snp", 3) == 0)
        {
            // Optimism win.
            type = header_bytes[3];
            address = header_bytes[4];
        }
        else
        {
            std::string snp;
            ser_.readline(snp, 512, "snp");
            if (!boost::algorithm::ends_with(snp, "snp")){
                ROS_WARN("[UM7_DRIVER] cannot find \"snp\"");
                throw std::exception();
            }
            if (snp.length() > 3)
            {
                ROS_WARN_STREAM("Discarded " << 5 + snp.length() - 3 << " junk byte(s) preceeding packet.");
            }
            if (ser_.read(&type, 1) != 1){
                ROS_WARN("[UM7_DRIVER] cannot find \"PT\"");
                throw std::exception();
            }
            if (ser_.read(&address, 1) != 1){
                ROS_WARN("[UM7_DRIVER] cannot find \"ADDR\"");
                throw std::exception();
            }
        }
        uint16_t checksum_calculated = 's' + 'n' + 'p' + type + address;

        //! 2. Read data. I assume that data received is valid, so valid-check is skipped
        // data structure: data_x, data_y, data_z, data_stamp (4 bytes each), checksum (2 bytes)
        uint8_t data_in[DATA_LENGTH];
        ser_.read(data_in, DATA_LENGTH);
        for(unsigned int idx = 0; idx < DATA_LENGTH; idx++)
        {
            checksum_calculated += data_in[idx];
        }

        //! 3. Check sum.
        uint16_t checksum_rcv;
        if (ser_.read(reinterpret_cast<uint8_t*>(&checksum_rcv), 2) != 2)
        {
            ROS_WARN("[UM7_DRIVER] cannot find checksum");
            throw std::exception();
        }
        checksum_rcv = ntohs(checksum_rcv);
        if (checksum_rcv != checksum_calculated){
            ROS_ERROR("[UM7_DRIVER] checksum wrong!");
            throw std::exception();
        }

        //! 4. Load data.
        if (ignore_begin_cnt_ < ignore_begin_num_){
            ignore_begin_cnt_++;
            return false;
        }
        if (address == GYRO_DREG_ADDR){
            memcpy(imu_data_,    data_in, DATA_LENGTH);
            imu_data_[0] = ntohl(imu_data_[0]);
            imu_data_[1] = ntohl(imu_data_[1]);
            imu_data_[2] = ntohl(imu_data_[2]);
            imu_data_[3] = ntohl(imu_data_[3]);
            memcpy(float_imu_data_, imu_data_, DATA_LENGTH);
            if (!is_set_offset_){
                time_offset_ = ros::Time::now().toSec() - float_imu_data_[3];
                is_set_offset_ = true;
            }
            memset(header_bytes, 0, 5); //clear header_bytes
            return false;
        }
        if (address == ACC_DREG_ADDR){
            memcpy(imu_data_ + 4, data_in, DATA_LENGTH);
            imu_data_[4] = ntohl(imu_data_[4]);
            imu_data_[5] = ntohl(imu_data_[5]);
            imu_data_[6] = ntohl(imu_data_[6]);
            imu_data_[7] = ntohl(imu_data_[7]);
            memcpy(float_imu_data_ + 4, imu_data_ + 4, DATA_LENGTH);
            memset(header_bytes, 0, 5); //clear header_bytes
            return true;
        }

        //ROS_INFO_STREAM("[UM7_DRIVER] Got one frame! Stamp:" << float_imu_data_[3]);

        /*
        memcpy(char_imu_data_, data_in, DATA_LENGTH);
        for (int i = 0; i < DATA_LENGTH; i++){
            printf("%X ",char_imu_data_[i]);
        }
        printf("\n%X\n", imu_data_[3]);
        */
        //ROS_INFO_STREAM("[UM7_DRIVER] time elasped: " << timer_prog.elapsed());

        ROS_INFO_STREAM("Available:" << ser_.available());
        
    }
    catch(const std::exception& e)
    {
        ROS_INFO("Exeption called?");
        // flush input data in the buffer
        //ser_.flushInput();
        ROS_INFO_STREAM("Available:" << ser_.available());
        return false;
    }

}

void publish_data()
{
    sensor_msgs::Imu imu_ros;
    imu_ros.header.frame_id = std::string("imu_frame");
    imu_ros.header.stamp = ros::Time(float_imu_data_[3] + time_offset_);
    if (!is_ned_to_nwu_){
        imu_ros.angular_velocity.x = float_imu_data_[0] * GYRO_SCALE;
        imu_ros.angular_velocity.y = float_imu_data_[1] * GYRO_SCALE;
        imu_ros.angular_velocity.z = float_imu_data_[2] * GYRO_SCALE;
        imu_ros.linear_acceleration.x = float_imu_data_[4] * ACC_SCALE;
        imu_ros.linear_acceleration.y = float_imu_data_[5] * ACC_SCALE;
        imu_ros.linear_acceleration.z = float_imu_data_[6] * ACC_SCALE;
    }else{// y,z axes takes on negative value
        imu_ros.angular_velocity.x = float_imu_data_[0] * GYRO_SCALE;
        imu_ros.angular_velocity.y = -float_imu_data_[1] * GYRO_SCALE;
        imu_ros.angular_velocity.z = -float_imu_data_[2] * GYRO_SCALE;
        imu_ros.linear_acceleration.x = float_imu_data_[4] * ACC_SCALE;
        imu_ros.linear_acceleration.y = -float_imu_data_[5] * ACC_SCALE;
        imu_ros.linear_acceleration.z = -float_imu_data_[6] * ACC_SCALE;
    }
    
    imu_pub_.publish(imu_ros);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "um7_driver");
	ros::NodeHandle nh("~");
    imu_pub_ = nh.advertise<sensor_msgs::Imu>("/imu0", 100);

	read_param(nh);

	init_serial();

	//! loop
    while (ros::ok())
    {
    	try
    	{
    		ser_.open();
    	}
    	catch (const serial::IOException& e)
    	{
    		ROS_ERROR("[UM7_DRIVER] unable to connect to port %s, ", port_.c_str());
    	}

    	if (ser_.isOpen())
    	{
    		ROS_INFO("[UM7_DRIVER] successfully connected to port %s", port_.c_str());
    		try
    		{
    			//configure_sensor();
                //ser_.flushInput();
    			while (ros::ok())
    			{
    				if (sensor_rcv())
    				{
    					publish_data();
    				}
    			}
    		}
    		catch (const std::exception& e)
    		{
    			if (ser_.isOpen())
    				ser_.close();
		        ROS_ERROR_STREAM(e.what());
		        ROS_ERROR("[UM7_DRIVER] attempting reconnection after error...");
		        ros::Duration(1.0).sleep();
    		}
    	}
    	else
    	{
    		ROS_ERROR("[UM7_DRIVER] trying every second...");
    		ros::Duration(1.0).sleep();
    	}
    }
}
