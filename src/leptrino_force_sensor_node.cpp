#include <ros/ros.h>
#include <serial/serial.h>
#include <geometry_msgs/WrenchStamped.h>
#include <vector>
#include <array>
#include <string>

class LeptrinoForceSensor
{
public:
    LeptrinoForceSensor(ros::NodeHandle& nh)
    : nh_(nh)
    {
        nh_.param<std::string>("port", port_, "/dev/ttyACM0");
        nh_.param<int>("baudrate", baudrate_, 460800);
        nh_.param<std::string>("frame_id", frame_id_, "force_sensor_link");

        pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("force_data", 10);

        serial_.setPort(port_);
        serial_.setBaudrate(baudrate_);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        serial_.setTimeout(to);

        try
        {
            serial_.open();
        }
        catch (serial::IOException& e)
        {
            ROS_ERROR_STREAM("Unable to open port " << port_ << ": " << e.what());
            ros::shutdown();
        }

        if (serial_.isOpen())
        {
            ROS_INFO_STREAM("Serial port " << port_ << " opened successfully.");
        }
        else
        {
            ROS_ERROR_STREAM("Failed to open serial port.");
            ros::shutdown();
        }

        sendStartSignal();
    }

    void spin()
    {
        ros::Rate loop_rate(100); // 100Hz
        while (ros::ok())
        {
            if (serial_.available())
            {
                std::string data = serial_.read(serial_.available());
                recv_buffer_.insert(recv_buffer_.end(), data.begin(), data.end());

                std::array<float, 6> wrench_data;
                bool found = false;
                while (parseFrame(recv_buffer_, wrench_data))
                {
                    found = true;
                }

                if (found)
                {
                    geometry_msgs::WrenchStamped msg;
                    msg.header.stamp = ros::Time::now();
                    msg.header.frame_id = frame_id_;
                    msg.wrench.force.x = wrench_data[0];
                    msg.wrench.force.y = wrench_data[1];
                    msg.wrench.force.z = wrench_data[2];
                    msg.wrench.torque.x = wrench_data[3];
                    msg.wrench.torque.y = wrench_data[4];
                    msg.wrench.torque.z = wrench_data[5];
                    pub_.publish(msg);
                }
            }

            ros::spinOnce();
            loop_rate.sleep();
        }
    }

private:
    void sendStartSignal()
    {
        uint8_t start_signal[] = {0x10, 0x02, 0x04, 0xFF, 0x32, 0x00, 0x10, 0x03, 0xCA};
        serial_.write(start_signal, sizeof(start_signal));
        ROS_INFO_STREAM("Start signal sent.");
    }

    bool parseFrame(std::vector<uint8_t>& buffer, std::array<float, 6>& out_force_torque)
    {
        const float MAX_FORCE_N = 1000.0;
        const float MAX_TORQUE_Nm = 30.0;
        const int MAX_VALUE = 10000;

        if (buffer.size() < 26) return false;

        size_t index = 0;
        while (index + 1 < buffer.size())
        {
            if (buffer[index] == 0x10 && buffer[index+1] == 0x02)
                break;
            ++index;
        }

        if (index + 26 > buffer.size())
        {
            return false;
        }

        std::vector<uint8_t> data;
        size_t i = index + 2;
        while (data.size() < 16 && i < buffer.size())
        {
            if (buffer[i] == 0x10)
            {
                ++i;
            }
            if (i < buffer.size())
            {
                data.push_back(buffer[i]);
                ++i;
            }
        }

        if (data.size() != 16)
        {
            return false;
        }

        for (int j = 0; j < 6; ++j)
        {
            int16_t raw = static_cast<int16_t>(data[j*2+4] | (data[j*2+5] << 8));

            float scaled = 0.0f;
            if (j < 3)
            {
                scaled = (static_cast<float>(raw) / MAX_VALUE) * MAX_FORCE_N;
            }
            else
            {
                scaled = (static_cast<float>(raw) / MAX_VALUE) * MAX_TORQUE_Nm;
            }
            out_force_torque[j] = scaled;
        }

        buffer.erase(buffer.begin(), buffer.begin() + (i + 1));
        return true;
    }

    ros::NodeHandle nh_;
    ros::Publisher pub_;
    serial::Serial serial_;
    std::string port_;
    int baudrate_;
    std::string frame_id_;
    std::vector<uint8_t> recv_buffer_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "leptrino_force_sensor_node");
    ros::NodeHandle nh("~");

    LeptrinoForceSensor sensor(nh);
    sensor.spin();

    return 0;
}
