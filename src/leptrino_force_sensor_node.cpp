#include <ros/ros.h>
#include <serial/serial.h>
#include <geometry_msgs/WrenchStamped.h>

class LeptrinoForceSensor
{
public:
    LeptrinoForceSensor(ros::NodeHandle& nh)
    : nh_(nh)
    {
        // パラメータ読み込み
        nh_.param<std::string>("port", port_, "/dev/ttyUSB0");
        nh_.param<int>("baudrate", baudrate_, 115200);
        nh_.param<std::string>("frame_id", frame_id_, "force_sensor_link");

        // トピックPublisher
        pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("force_data", 10);

        // シリアルポート設定
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
            ROS_ERROR_STREAM("Unable to open port " << port_);
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
    }

    void spin()
    {
        ros::Rate loop_rate(100); // 100Hz

        while (ros::ok())
        {
            if (serial_.available())
            {
                std::string data = serial_.read(serial_.available());
                parseAndPublish(data);
            }

            ros::spinOnce();
            loop_rate.sleep();
        }
    }

private:
    void parseAndPublish(const std::string& data)
    {
        // ここで受信データをパースする
        // 仮実装（適宜修正してください）

        if (data.size() < 24) // 例：最低24byteくらい必要な場合
        {
            return;
        }

        geometry_msgs::WrenchStamped msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = frame_id_;

        // 仮：データから適当に力・モーメントを抽出する（仕様に応じて変えてください）
        // たとえば6軸センサなら、X,Y,Z, Roll, Pitch, Yawで6個
        // ここでは簡単に先頭から順に並んでいると仮定します
        // 実際にはバイナリ読み取り＋エンディアン変換が必要かも！

        if (data.size() >= 24)
        {
            // 仮にfloat4バイトで6軸だとすると
            float fx = *((float*)&data[0]);
            float fy = *((float*)&data[4]);
            float fz = *((float*)&data[8]);
            float tx = *((float*)&data[12]);
            float ty = *((float*)&data[16]);
            float tz = *((float*)&data[20]);

            msg.wrench.force.x = fx;
            msg.wrench.force.y = fy;
            msg.wrench.force.z = fz;
            msg.wrench.torque.x = tx;
            msg.wrench.torque.y = ty;
            msg.wrench.torque.z = tz;

            pub_.publish(msg);
        }
    }

    ros::NodeHandle nh_;
    ros::Publisher pub_;
    serial::Serial serial_;
    std::string port_;
    int baudrate_;
    std::string frame_id_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "leptrino_force_sensor_node");
    ros::NodeHandle nh("~");

    LeptrinoForceSensor sensor(nh);
    sensor.spin();

    return 0;
}
