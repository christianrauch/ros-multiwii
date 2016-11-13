#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Imu.h>

#include <msp/FlightController.hpp>
#include <msp/fcu_msg.hpp>
#include <msp/msg_print.hpp>

//#include <Eigen/Core>
#include <Eigen/Geometry>

float deg2rad(const float deg) {
    return deg/180.0 * M_PI;
}

class MultiWiiNode {
public:
    MultiWiiNode() {
    }

    void setup() {
        imu_pub = nh.advertise<sensor_msgs::Imu>("imu", 1);
        magn_pub = nh.advertise<geometry_msgs::Vector3>("magnetometer", 1);
        imu_pose_pub = nh.advertise<geometry_msgs::Pose>("imu_pose", 1);
        imu_pose_stamped_pub = nh.advertise<geometry_msgs::PoseStamped>("imu_pose_stamped", 1);
        pose_pub = nh.advertise<geometry_msgs::Pose>("pose", 1);
        pose_stamped_pub = nh.advertise<geometry_msgs::PoseStamped>("pose_stamped", 1);
    }

    void onImu(const fcu::Imu *imu) {
        sensor_msgs::Imu imu_msg;

        imu_msg.header.stamp = ros::Time::now();
        imu_msg.header.frame_id = "imu";

        imu_msg.linear_acceleration.x = imu->acc[0];
        imu_msg.linear_acceleration.y = imu->acc[1];
        imu_msg.linear_acceleration.z = imu->acc[2];

        imu_msg.angular_velocity.x = imu->gyro[0]*M_PI/180.0;
        imu_msg.angular_velocity.y = imu->gyro[1]*M_PI/180.0;
        imu_msg.angular_velocity.z = imu->gyro[2]*M_PI/180.0;

        geometry_msgs::Vector3 mag_msg;
        mag_msg.x = imu->magn[0];
        mag_msg.y = imu->magn[1];
        mag_msg.z = imu->magn[2];

        const Eigen::Vector3f magn(imu->magn[0], imu->magn[1], imu->magn[2]);
        const Eigen::Vector3f lin_acc(imu->acc[0], imu->acc[1], imu->acc[2]);

        Eigen::Quaternionf orientation;
        orientation.setFromTwoVectors(magn, lin_acc);

        geometry_msgs::Quaternion quat;
        quat.x = orientation.x();
        quat.y = orientation.y();
        quat.z = orientation.z();
        quat.w = orientation.w();

        imu_msg.orientation = quat;

        geometry_msgs::Pose pose;
        pose.orientation = quat;

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time::now();
        pose_stamped.header.frame_id = "imu_pose";
        pose_stamped.pose = pose;

        imu_pub.publish(imu_msg);

        magn_pub.publish(mag_msg);
        imu_pose_pub.publish(pose);
        imu_pose_stamped_pub.publish(pose_stamped);
    }

    void onAttitude(const fcu::Attitude *attitude) {
        // r,p,y to rotation matrix
        Eigen::Matrix3f rot;
        rot = Eigen::AngleAxisf(deg2rad(attitude->ang_x), Eigen::Vector3f::UnitX())
            * Eigen::AngleAxisf(deg2rad(attitude->ang_y),  Eigen::Vector3f::UnitY())
            * Eigen::AngleAxisf(deg2rad(attitude->heading), Eigen::Vector3f::UnitZ());

        const Eigen::Quaternionf quat(rot);

        geometry_msgs::Pose pose;
        pose.orientation.x = quat.x();
        pose.orientation.y = quat.y();
        pose.orientation.z = quat.z();
        pose.orientation.w = quat.w();

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time::now();
        pose_stamped.pose = pose;

        pose_pub.publish(pose);
        pose_stamped_pub.publish(pose_stamped);
    }

private:
    ros::NodeHandle nh;
    ros::Publisher imu_pub, magn_pub;
    ros::Publisher imu_pose_pub, imu_pose_stamped_pub;
    ros::Publisher pose_pub, pose_stamped_pub;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "MultiWii");

    MultiWiiNode node;

    node.setup();

    ros::Rate update_rate(200);

    fcu::FlightController fcu("/dev/ttyUSB0");
    sleep(8);

    fcu.setAcc1G(512);
    fcu.setGyroUnit(1/4096.0);
    fcu.setMagnGain(1090/100.0);
    fcu.setStandardGravity(9.80665);

    ROS_INFO("MSP ready");

    fcu.subscribe(msp::ID::MSP_RAW_IMU, &MultiWiiNode::onImu, &node);
    fcu.subscribe(msp::ID::MSP_ATTITUDE, &MultiWiiNode::onAttitude, &node);

    while (ros::ok()) {
        fcu.handle();

        update_rate.sleep();
    }
}
