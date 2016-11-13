#include <ros/ros.h>

#include <std_msgs/UInt16.h>
#include <std_msgs/UInt32.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>

#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/RCOut.h>

#include <msp/FlightController.hpp>
#include <msp/fcu_msg.hpp>
#include <msp/msg_print.hpp>

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
        magn_pub = nh.advertise<sensor_msgs::MagneticField>("magnetometer", 1);
        pose_stamped_pub = nh.advertise<geometry_msgs::PoseStamped>("pose", 1);
        rpy_pub = nh.advertise<geometry_msgs::Vector3>("rpy", 1);
        rc_in_pub = nh.advertise<mavros_msgs::RCIn>("rc/in", 1);
        rc_out_pub = nh.advertise<mavros_msgs::RCOut>("rc/out", 1);
        motors_pub = nh.advertise<mavros_msgs::RCOut>("motors", 1);
        arm_pub = nh.advertise<std_msgs::UInt16>("arm_counter",1);
        lifetime_pub = nh.advertise<std_msgs::UInt32>("lifetime",1);
        battery_pub = nh.advertise<sensor_msgs::BatteryState>("battery",1);
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

        //geometry_msgs::Vector3 mag_msg;
        sensor_msgs::MagneticField mag_msg;
        mag_msg.header.stamp = ros::Time::now();
        mag_msg.magnetic_field.x = imu->magn[0];
        mag_msg.magnetic_field.y = imu->magn[1];
        mag_msg.magnetic_field.z = imu->magn[2];

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

        imu_pub.publish(imu_msg);
        magn_pub.publish(mag_msg);
    }

    void onAttitude(const fcu::Attitude *attitude) {
        // r,p,y to rotation matrix
        Eigen::Matrix3f rot;
        rot = Eigen::AngleAxisf(deg2rad(attitude->ang_x), Eigen::Vector3f::UnitX())
            * Eigen::AngleAxisf(deg2rad(attitude->ang_y),  Eigen::Vector3f::UnitY())
            * Eigen::AngleAxisf(deg2rad(attitude->heading), Eigen::Vector3f::UnitZ());

        const Eigen::Quaternionf quat(rot);

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time::now();
        pose_stamped.header.frame_id = "attitude";
        pose_stamped.pose.orientation.x = quat.x();
        pose_stamped.pose.orientation.y = quat.y();
        pose_stamped.pose.orientation.z = quat.z();
        pose_stamped.pose.orientation.w = quat.w();

        pose_stamped_pub.publish(pose_stamped);

        geometry_msgs::Vector3 rpy;
        rpy.x = attitude->ang_x;
        rpy.y = attitude->ang_y;
        rpy.z = attitude->heading;
        rpy_pub.publish(rpy);
    }

    void onRc(const msp::Rc *rc) {
        mavros_msgs::RCIn rc_msg;
        rc_msg.header.stamp = ros::Time::now();
        rc_msg.channels.push_back(rc->roll);
        rc_msg.channels.push_back(rc->pitch);
        rc_msg.channels.push_back(rc->yaw);
        rc_msg.channels.push_back(rc->throttle);
        rc_msg.channels.push_back(rc->aux1);
        rc_msg.channels.push_back(rc->aux2);
        rc_msg.channels.push_back(rc->aux3);
        rc_msg.channels.push_back(rc->aux4);

        rc_in_pub.publish(rc_msg);
    }

    void onServo(const msp::Servo *servo) {
        mavros_msgs::RCOut rc;
        for(const uint16_t s : servo->servo) {
            rc.channels.push_back(s);
        }
        rc_out_pub.publish(rc);
    }

    void onMotor(const msp::Motor *motor) {
        mavros_msgs::RCOut motor_out;
        for(const uint16_t m : motor->motor) {
            motor_out.channels.push_back(m);
        }
        motors_pub.publish(motor_out);
    }

    void onMisc(const fcu::Misc *misc) {
        std_msgs::UInt16 arm;
        arm.data = misc->arm;
        arm_pub.publish(arm);

        std_msgs::UInt32 lifetime;
        lifetime.data = misc->lifetime;
        lifetime_pub.publish(lifetime);
    }

    void onAnalog(const fcu::Analog *analog) {
        sensor_msgs::BatteryState battery;
        battery.header.stamp = ros::Time::now();
        battery.voltage = analog->vbat;
        battery.current = analog->amperage;

        battery_pub.publish(battery);
    }

private:
    ros::NodeHandle nh;
    ros::Publisher imu_pub, magn_pub;
    ros::Publisher pose_stamped_pub;
    ros::Publisher rpy_pub;
    ros::Publisher rc_in_pub, rc_out_pub;
    ros::Publisher motors_pub;
    ros::Publisher arm_pub, lifetime_pub;
    ros::Publisher battery_pub;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "MultiWii");

    MultiWiiNode node;

    // register publisher
    node.setup();

    // connect to flight controller
    fcu::FlightController fcu("/dev/ttyUSB0");
    sleep(8);

    fcu.setAcc1G(512);
    fcu.setGyroUnit(1/4096.0);
    fcu.setMagnGain(1090/100.0);
    fcu.setStandardGravity(9.80665);

    ROS_INFO("MSP ready");

    fcu.subscribe(msp::ID::MSP_RAW_IMU, &MultiWiiNode::onImu, &node);
    fcu.subscribe(msp::ID::MSP_ATTITUDE, &MultiWiiNode::onAttitude, &node);
    fcu.subscribe(msp::ID::MSP_RC, &MultiWiiNode::onRc, &node);
    fcu.subscribe(msp::ID::MSP_SERVO, &MultiWiiNode::onServo, &node);
    fcu.subscribe(msp::ID::MSP_MOTOR, &MultiWiiNode::onMotor, &node);
    fcu.subscribe(msp::ID::MSP_MISC, &MultiWiiNode::onMisc, &node);
    fcu.subscribe(msp::ID::MSP_ANALOG, &MultiWiiNode::onAnalog, &node);

    while (ros::ok()) {
        //fcu.handle();
        fcu.handle_batch();
    }
}
