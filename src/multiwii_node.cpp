#include <ros/ros.h>

#include <std_msgs/UInt16.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>

#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/RCOut.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/ActuatorControl.h>
#include <mavros_msgs/CommandBool.h>

#include <msp/FlightController.hpp>
#include <msp/msp_msg.hpp>
#include <msp/msg_print.hpp>

#include <Eigen/Geometry>

#include <msp/msg_print.hpp>

double deg2rad(const double deg) {
    return deg/180.0 * M_PI;
}

double rad2deg(const double rad) {
    return rad/M_PI * 180.0;
}

class MultiWiiNode {
private:
    ros::NodeHandle nh;
    fcu::FlightController &fcu;

    ros::Publisher imu_pub;
    ros::Publisher magn_pub;
    ros::Publisher pose_stamped_pub;
    ros::Publisher rpy_pub;
    ros::Publisher rc_in_pub, rc_out_pub;
    ros::Publisher motors_pub;
    ros::Publisher arm_pub, lifetime_pub;
    ros::Publisher battery_pub;
    ros::Publisher boxes_pub;
    ros::Publisher arm_status_pub;
    ros::Publisher heading_pub;

    ros::Subscriber rc_in_sub;
    ros::Subscriber motor_control_sub;

    ros::ServiceServer arming_srv;

public:

    MultiWiiNode(fcu::FlightController &fcu) : fcu(fcu) { }

    void setup() {
        // publisher
        imu_pub = nh.advertise<sensor_msgs::Imu>("imu/data", 1);
        magn_pub = nh.advertise<sensor_msgs::MagneticField>("imu/mag", 1);
        pose_stamped_pub = nh.advertise<geometry_msgs::PoseStamped>("local_position/pose", 1);
        rpy_pub = nh.advertise<geometry_msgs::Vector3>("rpy", 1);
        rc_in_pub = nh.advertise<mavros_msgs::RCIn>("rc/in", 1);
        rc_out_pub = nh.advertise<mavros_msgs::RCOut>("rc/out", 1);
        motors_pub = nh.advertise<mavros_msgs::RCOut>("motors", 1);
        arm_pub = nh.advertise<std_msgs::UInt16>("arm_counter",1);
        lifetime_pub = nh.advertise<std_msgs::UInt32>("lifetime",1);
        battery_pub = nh.advertise<sensor_msgs::BatteryState>("battery",1);
        boxes_pub = nh.advertise<std_msgs::UInt8MultiArray>("boxes",1);
        arm_status_pub = nh.advertise<std_msgs::Bool>("armed",1);
        heading_pub = nh.advertise<std_msgs::Float64>("global_position/compass_hdg",1);

        // subscriber
        rc_in_sub = nh.subscribe("rc/override", 1, &MultiWiiNode::rc_override, this);
        motor_control_sub = nh.subscribe("actuator_control", 1, &MultiWiiNode::motor_control, this);

        // services
        arming_srv = nh.advertiseService("cmd/arming", &MultiWiiNode::arming, this);
    }


    ////////////////////////////////////////////////////////////////////////////
    /// callbacks for published messages

    void onImu(const msp::Imu &imu) {
        sensor_msgs::Imu imu_msg;

        imu_msg.header.stamp = ros::Time::now();
        imu_msg.header.frame_id = "imu";

        // linear acceleration in m/s²
        imu_msg.linear_acceleration.x = imu.acc[0];
        imu_msg.linear_acceleration.y = imu.acc[1];
        imu_msg.linear_acceleration.z = imu.acc[2];

        // angular velocity in rad/s
        imu_msg.angular_velocity.x = deg2rad(imu.gyro[0]);
        imu_msg.angular_velocity.y = deg2rad(imu.gyro[1]);
        imu_msg.angular_velocity.z = deg2rad(imu.gyro[2]);

        // magnetic field in uT
        sensor_msgs::MagneticField mag_msg;
        mag_msg.header.stamp = ros::Time::now();
        mag_msg.magnetic_field.x = imu.magn[0];
        mag_msg.magnetic_field.y = imu.magn[1];
        mag_msg.magnetic_field.z = imu.magn[2];

        // heading
        std_msgs::Float64 heading;
        heading.data = rad2deg(atan2(imu.magn[0], imu.magn[1]));
        heading_pub.publish(heading);

        const Eigen::Vector3f magn(imu.magn[0], imu.magn[1], imu.magn[2]);
        const Eigen::Vector3f lin_acc(imu.acc[0], imu.acc[1], imu.acc[2]);

        // http://www.camelsoftware.com/2016/02/20/imu-maths/
        Eigen::Matrix3f rot;
        rot.row(0) = lin_acc.normalized();
        rot.row(1) = lin_acc.cross(magn).normalized();
        rot.row(2) = lin_acc.cross(magn).cross(lin_acc).normalized();

        const Eigen::Quaternionf orientation(rot);

        geometry_msgs::Quaternion quat;
        quat.x = orientation.x();
        quat.y = orientation.y();
        quat.z = orientation.z();
        quat.w = orientation.w();

        imu_msg.orientation = quat;

        imu_pub.publish(imu_msg);
        magn_pub.publish(mag_msg);
    }

    void onAttitude(const msp::Attitude &attitude) {
        // r,p,y to rotation matrix
        Eigen::Matrix3f rot;
        rot = Eigen::AngleAxisf(deg2rad(attitude.ang_x), Eigen::Vector3f::UnitX())
            * Eigen::AngleAxisf(deg2rad(attitude.ang_y),  Eigen::Vector3f::UnitY())
            * Eigen::AngleAxisf(deg2rad(attitude.heading), Eigen::Vector3f::UnitZ());

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
        rpy.x = attitude.ang_x;
        rpy.y = attitude.ang_y;
        rpy.z = attitude.heading;
        rpy_pub.publish(rpy);
    }

    void onRc(const msp::Rc &rc) {
        mavros_msgs::RCIn rc_msg;
        rc_msg.header.stamp = ros::Time::now();
        rc_msg.channels.push_back(rc.roll);
        rc_msg.channels.push_back(rc.pitch);
        rc_msg.channels.push_back(rc.yaw);
        rc_msg.channels.push_back(rc.throttle);
        rc_msg.channels.push_back(rc.aux1);
        rc_msg.channels.push_back(rc.aux2);
        rc_msg.channels.push_back(rc.aux3);
        rc_msg.channels.push_back(rc.aux4);

        rc_in_pub.publish(rc_msg);
    }

    void onServo(const msp::Servo &servo) {
        mavros_msgs::RCOut rc;
        for(const uint16_t s : servo.servo) {
            rc.channels.push_back(s);
        }
        rc_out_pub.publish(rc);
    }

    void onMotor(const msp::Motor &motor) {
        mavros_msgs::RCOut motor_out;
        for(const uint16_t m : motor.motor) {
            motor_out.channels.push_back(m);
        }
        motors_pub.publish(motor_out);
    }

    void onMisc(const msp::Misc &misc) {
        std_msgs::UInt16 arm;
        arm.data = misc.arm;
        arm_pub.publish(arm);

        std_msgs::UInt32 lifetime;
        lifetime.data = misc.lifetime;
        lifetime_pub.publish(lifetime);
    }

    void onAnalog(const msp::Analog &analog) {
        //std::cout<<analog;
        sensor_msgs::BatteryState battery;
        battery.header.stamp = ros::Time::now();
        battery.voltage = analog.vbat;
        battery.current = analog.amperage;

        battery_pub.publish(battery);
    }

    void onStatus(const msp::Status &status) {
        //std::cout<<status;

        std_msgs::Bool armed;
        armed.data = status.active_box_id.count(0);

        std_msgs::UInt8MultiArray box_ids;
        for(const uint b : status.active_box_id) {
            box_ids.data.push_back(b);
        }

        boxes_pub.publish(box_ids);
        arm_status_pub.publish(armed);
    }


    ////////////////////////////////////////////////////////////////////////////
    /// callbacks for subscribed messages

    void rc_override(const mavros_msgs::OverrideRCIn &rc) {
        std::cout<<"overriding RC:"<<std::endl;
        std::cout<<rc.channels[0]<<" "<<rc.channels[1]<<" "<<rc.channels[2]<<" "<<rc.channels[3]<<std::endl;
        fcu.setRc(rc.channels[0], rc.channels[1], rc.channels[2], rc.channels[3],
                  rc.channels[4], rc.channels[5], rc.channels[6], rc.channels[7]);
    }

    void motor_control(const mavros_msgs::ActuatorControl &motors) {
        // ActuatorControl::controls contains normed values in range [-1,+1],
        // negative values are for reversing the motor spinning direction.
        // We will ignore reversed motor direction and the final motor value
        // becomes: 1000 + abs(m)*1000

        std::array<uint16_t,msp::N_MOTOR> motor_values;
        for(uint i(0); i<motor_values.size(); i++) {
            motor_values[i] = 1000+abs(motors.controls[i]*1000);
        }

        fcu.setMotors(motor_values);
    }

    ////////////////////////////////////////////////////////////////////////////
    /// services

    bool arming(mavros_msgs::CommandBool::Request &req, mavros_msgs::CommandBool::Response &res) {
        res.success = (req.value) ? fcu.arm_block() : fcu.disarm_block();
        return res.success;
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "MultiWii");

    // connect to flight controller
    fcu::FlightController fcu("/dev/ttyUSB0");
    fcu.setAcc1G(512);
    fcu.setGyroUnit(1/4.096);
    // 1 Gs = 0.1 mT = 100 uT
    // HMC5883L default gain: 1090 LSb/Gauss = 0.92 mG/LSb
    // 1 unit = 0.92 mGs = 0.92/1000 Gs = 100*0.92/1000 uT = 0.92/10 uT
    fcu.setMagnGain(0.92/10.0);
    fcu.setStandardGravity(9.80665);

    fcu.initialise();

    ROS_INFO("MSP ready");
    std::cout<<"MSP ready"<<std::endl;

    MultiWiiNode node(fcu);

    // register publisher
    node.setup();

    fcu.subscribe(&MultiWiiNode::onImu, &node, 0.01);
    fcu.subscribe(&MultiWiiNode::onAttitude, &node, 0.01);
    fcu.subscribe(&MultiWiiNode::onRc, &node, 0.1);
    fcu.subscribe(&MultiWiiNode::onServo, &node, 1);
    fcu.subscribe(&MultiWiiNode::onMotor, &node, 0.1);
    fcu.subscribe(&MultiWiiNode::onMisc, &node, 1);
    fcu.subscribe(&MultiWiiNode::onAnalog, &node, 10);
    fcu.subscribe(&MultiWiiNode::onStatus, &node, 0.1);

    while (ros::ok()) {
//        fcu.handle();
//        fcu.handle_batch();
//        fcu.sendRequests();
        fcu.handleRequests();

        ros::spinOnce();
    }

    ros::shutdown();
}
