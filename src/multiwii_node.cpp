#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <multiwii/UpdateRatesConfig.h>

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

#include <tf/transform_broadcaster.h>

#include <msp/FlightController.hpp>
#include <msp/msp_msg.hpp>

#include <Eigen/Geometry>

double deg2rad(const double deg) {
    return deg/180.0 * M_PI;
}

double rad2deg(const double rad) {
    return rad/M_PI * 180.0;
}

class MultiWiiNode {
private:
    ros::NodeHandle nh;
    fcu::FlightController *fcu;

    float acc_1g;
    float gyro_unit;
    float magn_gain;
    float si_unit_1g;
    std::string tf_base_frame;

    dynamic_reconfigure::Server<multiwii::UpdateRatesConfig> dyn_conf_srv;

    ros::Publisher imu_pub;
    ros::Publisher magn_pub;
    ros::Publisher pose_stamped_pub;
    ros::Publisher rpy_pub;
    ros::Publisher rc_in_pub, servo_pub;
    ros::Publisher motors_pub;
    ros::Publisher battery_pub;
    ros::Publisher heading_pub;
    ros::Publisher vis_pub;
    ros::Publisher altitude_pub;
    ros::Publisher sonar_altitude_pub;

    ros::Subscriber rc_in_sub;
    ros::Subscriber rc_in_sub2;
    ros::Subscriber motor_control_sub;

    tf::TransformBroadcaster tf_broadcaster;

public:
    MultiWiiNode() {
        nh = ros::NodeHandle("~");
        // configure
        fcu = new fcu::FlightController();

        float std_grav;
        if(nh.getParam("standard_gravity", std_grav))
            this->si_unit_1g = std_grav;
        else
            ROS_ERROR("Parameter 'standard_gravity' not set.");

        float acc_1g;
        if(nh.getParam("acc_1g", acc_1g))
            this->acc_1g = acc_1g;
        else
            ROS_ERROR("Parameter 'acc_1g' not set.");

        float gyro_unit;
        if(nh.getParam("gyro_unit", gyro_unit))
            this->gyro_unit = gyro_unit;
        else
            ROS_ERROR("Parameter 'gyro_unit' not set.");

        float magn_gain;
        if(nh.getParam("magn_gain", magn_gain))
            this->magn_gain = magn_gain;
        else
            ROS_ERROR("Parameter 'magn_gain' not set.");

        // Get the base frame to which the TF is published
        nh.param<std::string>("tf_base_frame", this->tf_base_frame, "map");
    }

    ~MultiWiiNode() {
        delete fcu;
    }

    fcu::FlightController& fc() const {
        return *fcu;
    }

    void setup() {
        std::string device;
        int baudrate = 115200;
        if(nh.getParam("device", device)) {
            if(!nh.getParam("baudrate", baudrate)) {
                ROS_ERROR("Parameter 'baudrate' not set. Using default baudrate of %i", baudrate);
            }
            else {
                if(baudrate<=0) {
                    ROS_ERROR("'baudrate' must be positive!");
                    baudrate = 115200;
                }
            }
            ROS_INFO("Connected to FCU at %s", device.c_str());
        }
        else {
            ROS_ERROR("Parameter 'device' not set.");
        }

        fcu->connect(device, uint(baudrate));

        // publisher
        imu_pub = nh.advertise<sensor_msgs::Imu>("imu/data", 1);
        magn_pub = nh.advertise<sensor_msgs::MagneticField>("imu/mag", 1);
        pose_stamped_pub = nh.advertise<geometry_msgs::PoseStamped>("local_position/pose", 1);
        rpy_pub = nh.advertise<geometry_msgs::Vector3>("rpy", 1);
        rc_in_pub = nh.advertise<mavros_msgs::RCIn>("rc/in", 1);
        servo_pub = nh.advertise<mavros_msgs::RCOut>("rc/servo", 1);
        motors_pub = nh.advertise<mavros_msgs::RCOut>("motors", 1);
        battery_pub = nh.advertise<sensor_msgs::BatteryState>("battery",1);
        heading_pub = nh.advertise<std_msgs::Float64>("global_position/compass_hdg",1);
        altitude_pub = nh.advertise<std_msgs::Float64>("global_position/rel_alt",1);
        sonar_altitude_pub = nh.advertise<std_msgs::Float64>("global_position/sonar_alt", 1);

        // subscriber
        rc_in_sub = nh.subscribe("rc/override", 1, &MultiWiiNode::rc_override_AERT1234, this); // AERT1234
        rc_in_sub2 = nh.subscribe("rc/override/raw", 1, &MultiWiiNode::rc_override_raw, this); // raw channel order
        motor_control_sub = nh.subscribe("actuator_control", 1, &MultiWiiNode::motor_control, this);
    }

    /**
     * @brief setDynamicConfigureCallback set the callback
     * This will call the callback once for initialisation
     */
    void setDynamicConfigureCallback() {
        // dynamic configure
        dyn_conf_srv.setCallback(boost::bind(&MultiWiiNode::dynconf_callback, this, _1, _2));
    }

    void dynconf_callback(multiwii::UpdateRatesConfig &config, uint32_t /*level*/) {
        // define map with matching update rate per message ID
        const std::map<msp::ID, double> msp_rates = {
            {msp::ID::MSP_STATUS, config.MSP_STATUS},
            {msp::ID::MSP_RAW_IMU, config.MSP_RAW_IMU},
            {msp::ID::MSP_ALTITUDE, config.MSP_ALTITUDE},
            {msp::ID::MSP_ATTITUDE, config.MSP_ATTITUDE},
            {msp::ID::MSP_RC, config.MSP_RC},
            {msp::ID::MSP_SERVO, config.MSP_SERVO},
            {msp::ID::MSP_MOTOR, config.MSP_MOTOR},
            {msp::ID::MSP_SERVO, config.MSP_SERVO},
            {msp::ID::MSP_MISC, config.MSP_MISC},
            {msp::ID::MSP_ANALOG, config.MSP_ANALOG},
            {msp::ID::MSP_SONAR_ALTITUDE, config.MSP_SONAR_ALTITUDE},
        };

        // apply update
        for(const auto& r : msp_rates) {
            if(fcu->hasSubscription(r.first)) {
                fcu->getSubscription(r.first)->setTimerFrequency(r.second);
            }
            else {
                std::cerr<<"message ID "<<uint(r.first)<<" not subscribed"<<std::endl;
            }
        }
    }


    ////////////////////////////////////////////////////////////////////////////
    /// callbacks for published messages

    void onImu(const msp::msg::RawImu &imu) {
        ///////////////////////////////////
        /// IMU data

        sensor_msgs::Imu imu_msg;
        imu_msg.header.stamp = ros::Time::now();
        imu_msg.header.frame_id = "multiwii";

        // linear acceleration in m/s²
        imu_msg.linear_acceleration.x = imu.acc[0];
        imu_msg.linear_acceleration.y = imu.acc[1];
        imu_msg.linear_acceleration.z = imu.acc[2];

        // angular velocity in rad/s
        imu_msg.angular_velocity.x = deg2rad(imu.gyro[0]);
        imu_msg.angular_velocity.y = deg2rad(imu.gyro[1]);
        imu_msg.angular_velocity.z = deg2rad(imu.gyro[2]);

        // rotation from direction of acceleration and magnetic field
        const Eigen::Vector3d magn(imu.mag[0], imu.mag[1], imu.mag[2]);
        const Eigen::Vector3d lin_acc(imu.acc[0], imu.acc[1], imu.acc[2]);

        // http://www.camelsoftware.com/2016/02/20/imu-maths/
        Eigen::Matrix3d rot;
        rot.col(0) = lin_acc.cross(magn).cross(lin_acc).normalized();
        rot.col(1) = lin_acc.cross(magn).normalized();
        rot.col(2) = lin_acc.normalized();

        const Eigen::Quaterniond orientation(rot);
        imu_msg.orientation.x = orientation.x();
        imu_msg.orientation.y = orientation.y();
        imu_msg.orientation.z = orientation.z();
        imu_msg.orientation.w = orientation.w();

        imu_pub.publish(imu_msg);

        ///////////////////////////////////
        /// magnetic field vector

        sensor_msgs::MagneticField mag_msg;
        mag_msg.header.stamp = ros::Time::now();

        // magnetic field in uT
        mag_msg.magnetic_field.x = imu.mag[0];
        mag_msg.magnetic_field.y = imu.mag[1];
        mag_msg.magnetic_field.z = imu.mag[2];
        magn_pub.publish(mag_msg);

        ///////////////////////////////////
        /// heading from magnetic field

        std_msgs::Float64 heading;
        heading.data = rad2deg(std::atan2(double(imu.mag[0]), double(imu.mag[1])));
        heading_pub.publish(heading);
    }

    void onAttitude(const msp::msg::Attitude &attitude) {
        // r,p,y to rotation matrix
        Eigen::Matrix3f rot;
        rot = Eigen::AngleAxisf(deg2rad(attitude.roll), Eigen::Vector3f::UnitX())
            * Eigen::AngleAxisf(deg2rad(attitude.pitch),  Eigen::Vector3f::UnitY())
            * Eigen::AngleAxisf(deg2rad(attitude.yaw), Eigen::Vector3f::UnitZ());

        const Eigen::Quaternionf quat(rot);

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time::now();
        pose_stamped.header.frame_id = "multiwii";
        pose_stamped.pose.orientation.x = quat.x();
        pose_stamped.pose.orientation.y = quat.y();
        pose_stamped.pose.orientation.z = quat.z();
        pose_stamped.pose.orientation.w = quat.w();

        pose_stamped_pub.publish(pose_stamped);

        ///////////////////////////////////
        // Broadcast transform to relate multiwii transformation to the base frame
        // Convert attitude values to quaternion
        tf::Quaternion multiwii_quaternion;
        multiwii_quaternion.setRPY(deg2rad(attitude.roll), deg2rad(attitude.pitch), deg2rad(attitude.yaw));
        // Pack attitude into tf::Transform
        tf::Transform multiwii_transform;
        multiwii_transform.setRotation(multiwii_quaternion);
        multiwii_transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
        // Broadcast as tf::StampedTransform
        tf_broadcaster.sendTransform(tf::StampedTransform(multiwii_transform, ros::Time::now(), "multiwii_cartesian", "multiwii"));

        geometry_msgs::Vector3 rpy;
        rpy.x = attitude.roll;
        rpy.y = attitude.pitch;
        rpy.z = attitude.yaw;
        rpy_pub.publish(rpy);
    }

    void onAltitude(const msp::msg::Altitude &altitude) {
        std_msgs::Float64 alt; // altitude in meter
        alt.data = altitude.altitude;
        altitude_pub.publish(alt);

        ///////////////////////////////////
        // Broadcast transform to relate multiwii transformation to the base frame
        // Pack attitude into tf::Transform
        tf::Quaternion multiwii_quaternion(0.0, 0.0, 0.0, 1.0);
        tf::Transform multiwii_transform;
        multiwii_transform.setRotation(multiwii_quaternion);
        multiwii_transform.setOrigin(tf::Vector3(0.0, 0.0, altitude.altitude));
        // Broadcast as tf::StampedTransform
        tf_broadcaster.sendTransform(tf::StampedTransform(multiwii_transform, ros::Time::now(), this->tf_base_frame, "multiwii_cartesian"));
    }

    void onRc(const msp::msg::Rc &rc) {
        mavros_msgs::RCIn rc_msg;
        rc_msg.header.stamp = ros::Time::now();
        rc_msg.channels = rc.channels;

        rc_in_pub.publish(rc_msg);
    }

    void onServo(const msp::msg::Servo &servo) {
        mavros_msgs::RCOut rc;
        for(const uint16_t s : servo.servo) {
            rc.channels.push_back(s);
        }
        servo_pub.publish(rc);
    }

    void onMotor(const msp::msg::Motor &motor) {
        mavros_msgs::RCOut motor_out;
        for(const uint16_t m : motor.motor) {
            motor_out.channels.push_back(m);
        }
        motors_pub.publish(motor_out);
    }

    void onAnalog(const msp::msg::Analog &analog) {
        sensor_msgs::BatteryState battery;
        battery.header.stamp = ros::Time::now();
        battery.voltage = analog.vbat;
        battery.current = analog.amperage;

        battery_pub.publish(battery);
    }

    void onSonarAltitude(const msp::msg::SonarAltitude &sonar_altitude) {
        std_msgs::Float64 alt;
        alt.data = sonar_altitude.altitude_cm;
        sonar_altitude_pub.publish(alt);

        ///////////////////////////////////
        // Broadcast transform to relate multiwii transformation to the base frame
        // Pack attitude into tf::Transform
        tf::Quaternion multiwii_quaternion(0.0, 0.0, 0.0, 1.0);
        tf::Transform multiwii_transform;
        multiwii_transform.setRotation(multiwii_quaternion);
        multiwii_transform.setOrigin(tf::Vector3(0.0, 0.0, sonar_altitude.altitude_cm));
        // Broadcast as tf::StampedTransform
        tf_broadcaster.sendTransform(tf::StampedTransform(multiwii_transform, ros::Time::now(), this->tf_base_frame, "multiwii_cartesian"));
    }


    ////////////////////////////////////////////////////////////////////////////
    /// callbacks for subscribed messages

    void rc_override_AERT1234(const mavros_msgs::OverrideRCIn &rc) {
        fcu->setRc(rc.channels[0], rc.channels[1], rc.channels[2], rc.channels[3],
                   rc.channels[4], rc.channels[5], rc.channels[6], rc.channels[7]);
    }

    void rc_override_raw(const mavros_msgs::OverrideRCIn &rc) {
        std::vector<uint16_t> channels;
        for(const uint16_t c : rc.channels) { channels.push_back(c); }
        fcu->setRc(channels);
    }

    void motor_control(const mavros_msgs::ActuatorControl &motors) {
        // ActuatorControl::controls contains normed values in range [-1,+1],
        // negative values are for reversing the motor spinning direction.
        // We will ignore reversed motor direction and the final motor value
        // becomes: 1000 + abs(m)*1000

        std::array<uint16_t,msp::msg::N_MOTOR> motor_values;
        for(uint i(0); i<motor_values.size(); i++) {
            motor_values[i] = 1000+abs(motors.controls[i]*1000);
        }

        fcu->setMotors(motor_values);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "MultiWii");

    MultiWiiNode node;

    // setup FCU, register publisher
    node.setup();

    ROS_INFO("MSP ready");
    std::cout<<"MSP ready"<<std::endl;

    node.fc().subscribe(&MultiWiiNode::onImu, &node);
    node.fc().subscribe(&MultiWiiNode::onAttitude, &node);
    node.fc().subscribe(&MultiWiiNode::onAltitude, &node);
    node.fc().subscribe(&MultiWiiNode::onRc, &node);
    node.fc().subscribe(&MultiWiiNode::onServo, &node);
    node.fc().subscribe(&MultiWiiNode::onMotor, &node);
    node.fc().subscribe(&MultiWiiNode::onAnalog, &node);
    node.fc().subscribe(&MultiWiiNode::onSonarAltitude, &node);

    // register callback for dynamic configuration
    // - update rates for MSP subscriber
    // - main ROS node loop rate
    node.setDynamicConfigureCallback();

    ros::spin();

    ros::shutdown();
}
