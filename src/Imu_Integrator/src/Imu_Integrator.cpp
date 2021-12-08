#include "Imu_Integrator/Imu_Integrator.h"
#include <cmath>

ImuIntegrator::ImuIntegrator(const ros::Publisher &pub) {
  Eigen::Vector3d zero(0, 0, 0);
  pose.pos = zero;
  pose.orien = Eigen::Matrix3d::Identity();
  g_count = 0;
  velocity = zero;
  line_pub = pub;
  firstT = true;

  // Line strip is blue
  path.color.b = 1.0;
  path.color.a = 1.0;
  path.type = visualization_msgs::Marker::LINE_STRIP;
  path.header.frame_id = "map";
  path.ns = "points_and_lines";
  path.action = visualization_msgs::Marker::ADD;
  path.pose.orientation.w = 1.0;
  path.scale.x = 0.2;
  geometry_msgs::Point p;
  p.x = 0;
  p.y = 0;
  p.z = 0;
  path.points.push_back(p);
}

void ImuIntegrator::ImuCallback(const sensor_msgs::Imu &msg) {
  if (firstT) {
    time = msg.header.stamp;
    deltaT = 0;
    firstT = !setGravity(msg.linear_acceleration);
    ROS_ERROR(
      "%f, %f, %f", 
      gravity[0], 
      gravity[1],
      gravity[2]
    );
  } else {
    deltaT = (msg.header.stamp - time).toSec();
    time = msg.header.stamp;
    calcOrientation(msg.angular_velocity);
    calcPosition(msg.linear_acceleration);
    updatePath(pose.pos);
    publishMessage();
    ROS_WARN("x:%f, y:%f, z:%f", pose.pos[0], pose.pos[1], pose.pos[2]);
  }
  // std::cout << pose.pos << std::endl;
}

bool ImuIntegrator::setGravity(const geometry_msgs::Vector3 &msg) {
  if (
    (
      (std::fabs(msg.x) < 0.3) && 
      (std::fabs(msg.y) < 0.3) && 
      (std::fabs(msg.z) > 9.0) && 
      g_count == 0
    ) || (
      (std::fabs(msg.x - gravities[g_count-1][0]) < 0.05) && 
      (std::fabs(msg.y - gravities[g_count-1][1]) < 0.05) && 
      (std::fabs(msg.z - gravities[g_count-1][2]) < 0.05) && 
      g_count > 0
    )
  ) {
    gravities[g_count][0] = msg.x;
    gravities[g_count][1] = msg.y;
    gravities[g_count][2] = msg.z;
    g_count++;
  } else {
    g_count = 0;
  }
  ROS_WARN("%i", g_count);
  ROS_ERROR("%f, %f, %f", msg.x, msg.y, msg.z);
  gravity[0] = 0.0;
  gravity[1] = 0.0;
  gravity[2] = 0.0;
  if (g_count >= 100) {
    for(int i = 0; i < 100; i++) {
      for(int j = 0; j < 3; j++) {
        gravity[j] += gravities[i][j] / 100.0;
      }
      ROS_WARN(
        "%f, %f, %f", 
        gravities[i][0], 
        gravities[i][1],
        gravities[i][2]
      );
    }
    return true;
  }

  return false;
}

void ImuIntegrator::updatePath(const Eigen::Vector3d &msg) {
  geometry_msgs::Point p;
  p.x = msg[0];
  p.y = msg[1];
  p.z = msg[2];
  path.points.push_back(p);
}

void ImuIntegrator::publishMessage() { line_pub.publish(path); }

void ImuIntegrator::calcOrientation(const geometry_msgs::Vector3 &msg) {
  Eigen::Matrix3d B;
  B << 0, -msg.z * deltaT, msg.y * deltaT, msg.z * deltaT, 0, -msg.x * deltaT,
      -msg.y * deltaT, msg.x * deltaT, 0;
  double sigma =
      std::sqrt(std::pow(msg.x, 2) + std::pow(msg.y, 2) + std::pow(msg.z, 2)) *
      deltaT;
  // std::cout << "sigma: " << sigma << std::endl << Eigen::Matrix3d::Identity()
  // + (std::sin(sigma) / sigma) * B << std::endl << pose.orien << std::endl;
  pose.orien = pose.orien *
               (Eigen::Matrix3d::Identity() + (std::sin(sigma) / sigma) * B -
                ((1 - std::cos(sigma)) / std::pow(sigma, 2)) * B * B);
}

void ImuIntegrator::calcPosition(const geometry_msgs::Vector3 &msg) {
  Eigen::Vector3d acc_l(msg.x, msg.y, msg.z);
  Eigen::Vector3d acc_g = pose.orien * acc_l;
  // Eigen::Vector3d acc(msg.x - gravity[0], msg.y - gravity[1], msg.z -
  // gravity[2]);
  velocity = velocity + deltaT * (acc_g - gravity);
  pose.pos = pose.pos + deltaT * velocity;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "Imu_Integrator_node");
  ros::NodeHandle nh;
  ros::Publisher line =
      nh.advertise<visualization_msgs::Marker>("Imu_path", 1000);
  ImuIntegrator *imu_integrator = new ImuIntegrator(line);
  ros::Subscriber Imu_message = nh.subscribe(
      "/imu/data", 1000, &ImuIntegrator::ImuCallback, imu_integrator);
  ros::spin();
}
