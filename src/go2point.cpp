#include<ros/ros.h>
#include<nav_msgs/Odometry.h>
#include<geometry_msgs/Quaternion.h>
#include<tf/tf.h>
#include<geometry_msgs/Twist.h>
#include<math.h>

ros::Publisher twist_pub;
geometry_msgs::Twist robotspeed;
double xf =4, yf = 1.5;
double k =1;

double calculateDistance(double x, double y, double xf, double yf){
    double d = sqrt(pow((xf-x),2)+pow((yf-y),2));
    return d;
}

void odomCallBack(const nav_msgs::OdometryConstPtr &msg)
{
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    
    geometry_msgs::Quaternion qt;
    qt = msg->pose.pose.orientation;
    double yaw = tf::getYaw(qt);
    
    
    double d = calculateDistance (x,y,xf,yf);
    double angle = atan2(yf-y,xf-x);
    double deltaAngle = angle - yaw;
    if ((deltaAngle)>180)
    {
        deltaAngle -=360;
    }
    else if ((deltaAngle)<-180)
    {
        deltaAngle +=360;
    }
    if (d > 0.1){
        double w = k*deltaAngle;
        robotspeed.angular.z = w;
        if (d > 0.5)
        {
            double v = 1;
            robotspeed.linear.x = v;
        }
        else if (0.5 < d< 0.1)
        {
            double v = 0.3;
            robotspeed.linear.x = v;
        }
    }
    else
    {
        double w = 0;
        robotspeed.angular.z = w;
        double v = 0;
        robotspeed.linear.x = v;
    }
    twist_pub.publish(robotspeed);
     
}


int main (int argc, char **argv)
{
    ros::init(argc, argv, "quat2yaw");
    ros::NodeHandle nh;
    ros::Subscriber odom_sub = nh.subscribe("/vrep/vehicle/odometry", 1, odomCallBack);
    twist_pub = nh.advertise<geometry_msgs::Twist>("/go2point/robotSpeeds", 1);
    ros::spin();
    
    
}