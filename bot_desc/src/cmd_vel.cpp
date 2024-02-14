#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>
#include <vector>

// Fonction pour calculer le signe d'un nombre

int sign(double val) {
    return (val > 0) - (val < 0);
}

class CommandPublisher : public rclcpp::Node
{
public:
    CommandPublisher() : Node("command_publisher")
    {
        // Initialision publisher of  position and velocity and suscriber to cmd_vel

        position_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/position_controller/commands", 10);
        
        velocity_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/velocity_controller/commands", 10);

        
        cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10,
            std::bind(&CommandPublisher::cmd_vel_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "command node is ready(ackerman)...");
    }

    


        

private:
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        std_msgs::msg::Float64MultiArray position_command;
        std_msgs::msg::Float64MultiArray velocity_command;

        double wheelBase = 1.3;
        double wheel_seperation = 1.0;
        double wheel_radius = 0.25 ;
        double wheel_steering_y_offset = 0.00 ;
        double steering_track = wheel_seperation - 2*wheel_steering_y_offset ;

        double omega = msg->angular.z; 
        double V = msg->linear.x;

        double vel_steerring_offset = omega * wheel_steering_y_offset;

        int sign_ = sign(V);

        std::vector<double> vel(4, 0.0);
        std::vector<double> pos(4, 0.0); 

        // Calcul des vitesses
        vel[0] = (sign_ / wheel_radius) * std::hypot(V - omega * steering_track / 2, omega * wheelBase / 2) - vel_steerring_offset;
        vel[1] = (sign_ / wheel_radius) * std::hypot(V + omega * steering_track / 2, omega * wheelBase / 2) + vel_steerring_offset;
        vel[2] = (sign_ / wheel_radius) * std::hypot(V - omega * steering_track / 2, omega * wheelBase / 2) - vel_steerring_offset;
        vel[3] = (sign_ / wheel_radius) * std::hypot(V + omega * steering_track / 2, omega * wheelBase / 2) + vel_steerring_offset;

        // Calcul des positions
        pos[0] = std::atan(omega * wheelBase / (1+ 2 * V + omega * steering_track));
        pos[1] = std::atan(omega * wheelBase / (1+ 2 * V - omega * steering_track));
        pos[2] = -pos[0];
        pos[3] = -pos[1];






        // double wheelSpeeds[4]; 
         
        

     // set the angles 
        // double FL_ang =  msg->angular.z;
        // double FR_ang =  msg->angular.z;
        // double RL_ang = - (msg->angular.z);
        // double RR_ang = - (msg->angular.z);

        // Calculer les rayons de virage pour chaque roue
        // double R_FL = wheelBase / tan(FL_ang * M_PI / 180.0);
        // double R_FR = wheelBase / tan(FR_ang * M_PI / 180.0);
        // double R_RL = wheelBase / tan(RL_ang * M_PI / 180.0);
        // double R_RR = wheelBase / tan(RR_ang * M_PI / 180.0);

        // // Calculer les vitesses des roues
        // wheelSpeeds[0] = V + omega * R_FL; // Vitesse roue avant gauche
        // wheelSpeeds[1] = V + omega * R_FR; // Vitesse roue avant droite
        // wheelSpeeds[2] = V + omega * R_RL; // Vitesse roue arrière gauche
        // wheelSpeeds[3] = V + omega * R_RR; // Vitesse roue arrière droite
        
        

        // double steering_angle = msg->angular.z; 
        // double velocity = msg->linear.x;     

        position_command.data = {pos[0],pos[1],pos[3],pos[2]}; 
        velocity_command.data = {vel[0],vel[1],vel[3],vel[2]};
        
        
        velocity_publisher_->publish(velocity_command);
        position_publisher_->publish(position_command);

        
    }

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr position_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
    
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CommandPublisher>());
    rclcpp::shutdown();
    return 0;
}
