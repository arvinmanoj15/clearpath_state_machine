#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <position_msgs/ObjectPositions.h>
#include <position_msgs/ObjectPosition.h>

ros::Publisher new_topic_publisher;

void objectMessageCallback(const position_msgs::ObjectPositions::ConstPtr& msg)
{
  
  for (const auto& pos : msg->object_positions)
  {
        if(pos.Class == "bottle")
        {

            int64_t x_int = pos.x;
            int64_t y_int = pos.y;
            int64_t z_int = pos.z;

	    // Offset values for calculating UR5 X,Y,Z
            float x_offset = 80.0;
            float y_offset = 0.0;
            float z_offset = 0.0;

            // X,Y,Z for UR5
            float x = z_int - x_offset;
            float y = -(x_int - y_offset);
            float z = -y_int - z_offset;
            
            if(x>330.0)
            {
            	x = 330.0;
            	std::cout << "X was Capped" << std::endl;
            }

            // Create a new message with the float values
            std_msgs::Float64MultiArray new_msg;

            new_msg.data.resize(3);
            new_msg.data[0] = x;
            new_msg.data[1] = y;
            new_msg.data[2] = z;
            std::cout << "x " <<x<<" y"<<y<<" z"<<z<< std::endl;
            new_topic_publisher.publish(new_msg);
        }
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "object_converter_node");
  ros::NodeHandle nh;

  ros::Subscriber object_subscriber = nh.subscribe("/objects_position/message", 10, objectMessageCallback);
  new_topic_publisher = nh.advertise<std_msgs::Float64MultiArray>("/transformed_object_position", 10);

  ros::spin();

  return 0;
}
