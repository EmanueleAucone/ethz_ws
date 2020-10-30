#ifndef ROTORS_GAZEBO_PLUGINS_GAZEBO_VISUAL_PLUGIN_H
#define ROTORS_GAZEBO_PLUGINS_GAZEBO_VISUAL_PLUGIN_H

#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/msgs/MessageTypes.hh"

#include <string>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include "rotors_gazebo_plugins/common.h"

#include "gazebo/common/Time.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/common/Events.hh"

#include "gazebo/rendering/DynamicLines.hh"
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/Scene.hh"

#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <ros/ros.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/WrenchStamped.h>

#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include "gazebo/msgs/msgs.hh"
#include <unistd.h>
#include <iostream>

namespace gazebo
{
  namespace rendering
  {
    class GazeboVisualPlugin : public VisualPlugin
    {
      public:
        /// \brief Constructor
        GazeboVisualPlugin();

        /// \brief Destructor
        virtual ~GazeboVisualPlugin();

        /// \brief Load the visual force plugin tags
        /// \param node XML config node
        void Load( rendering::VisualPtr _parent, sdf::ElementPtr _sdf );

	// Get the arrow's starting point
	void CurrentLinkPose(const geometry_msgs::PointStamped::ConstPtr &point_msg);

	// Get the arrow's end point
	geometry_msgs::Point CalculateEndpointOfForceVector(geometry_msgs::Wrench wrench);

        /// \brief Visualize the force
        void VisualizeForceOnLink(const geometry_msgs::WrenchStamped::ConstPtr &force_ms);


      protected: 
        /// \brief Update the visual plugin
        virtual void UpdateChild();


      private:
        /// \brief pointer to ros node
        ros::NodeHandle* rosnode_;

        /// \brief store model name
        std::string model_name_;

        /// \brief topic name
        std::string topic_name_;

        // /// \brief The visual pointer used to visualize the force.
        rendering::VisualPtr visual_;

        // /// \brief The scene pointer.
        ScenePtr scene_;

	geometry_msgs::Point link_pose, endpoint;        

        /// \brief for setting ROS name space
        std::string visual_namespace_;

        /// \Subscribe to some force and odometry
        ros::Subscriber force_sub_, odom_sub_;

        // Pointer to the update event connection
        event::ConnectionPtr update_connection_;
    };
  }
}

#endif
