#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/msgs/MessageTypes.hh"

#include <string>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>

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
	public: GazeboVisualPlugin() 
	{
      		std::cout << "VISUAL PLUGIN CREATED" << std::endl;
    	}

        private:
		ros::NodeHandle* rosnode_;
		
		std::string model_name_;
		std::string topic_name_;

		// The visual pointer used to visualize the force.
		rendering::VisualPtr visual_;

		// The scene pointer.
		ScenePtr scene_;

		geometry_msgs::Point link_pose, endpoint;        

		std::string visual_namespace_;

		// Subscribe to some force and odometry
		ros::Subscriber force_sub_, odom_sub_;

		// Pointer to the update event connection
		event::ConnectionPtr update_connection_;

        // Load the visual force plugin tags
        public: void Load( rendering::VisualPtr _parent, sdf::ElementPtr _sdf )
	{
		this->visual_ = _parent;

		this->visual_namespace_ = "force_arrow";

		// start ros node
		if (!ros::isInitialized())
		{
			int argc = 0;
			char** argv = NULL;
			ros::init(argc,argv,"gazebo_visual",ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
		}

		this->rosnode_ = new ros::NodeHandle(this->visual_namespace_);
		this->force_sub_ = this->rosnode_->subscribe("/haptic_drone/ft_filtered_measurements", 1000, &GazeboVisualPlugin::VisualizeForceOnLink, this);
		this->odom_sub_ = this->rosnode_->subscribe("/haptic_drone/odometry_sensor1/position", 1000, &GazeboVisualPlugin::CurrentLinkPose, this);

		std::cout << "VISUAL PLUGIN LOADED" << std::endl;
	}

	// Get the arrow's starting point
	public: void CurrentLinkPose(const geometry_msgs::PointStamped::ConstPtr &point_msg)
	{
		this->link_pose = point_msg->point;
	}

	// Get the arrow's end point
	public: geometry_msgs::Point CalculateEndpointOfForceVector(geometry_msgs::Wrench wrench)
	{
		geometry_msgs::Point p;
		p.x = this->link_pose.x;
		p.y = this->link_pose.y - 0.5;
		p.z = this->link_pose.z - 0.5;

		return p;
	}

        /// \brief Visualize the force
        public: void VisualizeForceOnLink(const geometry_msgs::WrenchStamped::ConstPtr &force_msg)
	{
	      //this->line = this->visual_->CreateDynamicLine(RENDERING_LINE_STRIP);

	      // Get the current link position
	      //link_pose = CurrentLinkPose();
	      // Get the current end position
	      this->endpoint = CalculateEndpointOfForceVector(force_msg->wrench);

	      // Add two points to a connecting line strip from link_pose to endpoint
	      /*this->line->AddPoint(math::Vector3(link_pose.x, link_pose.y, link_pose.z));
	      this->line->AddPoint(math::Vector3(endpoint.x, endpoint.y, endpoint.z));
	      // set the Material of the line, in this case to purple
	      this->line->setMaterial("Gazebo/Purple");
	      this->line->setVisibilityFlags(GZ_VISIBILITY_GUI);
	      this->visual_->SetVisible(true);*/
	}

	public: void Init()
	{
		// Listen to the update event. This event is broadcast every simulation iteration.
		this->update_connection_ = event::Events::ConnectRender(boost::bind(&GazeboVisualPlugin::OnUpdate, this));
	}
	// Update the visual plugin
	protected: void OnUpdate()
	{
		/// A line/arrow to visualize the force
		DynamicLines *line;
		line = this->visual_->CreateDynamicLine(RENDERING_LINE_STRIP);

		ros::spinOnce();

		line->AddPoint(math::Vector3(link_pose.x, link_pose.y, link_pose.z));
		line->AddPoint(math::Vector3(endpoint.x, endpoint.y, endpoint.z));
		line->setMaterial("Gazebo/Purple");
		line->setVisibilityFlags(GZ_VISIBILITY_GUI);
		this->visual_->SetVisible(true);
	}
      
    };
    
    // Register this plugin within the simulator
    GZ_REGISTER_VISUAL_PLUGIN(GazeboVisualPlugin)
  }
}
