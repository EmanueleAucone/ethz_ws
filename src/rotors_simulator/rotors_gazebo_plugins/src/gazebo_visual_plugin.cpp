#include <rotors_gazebo_plugins/gazebo_visual_plugin.h>

namespace gazebo
{
  namespace rendering
  {

    ////////////////////////////////////////////////////////////////////////////////
    // Constructor
    GazeboVisualPlugin::GazeboVisualPlugin()/*: 
      line(NULL)*/
    {
      std::cout << "VISUAL PLUGIN CREATED" << std::endl;
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Destructor
    GazeboVisualPlugin::~GazeboVisualPlugin()
    {
      // Finalize the visualizer
      this->rosnode_->shutdown();
      delete this->rosnode_;
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Load the plugin
    void GazeboVisualPlugin::Load( rendering::VisualPtr _parent, sdf::ElementPtr _sdf )
    {
      this->visual_ = _parent;

      this->visual_namespace_ = "haptic_drone";

      // start ros node
      if (!ros::isInitialized())
      {
        int argc = 0;
        char** argv = NULL;
        ros::init(argc,argv,"gazebo_visual",ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
      }

      this->rosnode_ = new ros::NodeHandle(this->visual_namespace_);
      this->force_sub_ = this->rosnode_->subscribe("/haptic_drone/ft_sensor_topic", 1000, &GazeboVisualPlugin::VisualizeForceOnLink, this);
      this->odom_sub_ = this->rosnode_->subscribe("/haptic_drone/odometry_sensor1/position", 1000, &GazeboVisualPlugin::CurrentLinkPose, this);

      // Listen to the update event. This event is broadcast every simulation iteration.
      this->update_connection_ = event::Events::ConnectRender(
          boost::bind(&GazeboVisualPlugin::UpdateChild, this));

      std::cout << "VISUAL PLUGIN LOADED" << std::endl;
    }

    //////////////////////////////////////////////////////////////////////////////////
    // Update the visualizer
    void GazeboVisualPlugin::UpdateChild()
    {
      /// \brief For example a line to visualize the force
      DynamicLines *line;
      line = this->visual_->CreateDynamicLine(RENDERING_LINE_STRIP);

      ros::spinOnce();

      line->AddPoint(math::Vector3(link_pose.x, link_pose.y, link_pose.z));
      line->AddPoint(math::Vector3(endpoint.x, endpoint.y, endpoint.z));
      // set the Material of the line, in this case to purple
      line->setMaterial("Gazebo/Purple");
      line->setVisibilityFlags(GZ_VISIBILITY_GUI);
      this->visual_->SetVisible(true);
    }

    //////////////////////////////////////////////////////////////////////////////////
    // Get arrow's starting point
    void GazeboVisualPlugin::CurrentLinkPose(const geometry_msgs::PointStamped::ConstPtr &point_msg)
    {
	this->link_pose = point_msg->point;
    }

    //////////////////////////////////////////////////////////////////////////////////
    // Get arrow's ending point
    geometry_msgs::Point GazeboVisualPlugin::CalculateEndpointOfForceVector(geometry_msgs::Wrench wrench)
    {
	geometry_msgs::Point p;
	p.x = this->link_pose.x;
	p.y = this->link_pose.y - 0.5;
	p.z = this->link_pose.z - 0.5;

	return p;
    }

    //////////////////////////////////////////////////////////////////////////////////
    // VisualizeForceOnLink
    void GazeboVisualPlugin::VisualizeForceOnLink(const geometry_msgs::WrenchStamped::ConstPtr &force_msg)
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

    // Register this plugin within the simulator
    GZ_REGISTER_VISUAL_PLUGIN(GazeboVisualPlugin)
  }
}
