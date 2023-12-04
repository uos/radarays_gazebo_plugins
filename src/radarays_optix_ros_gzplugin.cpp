#include <radarays_gazebo_plugins/radarays_optix_ros_gzplugin.h>

#include <iostream>
#include <cv_bridge/cv_bridge.h>


namespace gazebo
{

RadaRaysOptixROS::RadaRaysOptixROS()
:Base()
{
    // ROS_INFO_STREAM("[RadaRaysOptixROS] Construct");
}

RadaRaysOptixROS::~RadaRaysOptixROS()
{
    // std::cout << "[RadaRaysOptixROS] Destroy" << std::endl;
}

void RadaRaysOptixROS::parseOutputs(sdf::ElementPtr outputs)
{
    
}

void RadaRaysOptixROS::Load(
    sensors::SensorPtr _sensor, 
    sdf::ElementPtr _sdf)
{
    m_sdf = _sdf;

    m_radarays_sensor =
        std::dynamic_pointer_cast<sensors::RadaRaysOptix>(_sensor);

    if (!m_radarays_sensor)
    {
        gzerr << "RadaRaysOptixROS requires a RadaRaysOptix sensor.\n";
        return;
    }

    if (!ros::isInitialized())
    {
        ROS_FATAL_STREAM_NAMED("laser", "A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
        return;
    }

    m_nh = std::make_shared<ros::NodeHandle>(m_robot_namespace);
    m_it = std::make_shared<image_transport::ImageTransport>(*m_nh);

    m_frame_id = m_sdf->Get<std::string>("frame");
    m_topic = m_sdf->Get<std::string>("topic");

    m_pub_img = m_it->advertise(m_topic, 1);

    // m_pub = m_nh->advertise<sensor_msgs::Image>()

    // sdf::ElementPtr outputsElem = m_sdf->GetElement("outputs");
    // parseOutputs(outputsElem);

    // // Connect to the sensor update event.
    m_update_conn = m_radarays_sensor->ConnectUpdated(
        std::bind(&RadaRaysOptixROS::OnUpdate, this));

    ROS_INFO_STREAM("[RadaRaysOptixROS] Loaded.");
}

void RadaRaysOptixROS::OnUpdate()
{
    // std::cout << "[RadaRaysOptixROS] Update!" << std::endl;

    common::Time stamp_gz = m_radarays_sensor->stamp();
    ros::Time stamp(stamp_gz.sec, stamp_gz.nsec);

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(
                std_msgs::Header(), 
                "mono8",
                m_radarays_sensor->m_polar_image).toImageMsg();

    msg->header.stamp = stamp;
    msg->header.frame_id = m_frame_id;

    m_pub_img.publish(msg);
}

GZ_REGISTER_SENSOR_PLUGIN(RadaRaysOptixROS)

} // namespace gazebo