#include <radarays_gazebo_plugins/radarays_embree_ros_gzplugin.h>

#include <iostream>
#include <cv_bridge/cv_bridge.h>


namespace gazebo
{

RadaRaysEmbreeROS::RadaRaysEmbreeROS()
:Base()
{
    // ROS_INFO_STREAM("[RadaRaysEmbreeROS] Construct");
}

RadaRaysEmbreeROS::~RadaRaysEmbreeROS()
{
    // std::cout << "[RadaRaysEmbreeROS] Destroy" << std::endl;
}

void RadaRaysEmbreeROS::parseOutputs(sdf::ElementPtr outputs)
{
    // std::cout << "[RadaRaysEmbreeROS::parseOutputs]" << std::endl;

    // auto it = outputs->GetFirstElement();

    // while(it)
    // {
    //     if(it->GetName() == "output")
    //     {
    //         sdf::ParamPtr nameParam = it->GetAttribute("name");
    //         std::string name = nameParam->GetAsString();

    //         Publisher pub;
    //         pub.msg_type = it->Get<std::string>("msg");
    //         pub.topic = it->Get<std::string>("topic");

    //         if(pub.msg_type == "sensor_msgs/LaserScan")
    //         {
    //             pub.pub = std::make_shared<ros::Publisher>(
    //                     m_nh->advertise<sensor_msgs::LaserScan>(
    //                         pub.topic, 1
    //                     )
    //                 );
    //         }

    //         if(pub.msg_type == "sensor_msgs/PointCloud")
    //         {
    //             pub.pub = std::make_shared<ros::Publisher>(
    //                     m_nh->advertise<sensor_msgs::PointCloud>(
    //                         pub.topic, 1
    //                     ) 
    //                 );
    //         }

    //         if(pub.msg_type == "sensor_msgs/PointCloud2")
    //         {
    //             pub.pub = std::make_shared<ros::Publisher>(
    //                     m_nh->advertise<sensor_msgs::PointCloud2>(
    //                         pub.topic, 1
    //                     ) 
    //                 );

    //             if(it->HasElement("ordered"))
    //             {
    //                 bool ordered = it->Get<bool>("ordered");
                    
    //                 if(!ordered)
    //                 {
    //                     m_pcl2_unordered.insert(pub.topic);
    //                 }
    //             }
    //         }

    //         if(pub.pub)
    //         {
    //             m_pubs[name] = pub;
    //         } 
    //     }

    //     it = it->GetNextElement();
    // }
}

void RadaRaysEmbreeROS::Load(
    sensors::SensorPtr _sensor, 
    sdf::ElementPtr _sdf)
{
    m_sdf = _sdf;

    m_radarays_sensor =
        std::dynamic_pointer_cast<sensors::RadaRaysEmbree>(_sensor);

    if (!m_radarays_sensor)
    {
        gzerr << "RadaRaysEmbreeROS requires a RadaRaysEmbree sensor.\n";
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
        std::bind(&RadaRaysEmbreeROS::OnUpdate, this));

    ROS_INFO_STREAM("[RadaRaysEmbreeROS] Loaded.");
}

void RadaRaysEmbreeROS::OnUpdate()
{
    // std::cout << "[RadaRaysEmbreeROS] Update!" << std::endl;

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

GZ_REGISTER_SENSOR_PLUGIN(RadaRaysEmbreeROS)

} // namespace gazebo