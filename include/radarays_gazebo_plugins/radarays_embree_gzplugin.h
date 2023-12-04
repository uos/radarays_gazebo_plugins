#ifndef GAZEBO_RADARAYS_EMBREE_PLUGIN_H
#define GAZEBO_RADARAYS_EMBREE_PLUGIN_H

#include <ros/ros.h>

#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/sensors/RaySensor.hh>

#include <rmagine/math/types.h>
#include <rmagine/types/Memory.hpp>
#include <rmagine/types/sensor_models.h>
#include <rmagine/simulation/SphereSimulatorEmbree.hpp>
#include <rmagine/simulation/OnDnSimulatorEmbree.hpp>

#include <rmagine/noise/Noise.hpp>

#include <mutex>
#include <shared_mutex>
#include <memory>
#include <optional>
#include <unordered_map>


#include <rmagine_gazebo_plugins/rmagine_embree_spherical_gzplugin.h>

#include <radarays_ros/Radar.hpp>
#include <radarays_ros/RadarMaterials.h>
#include <radarays_ros/RadarModelConfig.h>
#include <radarays_ros/RadarParams.h>
#include <radarays_ros/radar_types.h>


#include <dynamic_reconfigure/server.h>


namespace rm = rmagine;

namespace gazebo
{

namespace sensors
{
class RadaRaysEmbree : public RmagineEmbreeSpherical
{
public:
    using Base = RmagineEmbreeSpherical;

    RadaRaysEmbree();
    virtual ~RadaRaysEmbree();

    virtual void Init() override;
    virtual void Load(const std::string& world_name) override;

    // simulation buffer
    cv::Mat m_polar_image;

protected:
    virtual bool UpdateImpl(const bool _force) override;

    void loadParamsSDF();
    void loadParams();

    void searchForMaterials();

    void simulate(rmagine::Transform Tsm);

// For Radar
    void updateDynCfg(radarays_ros::RadarModelConfig &config, uint32_t level);

    // ROS NODE
    std::shared_ptr<ros::NodeHandle> m_nh_p;

    std::unordered_map<unsigned int, rm::OnDnSimulatorEmbreePtr> m_sims;

    // DynRec
    std::shared_ptr<dynamic_reconfigure::Server<radarays_ros::RadarModelConfig> > m_dyn_rec_server;

     // Params
    radarays_ros::RadarParams m_params;
    rm::SphericalModel m_radar_model;
    radarays_ros::RadarModelConfig m_cfg;

    // materials
    int m_material_id_air = 0;
    std::vector<int> m_object_materials;
    bool m_materials_searched = false;

    float m_wave_energy_threshold = 0.001;
    std::vector<radarays_ros::DirectedWave> m_waves_start;
    bool m_resample = true;
    float m_max_signal = 120.0;
};

using RadaRaysEmbreePtr = std::shared_ptr<RadaRaysEmbree>;

// will by generated in cpp
void RegisterRadaRaysEmbree();

} // namespace sensors

} // namespace gazebo

#endif // GAZEBO_RADARAYS_EMBREE_PLUGIN_H