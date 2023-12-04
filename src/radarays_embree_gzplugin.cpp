#include <radarays_gazebo_plugins/radarays_embree_gzplugin.h>

#include <gazebo/sensors/SensorFactory.hh>

#include <iostream>
#include <boost/algorithm/string/replace.hpp>

#include <rmagine/noise/GaussianNoise.hpp>
#include <rmagine/noise/UniformDustNoise.hpp>
#include <rmagine/noise/RelGaussianNoise.hpp>

#include <gazebo/common/Console.hh>

#include <radarays_ros/RadarCPU.hpp>

#include <boost/bind.hpp>

#include <radarays_ros/ros_helper.h>
#include <radarays_ros/radar_algorithms.h>
#include <radarays_ros/image_algorithms.h>
#include <cv_bridge/cv_bridge.h>
#include <omp.h>

#include <rmagine/util/StopWatch.hpp>
#include <rmagine/util/prints.h>

using namespace radarays_ros;
namespace rm = rmagine;

namespace gazebo
{

namespace sensors
{

static rm::Transform to_rm(const ignition::math::Pose3d& pose)
{
    rmagine::Transform T;
    T.R.x = pose.Rot().X();
    T.R.y = pose.Rot().Y();
    T.R.z = pose.Rot().Z();
    T.R.w = pose.Rot().W();
    T.t.x = pose.Pos().X();
    T.t.y = pose.Pos().Y();
    T.t.z = pose.Pos().Z();
    return T;
}


RadaRaysEmbree::RadaRaysEmbree()
:Base() // if sensor is base class: Base(sensors::RAY)
{
    gzdbg << "[RadaRaysEmbree] Constructed." << std::endl;
}

RadaRaysEmbree::~RadaRaysEmbree()
{
    m_dyn_rec_server.reset();
    gzdbg << "[RadaRaysEmbree] Destroyed." << std::endl;
}

void RadaRaysEmbree::loadParamsSDF()
{
    // sdf::ElementPtr rayElem = this->sdf->GetElement("ray");

    
    

}

void RadaRaysEmbree::loadParams()
{
    m_params = default_params();
    m_material_id_air = 0;
    m_wave_energy_threshold = 0.001;
    m_resample = true;

    m_radar_model.theta.inc = -(2 * M_PI) / 400;
    m_radar_model.theta.min = 0.0;
    m_radar_model.theta.size = 400;
    m_radar_model.phi.inc = 1.0;
    m_radar_model.phi.min = 0.0;
    m_radar_model.phi.size = 1;


    // 2 default materials: air, wall stone
    m_params.materials.data.resize(2);
    {
        radarays_ros::RadarMaterial material_air;
        material_air.velocity = 0.3; // m/ns
        material_air.ambient  = 1.0;
        material_air.diffuse  = 0.0;
        material_air.specular = 1.0;
        m_params.materials.data[0] = material_air;
        radarays_ros::RadarMaterial material_wall_stone;
        material_wall_stone.velocity = 0.0;
        material_wall_stone.ambient  = 1.0;
        material_wall_stone.diffuse  = 0.0;
        material_wall_stone.specular = 3000.0;
        m_params.materials.data[1] = material_wall_stone;
    }

    size_t n_default_objects = 100;

    m_object_materials.resize(n_default_objects);
    for(size_t i=0; i<n_default_objects; i++)
    {
        m_object_materials[i] = 1; // wall stone
    }

    // find real materials


    // this->sdf->GetElement("ray");


    m_polar_image = cv::Mat_<unsigned char>(0, m_radar_model.theta.size);




    loadParamsSDF();



    // material properties
    
    
    // TODO:
    // this is how it was in a static environment:
    // 
    // m_params.materials = loadRadarMaterialsFromParameterServer(m_nh_p);
    // m_nh_p->getParam("object_materials", m_object_materials);
    // m_nh_p->getParam("material_id_air", m_material_id_air);
    // 
    // But Gazebo environments are changing. Maybe we can fetch custom 
    // materials from Gazebo? 
    // - A first quick solution would be to pass a list of materials
    //   as before and does not allow to change the world (at own risk)
    // 
    // Or maybe: For each object spawn something is added to 
    //   dynamic reconfigure?





    // TODO: load additional params from gazebo - xml (instead of ros paremeters)
}

void RadaRaysEmbree::searchForMaterials()
{
    m_object_materials.resize(0);
    m_params.materials.data.resize(1);
    radarays_ros::RadarMaterial material_air;
    material_air.velocity = 0.3; // m/ns
    material_air.ambient  = 1.0;
    material_air.diffuse  = 0.0;
    material_air.specular = 1.0;
    m_params.materials.data[0] = material_air;



    sdf::ElementPtr world_sdf = this->world->SDF();
    sdf::ElementPtr model_sdf = world_sdf->GetElement("model");

    for(sdf::ElementPtr model_sdf = world_sdf->GetElement("model");
        model_sdf;
        model_sdf = model_sdf->GetNextElement("model"))
    {
        std::string model_name = model_sdf->GetAttribute("name")->GetAsString();

        for(sdf::ElementPtr link_sdf = model_sdf->GetElement("link");
            link_sdf;
            link_sdf = link_sdf->GetNextElement("link"))
        {
            std::string link_name = link_sdf->GetAttribute("name")->GetAsString();
            
        
            for(sdf::ElementPtr vis_sdf = link_sdf->GetElement("visual");
                vis_sdf;
                vis_sdf = vis_sdf->GetNextElement("visual"))
            {
                std::string vis_name = vis_sdf->GetAttribute("name")->GetAsString();
                
                sdf::ElementPtr mat_sdf = vis_sdf->GetElement("radarays_material");

                radarays_ros::RadarMaterial material;
                
                if(mat_sdf)
                {
                    std::cout << "Found RadaRays material in: " << model_name << "/" << link_name << "/" << vis_name << std::endl;
                    material.velocity = mat_sdf->Get<float>("velocity");
                    material.ambient = mat_sdf->Get<float>("ambient");
                    material.diffuse = mat_sdf->Get<float>("diffuse");
                    material.specular = mat_sdf->Get<float>("specular");
                } else {
                    // wall stone? defaults
                    std::cout << "-- Setting RadaRays materials to defaults" << std::endl;
                    material.velocity = 0.0;
                    material.ambient  = 1.0;
                    material.diffuse  = 0.0;
                    material.specular = 3000.0;
                }
                    

                // figure out id in map

                std::string name = model_name + "::" + link_name + "::" + vis_name;

                // find model in embree_map
                if(m_map_mutex)
                {
                    m_map_mutex->lock_shared();
                }

                int obj_id = -1;
                if(m_map)
                {
                    std::cout << "SEARCH FOR: " << name << std::endl;
                    for(auto elem : m_map->scene->geometries())
                    {
                        std::cout << elem.first << ".: " << elem.second->name << std::endl;
                        if(elem.second->name == name)
                        {
                            std::cout << "FOUND OBJ ID: " << elem.first << std::endl;
                            obj_id = elem.first;
                            break;
                        }
                    }
                } else {
                    std::cout << "ERROR: no embree map" << std::endl;
                    continue;
                }

                if(m_map_mutex)
                {
                    m_map_mutex->unlock_shared();
                }

                std::cout << "FOUND MODEL: " << name << std::endl;
                if(obj_id >= 0)
                {
                    std::cout << " -> embree map geometry id: " << obj_id << std::endl;

                    m_params.materials.data.push_back(material);
                    size_t mat_id = m_params.materials.data.size() - 1;

                    // fill with unknown with air materials
                    while(m_object_materials.size() <= obj_id)
                    {
                        m_object_materials.push_back(0);
                    }
                    // object -> material
                    m_object_materials[obj_id] = mat_id;
                }
            }
        }
        
        // TODO:
        // full_name = model/link/...
        // put full name to embree map elements and compare instead
        // thus, we can apply materials to model parts
        
    }

}

void RadaRaysEmbree::Init()
{
    Base::Init();

    ros::NodeHandle gazebo_nh("~");
    m_nh_p = std::make_shared<ros::NodeHandle>(gazebo_nh, "radarays");

    loadParams();

    m_dyn_rec_server = std::make_shared<dynamic_reconfigure::Server<radarays_ros::RadarModelConfig> >(
        *m_nh_p
    );
    dynamic_reconfigure::Server<radarays_ros::RadarModelConfig>::CallbackType f;
    f = boost::bind(&RadaRaysEmbree::updateDynCfg, this, boost::placeholders::_1, boost::placeholders::_2);
    m_dyn_rec_server->setCallback(f);

    if(!m_map)
    {
        m_waiting_for_map = true;
    }
}

void RadaRaysEmbree::Load(const std::string& world_name)
{
    Base::Load(world_name);
    std::cout << "LOAD - RadaRaysEmbree!" << std::endl;
}

void RadaRaysEmbree::simulate(rm::Transform Tsm)
{
    // 
    if(m_polar_image.rows != m_cfg.n_cells)
    {
        // m_polar_image = cv::Mat_<unsigned char>(n_cells, radar_model.theta.size);
        std::cout << "[RadaRaysEmbree] Resize canvas to " << m_cfg.n_cells << std::endl;
        m_polar_image.resize(m_cfg.n_cells);
        std::cout << "[RadaRaysEmbree] Resizing canvas - done." << std::endl;
    }

    // std::cout << "Fill canvas: " << m_polar_image.cols << "x" << m_polar_image.rows << std::endl;
    m_polar_image.setTo(cv::Scalar(0));

    std::vector<float> denoising_weights;
    int denoising_mode = 0;

    if(m_cfg.signal_denoising > 0)
    {
        // std::cout << "Signal Denoising: ";
        if(m_cfg.signal_denoising == 1)
        {
            // std::cout << "Triangular";
            denoising_mode = m_cfg.signal_denoising_triangular_mode * m_cfg.signal_denoising_triangular_width;
            denoising_weights = make_denoiser_triangular(
                m_cfg.signal_denoising_triangular_width,
                denoising_mode
            );
            
        } else if(m_cfg.signal_denoising == 2) {
            // std::cout << "Gaussian";
            denoising_mode = m_cfg.signal_denoising_gaussian_mode * m_cfg.signal_denoising_gaussian_width;
            denoising_weights = make_denoiser_gaussian(
                m_cfg.signal_denoising_gaussian_width,
                denoising_mode
            );

        } else if(m_cfg.signal_denoising == 3) {
            // std::cout << "Maxwell Boltzmann";
            denoising_mode = m_cfg.signal_denoising_mb_mode * m_cfg.signal_denoising_mb_width;
            denoising_weights = make_denoiser_maxwell_boltzmann(
                m_cfg.signal_denoising_mb_width,
                denoising_mode
            );
        }
        // std::cout << std::endl;

        // scale so that mode has weight 1
        // if(false)
        if(denoising_weights.size() > 0)
        {
            double denoising_mode_val = denoising_weights[denoising_mode];

            for(size_t i=0; i<denoising_weights.size(); i++)
            {
                denoising_weights[i] /= denoising_mode_val;
            }
        }
        
    }


    DirectedWave wave;
    wave.energy       =  1.0;
    wave.polarization =  0.5;
    wave.frequency    = 76.5; // GHz
    wave.velocity     =  0.3; // m / ns - speed in air
    wave.material_id  =  0;   // air
    wave.time         =  0.0; // ns
    wave.ray.orig = {0.0, 0.0, 0.0};
    wave.ray.dir = {1.0, 0.0, 0.0};

    // el = sw();

    int n_cells = m_polar_image.rows;
    int n_angles = m_polar_image.cols;

    // std::random_device                      rand_dev;
    // std::mt19937                            gen(rand_dev());
    // std::uniform_real_distribution<float>   dist_uni(0.0, 1.0);


    // without motion: update Tsm only once
    // if(!m_cfg.include_motion)
    // {
    //     // if(!updateTsm())
    //     // {
    //     //     std::cout << "Couldn't get Transform between sensor and map. Skipping..." << std::endl;
    //     //     return msg;
    //     // }
    // }

    if(m_resample)
    {
        m_waves_start = sample_cone_local(
            wave,
            m_params.model.beam_width,
            m_params.model.n_samples,
            m_cfg.beam_sample_dist,
            m_cfg.beam_sample_dist_normal_p_in_cone);
        m_resample = false;
    }

    rm::StopWatch sw_radar_sim;
    sw_radar_sim();

    bool enable_omp = false;
    
    
    #pragma omp parallel for
    for(size_t angle_id = 0; angle_id < n_angles; angle_id++)
    {

        int tid = 0;
        {
            // threaded. get sim that belong to current thread
            tid = omp_get_thread_num();
        }

        auto sims_it = m_sims.find(tid);
        if(m_sims.find(tid) == m_sims.end())
        {
            m_sims[tid] = std::make_shared<rm::OnDnSimulatorEmbree>(m_map);
            sims_it = m_sims.find(tid);
            auto Tsb = rm::Transform::Identity();
            // Tsm.t.z += 1.0;
            sims_it->second->setTsb(Tsb);

            #pragma omp critical
            std::cout << "Created new simulator for thread " << tid << std::endl; 
        }
        auto sim = sims_it->second;

        if(!sim)
        {
            std::cout << "ERROR!! Sim shared ptr empty" << std::endl;
        }
        
        std::vector<DirectedWave> waves = m_waves_start;

        rm::OnDnModel model = make_model(waves);
        sim->setModel(model);

        // with motion: update at each angle
        // if(m_cfg.include_motion)
        // {
        //     if(!updateTsm())
        //     {
        //         continue;
        //     }
        // }
        
        // make Tam ? angle to map Tam = Tsm * Tas
        // Tas is the transformation of angle to sensor
        
        rm::Transform Tas;
        Tas.R = rm::EulerAngles{0.0, 0.0, m_radar_model.getTheta(angle_id)};
        Tas.t = m_radar_model.getOrigin(0, angle_id);

        rm::Transform Tam = Tsm * Tas;

        // std::cout << "Simulating from: " << Tam << std::endl; 

        rm::Memory<rm::Transform> Tams(1);
        Tams[0] = Tam;

        // double el1 = sw();
        // std::cout << "el1: " << el1 << std::endl;
        // std::cout << "Create Signal" << std::endl;
        
        // double el1 = sw_inner();

        std::vector<Signal> signals;
        ///////
        /// 1. Signal generation
        for(size_t pass_id = 0; pass_id < m_params.model.n_reflections; pass_id++)
        {
            using ResT = rm::Bundle<
                rm::Hits<rm::RAM>,
                rm::Ranges<rm::RAM>,
                rm::Normals<rm::RAM>,
                rm::ObjectIds<rm::RAM> // connection to material
            >;

            ResT results;

            results.hits.resize(model.size());
            results.ranges.resize(model.size());
            results.normals.resize(model.size());
            results.object_ids.resize(model.size());
            
            // std::cout << "Simulate!" << std::endl;
            sim->simulate(Tams, results);
            // std::cout << "Simulate end!" << std::endl;
            
            // reflect / refract / absorb / return
            std::vector<DirectedWave> waves_new;

            // Move rays
            // #pragma omp parallel for // TODO: embree seems to break, when enabling multithreading?
            for(size_t i=0; i < waves.size(); i++)
            {
                // get
                DirectedWave wave = waves[i];
                const float wave_range = results.ranges[i];
                const rmagine::Vector surface_normal = results.normals[i].normalize();
                const unsigned int obj_id = results.object_ids[i];


                if(obj_id > 10000)
                {
                    continue;
                }
                
                // do
                const DirectedWave incidence = wave.move(wave_range);

                // inititalize
                DirectedWave reflection = incidence;
                DirectedWave refraction = incidence;

                // if wave was in air, switch to new material
                // else if wave was in material, switch to air (is this right ?)
                if(incidence.material_id == m_material_id_air)
                {
                    refraction.material_id = m_object_materials[obj_id];
                } else {
                    refraction.material_id = m_material_id_air;
                }

                float v_refraction = 1.0;

                if(incidence.material_id != refraction.material_id)
                {
                    v_refraction = m_params.materials.data[refraction.material_id].velocity;
                } else {
                    v_refraction = incidence.velocity;
                }

                // Build surface patch
                auto res = fresnel(surface_normal, incidence, v_refraction);

                reflection.ray.dir = res.first.ray.dir;
                reflection.energy = res.first.energy;

                if(reflection.energy > m_wave_energy_threshold)
                {
                    waves_new.push_back(reflection);
                    
                    // split total reflection energy into 
                    // - scene reflection energy
                    // - path return energy
                    // - air return energy
                    //
                    // path return energy = air return energy 
                    //      for pass 0
                    // path return energy = air return energy = scene reflection energy
                    //      for pass 0 and perpendicular hit
                    
                    if(reflection.material_id == m_material_id_air)
                    {
                        // 1. signal travelling back along the path

                        auto material = m_params.materials.data[refraction.material_id];

                        double incidence_angle = get_incidence_angle(surface_normal, incidence);
                        // 1. signal traveling over path
                        double return_energy_path = back_reflection_shader(
                            incidence_angle,
                            reflection.energy,
                            material.ambient, // ambient
                            material.diffuse, // diffuse
                            material.specular // specular
                        );

                        // signal traveling the air path == traversal path
                        if(pass_id == 0 || m_cfg.record_multi_reflection)
                        {   
                            float time_back = incidence.time * 2.0;
                            signals.push_back({time_back, return_energy_path});
                        }

                        if(pass_id > 0 && m_cfg.record_multi_path)
                        {
                            // 2. signal traveling the air path
                            Signal signal_air;
                            // signal_path.time = incidence.time * 2.0;
                            // signal_path.strength = return_energy_path;
                            
                            rm::Vector dir_sensor_to_hit = reflection.ray.orig;
                            const double distance_between_sensor_and_hit = dir_sensor_to_hit.l2norm();
                            dir_sensor_to_hit.normalizeInplace();
                            // time_to_sensor in [ns] assuming transmission through space
                            const double time_to_sensor = distance_between_sensor_and_hit / reflection.velocity;


                            double sensor_view_scalar = wave.ray.dir.dot(dir_sensor_to_hit);

                            double angle_between_reflection_and_sensor_dir 
                                = angle_between(-reflection.ray.dir, dir_sensor_to_hit);

                            if(m_cfg.record_multi_path 
                                && sensor_view_scalar > m_cfg.multipath_threshold)
                            {
                                double return_energy_air = back_reflection_shader(
                                    angle_between_reflection_and_sensor_dir,
                                    reflection.energy,
                                    material.ambient, // ambient
                                    material.diffuse, // diffuse
                                    material.specular // specular
                                );

                                signal_air.time = incidence.time + time_to_sensor;
                                signal_air.strength = return_energy_air;
                                
                                signals.push_back(signal_air);
                            }
                        }
                    }
                }

                refraction.ray.dir = res.second.ray.dir;
                refraction.energy = res.second.energy;

                if(refraction.energy > m_wave_energy_threshold)
                {
                    waves_new.push_back(refraction);
                }
            }

            // skip certain distance for safety
            float skip_dist = 0.001;
            for(size_t i=0; i<waves_new.size(); i++)
            {
                waves_new[i].moveInplace(skip_dist);
            }

            waves = waves_new;

            // update sensor model
            if(pass_id < m_params.model.n_reflections - 1)
            {
                model = make_model(waves);
                sim->setModel(model);
            } else {
                // last run. dont need to update
            }
            
            // std::cout << "Angle " << angle_id << " - pass " << pass_id << " - done." << std::endl;
        }


        // double el2 = sw_inner();

        //////////////////
        /// 2. Signals -> Canvas
        /// 2.1. Signal Noise -> Slice
        /// 2.2. Ambient noise -> Slice
        /// 2.3. Slice -> Canvas
        cv::Mat_<float> slice(m_polar_image.rows, 1, 0.0);

        float max_val = 0.0;
        // Draw signals to slice
        for(size_t i=0; i<signals.size(); i++)
        {
            auto signal = signals[i];
            // wave speed in air (light speed) * t / 2
            float half_time = signal.time / 2.0;
            float signal_dist = 0.3 * half_time;

            int cell = static_cast<int>(signal_dist / m_cfg.resolution);
            if(cell < slice.rows)
            {
                // float signal_old = slice.at<float>(cell, 0);

                if(m_cfg.signal_denoising > 0)
                {
                    // signal denoising
                    for(int vid = 0; vid < denoising_weights.size(); vid++)
                    {
                        int glob_id = vid + cell - denoising_mode;
                        if(glob_id > 0 && glob_id < slice.rows)
                        {
                            slice.at<float>(glob_id, 0) += signal.strength * denoising_weights[vid];

                            if(slice.at<float>(glob_id, 0) > max_val)
                            {
                                max_val = slice.at<float>(glob_id, 0);
                            }
                        }
                    }
                } else {
                    // no signal denoising

                    // slice.at<float>(cell, 0) += 1.0;
                    // slice.at<float>(cell, 0) += signal.strength;
                    slice.at<float>(cell, 0) = std::max(
                        slice.at<float>(cell, 0),
                        (float)signal.strength
                    );

                    if(slice.at<float>(cell, 0) > max_val)
                    {
                        max_val = slice.at<float>(cell, 0);
                    }
                }
            }
        }

        // std::cout << angle_id << " -> " << max_val << std::endl;

        // normalize
        // slice *= m_cfg.energy_max;
        // max_val *= m_cfg.energy_max;
        


        int col = (m_cfg.scroll_image + angle_id) % m_polar_image.cols;

        if(m_cfg.ambient_noise)
        {
            max_val = std::max((float)m_cfg.ambient_noise_energy_max, max_val);

            std::random_device                      rand_dev;
            std::mt19937                            gen(rand_dev());
            std::uniform_real_distribution<float>   dist_uni(0.0, 1.0);
            double random_begin = dist_uni(gen) * 1000.0;
            
            // apply noise
            // low freq perlin
            double scale_low = m_cfg.ambient_noise_perlin_scale_low;
            // high freq perlin
            double scale_high = m_cfg.ambient_noise_perlin_scale_high;
            double p_low = m_cfg.ambient_noise_perlin_p_low;

            // scale_low = 0.01;

            
            
            for(size_t i=0; i<slice.rows; i++)
            {
                float signal = slice.at<float>(i);

                double p;

                if(m_cfg.ambient_noise == 1) // UNIFORM
                {
                    p = dist_uni(gen);
                } else if(m_cfg.ambient_noise == 2) // PERLIN
                {
                    p = perlin_noise_hilo(random_begin, random_begin,
                            static_cast<double>(col), static_cast<double>(i),
                            scale_low, scale_high,
                            p_low);
                }
                
                // p = p * 
                // p = (p + 1.0) / 2.0; // [0.0-1.0]

                // verwurschteltn
                float signal_min = 0;
                float signal_max = max_val;
                float signal_amp = signal_max - signal_min;

                float signal_ = 1.0 - ((signal - signal_min) / signal_amp);

                float noise_at_0 = signal_amp * m_cfg.ambient_noise_at_signal_0;
                float noise_at_1 = signal_amp * m_cfg.ambient_noise_at_signal_1;

                float signal__ = std::pow(signal_, 4.0);

                float noise_amp = (signal__ * noise_at_0 + (1.0 - signal__) * noise_at_1);

                // noise_amp * p * signal_max;
                
                float noise_energy_max = signal_max * m_cfg.ambient_noise_energy_max;
                float noise_energy_min = signal_max * m_cfg.ambient_noise_energy_min;
                float energy_loss = m_cfg.ambient_noise_energy_loss;

                float y_noise = noise_amp * p;

                float x = (static_cast<float>(i) + 0.5) * m_cfg.resolution;

                y_noise = y_noise + (noise_energy_max - noise_energy_min) * exp(-energy_loss * x) + noise_energy_min;
                y_noise = abs(y_noise);

                slice.at<float>(i) = signal + y_noise;
            }
        }

        slice *= m_cfg.signal_max / max_val;
        slice.convertTo(m_polar_image.col(col), CV_8UC1);
    }

    double el_radar_sim = sw_radar_sim();

    // std::cout << "SIM in " << el_radar_sim << "s" << std::endl;
    // std::cout << std::fixed << std::setprecision(8) << el_radar_sim << std::endl;
}

bool RadaRaysEmbree::UpdateImpl(const bool _force)
{
    if(!m_map)
    {
        if(!m_waiting_for_map)
        {
            gzdbg << "[RadaRaysEmbree] Start waiting for RmagineEmbreeMap..." << std::endl;
            m_waiting_for_map = true;
        }
        return false;
    } else {
        if(m_waiting_for_map) {
            gzdbg << "[RadaRaysEmbree] Map received!" << std::endl;
            m_waiting_for_map = false;
        }

        if(!m_materials_searched)
        {
            std::cout << "Search for materials!!" << std::endl;
            searchForMaterials();
            m_materials_searched = true;
        }
    }

    // std::cout << "RADARAYS UPDATE" << std::endl;
    auto pose = this->parentEntity->WorldPose();
    rm::Transform Tbm = to_rm(pose);
    rm::Transform Tsm = Tbm * m_Tsb;

    // std::cout << "World Pose of sensor: " << Tsm << std::endl;
    if(m_map_mutex)
    {
        m_map_mutex->lock_shared();
    }

    // fill global simulation buffer
    simulate(Tsm);

    if(m_map_mutex)
    {
        m_map_mutex->unlock_shared();
    }

    return Base::UpdateImpl(_force);
}


void RadaRaysEmbree::updateDynCfg(
    radarays_ros::RadarModelConfig &config,
    uint32_t level)
{
    ROS_INFO("RadaRaysEmbree - Changing Model");
    
    // z_offset
    // auto T = rm::Transform::Identity();
    // T.t.z = config.z_offset;
    // sim->setTsb(T);

    if(   config.beam_sample_dist != m_cfg.beam_sample_dist 
        || abs(config.beam_width - m_cfg.beam_width ) > 0.001
        || config.n_samples != m_cfg.n_samples
        || abs(config.beam_sample_dist_normal_p_in_cone - m_cfg.beam_sample_dist_normal_p_in_cone) > 0.001
    )
    {
        m_resample = true;
    }

    // update radar model
    m_radar_model.range.min = config.range_min;
    m_radar_model.range.max = config.range_max;

    // update params model
    m_params.model.beam_width = config.beam_width * M_PI / 180.0;
    m_params.model.n_samples = config.n_samples;
    m_params.model.n_reflections = config.n_reflections;

    m_cfg = config;
}


GZ_REGISTER_STATIC_SENSOR("radarays_embree", RadaRaysEmbree)

} // namespace sensors

} // namespace gazebo