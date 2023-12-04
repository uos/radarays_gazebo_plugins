#include <radarays_gazebo_plugins/radarays_optix_gzplugin.h>

#include <gazebo/sensors/SensorFactory.hh>

#include <iostream>
#include <boost/algorithm/string/replace.hpp>

#include <rmagine/noise/GaussianNoise.hpp>
#include <rmagine/noise/UniformDustNoise.hpp>
#include <rmagine/noise/RelGaussianNoise.hpp>

#include <gazebo/common/Console.hh>

#include <radarays_ros/RadarGPU.hpp>

#include <boost/bind.hpp>

#include <radarays_ros/ros_helper.h>

#include <radarays_ros/radar_algorithms.h>
#include <radarays_ros/image_algorithms.h>
#include <radarays_ros/radar_algorithms.cuh>
#include <radarays_ros/image_algorithms.cuh>

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


RadaRaysOptix::RadaRaysOptix()
:Base() // if sensor is base class: Base(sensors::RAY)
{
    gzdbg << "[RadaRaysOptix] Constructed." << std::endl;
}

RadaRaysOptix::~RadaRaysOptix()
{
    gzdbg << "[RadaRaysOptix] Destroyed." << std::endl;
}

void RadaRaysOptix::loadParamsSDF()
{
    // sdf::ElementPtr rayElem = this->sdf->GetElement("ray");

    
    

}

void RadaRaysOptix::loadParams()
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

void RadaRaysOptix::uploadBuffers()
{
    rm::Memory<int, rm::RAM_CUDA> object_materials2(m_object_materials.size());
    for(size_t i=0; i<m_object_materials.size(); i++)
    {
        object_materials2[i] = m_object_materials[i];
    }
    m_object_materials_gpu = object_materials2;

    rm::Memory<RadarMaterial, rm::RAM_CUDA> materials2(m_params.materials.data.size());
    for(size_t i=0; i<m_params.materials.data.size(); i++)
    {
        materials2[i] = m_params.materials.data[i];
    }
    m_materials_gpu = materials2;
}

void RadaRaysOptix::searchForMaterials()
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
                    std::cout << "Missing RadaRays material in: " << model_name << "/" << link_name << "/" << vis_name << std::endl;
                    std::cout << "-- Setting RadaRays materials to defaults" << std::endl;
                    material.velocity = 0.0;
                    material.ambient  = 1.0;
                    material.diffuse  = 0.0;
                    material.specular = 3000.0;
                }
                    

                // figure out id in map

                std::string name = model_name + "::" + link_name + "::" + vis_name;

                // find model in optix_map
                if(m_map_mutex)
                {
                    m_map_mutex->lock_shared();
                }

                int obj_id = -1;
                if(m_map)
                {
                    std::cout << "SEARCH FOR: " << name << std::endl;
                    for(auto elem : m_map->scene()->geometries())
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
                    std::cout << "ERROR: no optix map" << std::endl;
                    continue;
                }

                if(m_map_mutex)
                {
                    m_map_mutex->unlock_shared();
                }
                
                if(obj_id >= 0)
                {
                    std::cout << "FOUND MODEL: " << name << std::endl;
                    std::cout << " -> optix map geometry id: " << obj_id << std::endl;

                    m_params.materials.data.push_back(material);
                    size_t mat_id = m_params.materials.data.size() - 1;

                    // fill with unknown with air materials
                    while(m_object_materials.size() <= obj_id)
                    {
                        m_object_materials.push_back(0);
                    }
                    // object -> material
                    m_object_materials[obj_id] = mat_id;
                } else {
                    std::cout << "COULD NOT FIND GEOMETRY IN OptixMap!" << std::endl;
                }
            }
        }
        
        // TODO:
        // full_name = model/link/...
        // put full name to optix map elements and compare instead
        // thus, we can apply materials to model parts
    }

    uploadBuffers();
}

void RadaRaysOptix::Init()
{
    Base::Init();

    ros::NodeHandle gazebo_nh("~");
    m_nh_p = std::make_shared<ros::NodeHandle>(gazebo_nh, "radarays");

    loadParams();

    m_dyn_rec_server = std::make_shared<dynamic_reconfigure::Server<radarays_ros::RadarModelConfig> >(
        *m_nh_p
    );

    dynamic_reconfigure::Server<radarays_ros::RadarModelConfig>::CallbackType f;
    f = boost::bind(&RadaRaysOptix::updateDynCfg, this, boost::placeholders::_1, boost::placeholders::_2);
    m_dyn_rec_server->setCallback(f);

    if(!m_map)
    {
        m_waiting_for_map = true;
    }
}

void RadaRaysOptix::Load(const std::string& world_name)
{
    Base::Load(world_name);
    std::cout << "LOAD - RadaRaysOptix!" << std::endl;
}

void RadaRaysOptix::simulate(rm::Transform Tsm)
{
    // 
    if(m_polar_image.rows != m_cfg.n_cells)
    {
        std::cout << "[RadaRaysOptix] Resize canvas to " << m_cfg.n_cells << std::endl;
        m_polar_image.resize(m_cfg.n_cells);
        std::cout << "[RadaRaysOptix] Resizing canvas - done." << std::endl;
    }

    // std::cout << "Fill canvas: " << m_polar_image.cols << "x" << m_polar_image.rows << std::endl;
    m_polar_image.setTo(cv::Scalar(0));

    std::vector<float> denoising_weights;
    int denoising_mode = 0;

    // std::cout << "[RadaRaysOptix] Sim!" << std::endl;
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

    // upload buffers
    rm::Memory<float, rm::VRAM_CUDA> denoising_weights_gpu;
    if(denoising_weights.size())
    {
        rm::Memory<float, rm::RAM_CUDA> denoising_weights2(denoising_weights.size());
        for(size_t i=0; i<denoising_weights.size(); i++)
        {
            denoising_weights2[i] = denoising_weights[i];
        }
        denoising_weights_gpu = denoising_weights2;
    }


    // el = sw();
    DirectedWaveAttributes wave_att_ex;
    wave_att_ex.energy       =  1.0;
    wave_att_ex.polarization =  0.5;
    wave_att_ex.frequency    = 76.5; // GHz
    wave_att_ex.velocity     =  0.3; // m / ns - speed in air
    wave_att_ex.material_id  =  0;   // air
    wave_att_ex.time         =  0.0; // ns

    int n_cells = m_polar_image.rows;
    int n_angles = m_polar_image.cols;

    // prepare radar model

    size_t n_rays = n_angles * m_cfg.n_samples;
    // std::cout << "Waves: " << n_rays << std::endl;
    
    
    rm::OnDnModel waves;
    waves.range = m_radar_model.range;
    waves.dirs.resize(n_rays);
    waves.origs.resize(n_rays);
    waves.width = n_angles;
    waves.height = m_params.model.n_samples;
    rm::Memory<DirectedWaveAttributes> wave_attributes(n_rays);


    // auto samples = sample_cone

    rm::Vector front = {1.0, 0.0, 0.0};
    rm::Memory<rm::Vector> ray_dirs_local = sample_cone(
        front,
        m_params.model.beam_width,
        m_params.model.n_samples,
        m_cfg.beam_sample_dist,
        m_cfg.beam_sample_dist_normal_p_in_cone
    );

    // std::cout << "Filling Model" << std::endl;

    for(size_t angle_id=0; angle_id<n_angles; angle_id++)
    {
        auto orig = m_radar_model.getOrigin(0, angle_id);
        auto dir = m_radar_model.getDirection(0, angle_id);

        rm::Transform Tas;
        Tas.R = rm::EulerAngles{0.0, 0.0, m_radar_model.getTheta(angle_id)};
        Tas.t = m_radar_model.getOrigin(0, angle_id);

        for(size_t sample_id = 0; sample_id < m_params.model.n_samples; sample_id++)
        {
            size_t buf_id = waves.getBufferId(sample_id, angle_id);
            waves.dirs[buf_id] = Tas.R * ray_dirs_local[sample_id];
            waves.origs[buf_id] = {0.0, 0.0, 0.0};
            wave_attributes[buf_id] = wave_att_ex;
        }
    }

    // std::cout << "Done filling model" << std::endl;

    // going to GPU

    // 1. preallocate everything necessary

    // m_sim->preBuildProgram<ResT>();

    // std::cout << "Prealloc memory buffers - start" << std::endl;

    // pass 1
    if(m_waves_gpu.size() != m_cfg.n_reflections)
    {
        std::cout << "Prealloc memory buffers - start" << std::endl;
        initWorkMemory();
        std::cout << "Prealloc memory buffers - end" << std::endl;
    }


   
    
    // rm::Transform Tsm = Tsm_last;
    rm::Memory<rm::Transform> Tsms(1);
    Tsms[0] = Tsm;
    // std::cout << "Start Simulation from pose: " << Tsm << std::endl;

    // m_sim->setModel(m_waves_gpu[0]);

    rm::StopWatch sw;
    double el_tot = 0.0;
    double el1;
    double el2;
    double el3;

    // 1. Generate Signals
    // sw();
    {
        // std::cout << "[RadaRaysOptix] start simulate" << std::endl;
        // sw();

        if(m_cfg.n_reflections > 0)
        {
             // upload first bunch of rays
            m_waves_gpu[0].origs = waves.origs;
            m_waves_gpu[0].dirs = waves.dirs;

            if(wave_attributes.size() != m_wave_attributes_gpu[0].size())
            {
                std::cout << "PREALLOCATED MEMORY WRONG!" << std::endl;
                return;
            }
            m_wave_attributes_gpu[0] = wave_attributes;


            // m_results.size() == Number of reflections
            // For each reflection
            // - sim
            // - reflection shader
            // - if 
            for(size_t i=0; i<m_cfg.n_reflections; i++)
            {
                m_sim->setModel(m_waves_gpu[i]);
                // std::cout << "[RadaRaysOptix] Simulate Signals - pass " << i+1 << std::endl;
                m_sim->simulate(Tsms, m_results[i]);
                // cudaDeviceSynchronize();
                // el = sw(); el_tot += el;
                // std::cout << "- ray cast: " << el*1000.0 << "ms" << std::endl;
                // std::cout << "[RadaRaysOptix] end simulate" << std::endl;

                // sw();
                move_waves(
                    m_waves_gpu[i].origs,
                    m_waves_gpu[i].dirs,
                    m_wave_attributes_gpu[i],
                    m_results[i].ranges, 
                    m_results[i].hits);
                // el = sw(); el_tot += el;

                // std::cout << "- move: " << el*1000.0 << "ms" << std::endl;
                if(m_materials_gpu.size() == 0 || m_object_materials.size() == 0)
                {
                    std::cout << "[RadaRaysOptix] WARNING. materials or object->materials buffers seam to have wrong sizes" << std::endl;
                    std::cout << "- Materials: " << m_materials_gpu.size() << std::endl;
                    std::cout << "- Objects with materials: " << m_object_materials.size() << std::endl;
                }
               
                // sw();
                signal_shader(
                    m_materials_gpu,
                    m_object_materials_gpu,
                    m_material_id_air,

                    m_waves_gpu[i].dirs,
                    m_wave_attributes_gpu[i],
                    m_results[i].hits,
                    m_results[i].normals,
                    m_results[i].object_ids,
                    
                    m_signals[i]
                );

                // sw();
                if(i < m_cfg.n_reflections - 1)
                {
                    // std::cout << i+1 << " -> SPLIT -> " << i+2 << std::endl;
                    // i -> i+1
                    auto lo = m_waves_gpu[i+1].origs(0, m_waves_gpu[i].origs.size());
                    auto ld = m_waves_gpu[i+1].dirs(0, m_waves_gpu[i].dirs.size());
                    auto la = m_wave_attributes_gpu[i+1](0, m_wave_attributes_gpu[i].size());
                    
                    auto ro = m_waves_gpu[i+1].origs(m_waves_gpu[i].origs.size(), m_waves_gpu[i].origs.size() * 2);
                    auto rd = m_waves_gpu[i+1].dirs(m_waves_gpu[i].dirs.size(), m_waves_gpu[i].dirs.size() * 2);
                    auto ra = m_wave_attributes_gpu[i+1](m_wave_attributes_gpu[i].size(), m_wave_attributes_gpu[i].size() * 2);

                    // FRESNEL SPLIT
                    fresnel_split(
                        m_materials_gpu,
                        m_object_materials_gpu,
                        m_material_id_air,
                        // INCIDENCE
                        m_waves_gpu[i].origs,
                        m_waves_gpu[i].dirs,
                        m_wave_attributes_gpu[i],
                        m_results[i].hits,
                        m_results[i].normals,
                        m_results[i].object_ids,
                        // SPLIT
                        lo, ld, la,
                        ro, rd, ra
                    );
                    // cudaDeviceSynchronize();
                } // fresnel split
            }
        }  
    }
    // el1 = sw(); el_tot += el1;
    
    // std::cout << "RUNTIME" << std::endl;
    // std::cout << "- Signal Gen: " << el1 << "s" << std::endl;


    // USE UNIFIED MEMORY
    rm::Memory<float, rm::UNIFIED_CUDA> img(n_cells * n_angles);
    rm::Memory<float, rm::UNIFIED_CUDA> max_vals(n_angles);
    rm::Memory<unsigned int, rm::UNIFIED_CUDA> signal_counts(n_angles);
    rm::Memory<float, rm::UNIFIED_CUDA> random_begins(n_angles);

    cudaMemset(img.raw(), 0, n_cells * n_angles * sizeof(float) );
    cudaMemset(max_vals.raw(), 0, n_angles * sizeof(float) );
    cudaMemset(signal_counts.raw(), 0, n_angles * sizeof(unsigned int) );

    std::vector<rm::Memory<Signal> > signals_cpu(m_cfg.n_reflections);
    std::vector<rm::Memory<uint8_t> > hits_cpu(m_cfg.n_reflections);

    for(size_t i=0; i<m_cfg.n_reflections; i++)
    {
        signals_cpu[i] = m_signals[i];
        hits_cpu[i] = m_results[i].hits;
    }

    cudaDeviceSynchronize();

    // 2. noise(signal+system) signals
    // sw();
    {
        size_t n_samples = m_cfg.n_samples;
        
        #pragma omp parallel for
        for(size_t angle_id_ = 0; angle_id_ < n_angles; angle_id_++)
        {
            size_t angle_id = (m_cfg.scroll_image + angle_id_) % n_angles;

            unsigned int img_offset = angle_id * n_cells;

            float max_val = 0.0;
            unsigned int signal_count = 0;    

            for(size_t pass_id=0; pass_id < m_cfg.n_reflections; pass_id++)
            {
                // Draw signals to slice
                // draw signals 1
                int size_factor = std::pow(2, pass_id);
                for(size_t sample_id=0; sample_id < n_samples * size_factor; sample_id++)
                {
                    const unsigned int signal_id = sample_id * n_angles + angle_id;

                    if(hits_cpu[pass_id][signal_id])
                    {
                        auto signal = signals_cpu[pass_id][signal_id];
                        // wave speed in air (light speed) * t / 2
                        float half_time = signal.time / 2.0;
                        float signal_dist = 0.3 * half_time;

                        int cell = static_cast<int>(signal_dist / m_cfg.resolution);
                        if(cell < n_cells)
                        {
                            if(m_cfg.signal_denoising > 0)
                            {
                                // signal denoising
                                for(int vid = 0; vid < denoising_weights.size(); vid++)
                                {
                                    int glob_id = vid + cell - denoising_mode;
                                    if(glob_id > 0 && glob_id < n_cells)
                                    {
                                        // TODO: check this
                                        const float old_val = img[img_offset + glob_id];
                                        const float new_val = old_val + signal.strength * denoising_weights[vid];
                                        img[img_offset + glob_id] = new_val;

                                        if(new_val > max_val)
                                        {
                                            max_val = new_val;
                                        }
                                    }
                                }
                            } else {
                                // read 
                                // TODO: check this
                                const float old_val = img[img_offset + cell];
                                const float new_val = std::max(old_val, (float)signal.strength);
                                img[img_offset + cell] = new_val;

                                if(new_val > max_val)
                                {
                                    max_val = new_val;
                                }
                            }

                            signal_count++;
                        }
                    }
                }
            }

            max_vals[angle_id] = max_val;
            signal_counts[angle_id] = signal_count;
        }
    
        // cudaDeviceSynchronize();
        // el2 = sw(); el_tot += el2;

        // 3. ambient noise
        // sw();
        if(m_cfg.ambient_noise)
        {
            std::random_device                      rand_dev;
            std::mt19937                            gen(rand_dev());
            std::uniform_real_distribution<float>   dist_uni(0.0, 1.0);

            for(size_t i=0; i<n_angles; i++)
            {
                max_vals[i] = std::max(
                    (float)m_cfg.ambient_noise_energy_max, 
                    max_vals[i]);
                
                random_begins[i] = dist_uni(gen) * 1000.0;
            }
            
            AmbientNoiseParams noise_params;
            noise_params.noise_at_signal_0 = m_cfg.ambient_noise_at_signal_0;
            noise_params.noise_at_signal_1 = m_cfg.ambient_noise_at_signal_1;
            noise_params.noise_energy_loss = m_cfg.ambient_noise_energy_loss;
            noise_params.noise_energy_min = m_cfg.ambient_noise_energy_min;
            noise_params.noise_energy_max = m_cfg.ambient_noise_energy_max;
            noise_params.resolution = m_cfg.resolution;

            // apply noise
            // low freq perlin
            double scale_low = m_cfg.ambient_noise_perlin_scale_low;
            // high freq perlin
            double scale_high = m_cfg.ambient_noise_perlin_scale_high;
            double p_low = m_cfg.ambient_noise_perlin_p_low;
            
            fill_perlin_noise_hilo(
                img, max_vals,
                n_angles, n_cells,
                random_begins, random_begins,
                scale_low, scale_high,
                p_low,
                noise_params
            );
            cudaDeviceSynchronize();
        }
        // el3 = sw(); el_tot += el3;

        // float max_signal = 120.0;
        float max_signal = m_cfg.signal_max;
        cv::Mat_<float> polar_img_f(n_cells, n_angles);

        for(size_t x=0; x<n_angles; x++)
        {
            float max_val = max_vals[x];
            
            for(size_t y=0; y<n_cells; y++)
            {
                polar_img_f.at<float>(y, x) = img[x * n_cells + y] * max_signal / max_val;
            }
        }
        polar_img_f.convertTo(m_polar_image, CV_8UC1);
    }
    
    // std::cout << std::fixed << std::setprecision(8) << el1/el_tot << ", " << el2/el_tot << ", " << el3/el_tot << ", " << el_tot << std::endl;
}

bool RadaRaysOptix::UpdateImpl(const bool _force)
{
    if(!m_map)
    {
        if(!m_waiting_for_map)
        {
            gzdbg << "[RadaRaysOptix] Start waiting for RmagineOptixMap..." << std::endl;
            m_waiting_for_map = true;
        }
        return false;
    } else {
        if(m_waiting_for_map) {
            gzdbg << "[RadaRaysOptix] Map received!" << std::endl;
            m_waiting_for_map = false;
        }

        if(!m_sim)
        {
            m_map->context()->getCudaContext()->use();
            m_sim = std::make_shared<rm::OnDnSimulatorOptix>(m_map);
            std::cout << "[RadaRaysOptix] OnDnSimulator constructed." << std::endl;
            
            m_sim->setMap(m_map);
            m_sim->setTsb(rm::Transform::Identity());
            std::cout << "[RadaRaysOptix] OnDnSimulator map set." << std::endl;
            
            m_sim->preBuildProgram<ResT>();
        }

        if(!m_materials_searched)
        {
            std::cout << "Search for radarays materials!!" << std::endl;
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

    m_map->context()->getCudaContext()->use();

    // fill global simulation buffer
    simulate(Tsm);

    if(m_map_mutex)
    {
        m_map_mutex->unlock_shared();
    }

    return Base::UpdateImpl(_force);
}

void RadaRaysOptix::initWorkMemory()
{
    std::cout << "Resizing working memory for " << m_cfg.n_reflections << " reflections passes" << std::endl;
    if(m_cfg.n_reflections == 0)
    {
        std::cout << "Cannot init working memory for 0 reflections!" << std::endl;
        return;
    }
    // update preallocated memory buffers
    m_waves_gpu.resize(m_cfg.n_reflections);
    m_wave_attributes_gpu.resize(m_cfg.n_reflections);
    m_results.resize(m_cfg.n_reflections);
    m_signals.resize(m_cfg.n_reflections);
    m_signal_mask.resize(m_cfg.n_reflections);

    int n_cells = m_cfg.n_cells;
    int n_angles = m_radar_model.theta.size;
    size_t n_rays = n_angles * m_cfg.n_samples;

    if(m_cfg.n_reflections > 0)
    {
        std::cout << "Alloc for Pass 1" << std::endl;
        // pass - i = 0
        {
            m_waves_gpu[0].width = n_angles;
            m_waves_gpu[0].height = m_params.model.n_samples;
            m_waves_gpu[0].range = m_radar_model.range;
            m_waves_gpu[0].origs.resize(n_rays);
            m_waves_gpu[0].dirs.resize(n_rays);
        }

        m_wave_attributes_gpu[0].resize(n_rays);
        
        rm::resize_memory_bundle<rm::VRAM_CUDA>(
            m_results[0], m_waves_gpu[0].width, m_waves_gpu[0].height, 1);
        m_signals[0].resize(m_waves_gpu[0].size());
        m_signal_mask[0].resize(m_waves_gpu[0].size());

        // pass - i > 0
        for(size_t i=1; i<m_cfg.n_reflections; i++)
        {
            std::cout << "Alloc for Pass " << i+1 << std::endl;
            // pass i

            // m_waves_gpu
            {
                m_waves_gpu[i].width = m_waves_gpu[i-1].width;
                m_waves_gpu[i].height = m_waves_gpu[i-1].height * 2;
                m_waves_gpu[i].range = m_waves_gpu[i-1].range;
                m_waves_gpu[i].origs.resize(m_waves_gpu[i-1].origs.size() * 2);
                m_waves_gpu[i].dirs.resize(m_waves_gpu[i-1].dirs.size() * 2);
            }

            m_wave_attributes_gpu[i].resize(m_wave_attributes_gpu[i-1].size() * 2);

            rm::resize_memory_bundle<rm::VRAM_CUDA>(
                m_results[i], m_waves_gpu[i].width, m_waves_gpu[i].height, 1);
            
            m_signals[i].resize(m_waves_gpu[i].size());
            m_signal_mask[i].resize(m_waves_gpu[i].size());
        }
    }
}

void RadaRaysOptix::updateDynCfg(
    radarays_ros::RadarModelConfig &config,
    uint32_t level)
{
    ROS_INFO("RadaRaysOptix - Changing Model");
    
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
    
    bool buffer_reinit = (config.n_reflections != m_params.model.n_reflections);
    
    m_params.model.n_reflections = config.n_reflections;

    m_cfg = config;

    if(buffer_reinit)
    {
        std::cout << "REINIT PREALLOCATED MEMORY BUFFER FOR " << m_params.model.n_reflections << " REFLECTIONS" << std::endl;
        initWorkMemory();
    }
}


GZ_REGISTER_STATIC_SENSOR("radarays_optix", RadaRaysOptix)

} // namespace sensors

} // namespace gazebo