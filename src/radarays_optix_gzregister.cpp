#include <gazebo/gazebo.hh>
#include <radarays_gazebo_plugins/radarays_optix_gzplugin.h>


namespace gazebo
{
  class RegisterRadaRaysOptixPlugin : public SystemPlugin
  {
    /////////////////////////////////////////////
    /// \brief Destructor
    public: virtual ~RegisterRadaRaysOptixPlugin()
    {
    }

    /////////////////////////////////////////////
    /// \brief Called after the plugin has been constructed.
    public: void Load(int /*_argc*/, char ** /*_argv*/)
    {
      sensors::RegisterRadaRaysOptix();
    }

    /////////////////////////////////////////////
    // \brief Called once after Load
    private: void Init()
    {

    }

  };

  // Register this plugin with the simulator
  GZ_REGISTER_SYSTEM_PLUGIN(RegisterRadaRaysOptixPlugin)
}