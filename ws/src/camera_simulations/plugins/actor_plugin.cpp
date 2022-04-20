#include "actor_plugin.h"

namespace gazebo {
    ActorPlugin::ActorPlugin() {}

    void ActorPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        std::cout<< "Loading plugin"<< std::endl;
    }

    void ActorPlugin::OnUpdate(const common::UpdateInfo &_info)
    {
        std::cout<< "Update" << std::endl;
    }

    GZ_REGISTER_MODEL_PLUGIN(ActorPlugin)
}
