#include "actor_plugin.h"

namespace gazebo {
    ActorPlugin::ActorPlugin() {}

    void ActorPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {}

    GZ_REGISTER_MODEL_PLUGIN(ActorPlugin)
}
