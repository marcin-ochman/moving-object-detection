#ifndef CAMERA_SIMULATIONS_ACTOR_PLUGIN_H
#define CAMERA_SIMULATIONS_ACTOR_PLUGIN_H
#include <vector>

#include "gazebo/common/Plugin.hh"
#include "gazebo/util/system.hh"
#include "gazebo/physics/physics.hh"

namespace gazebo {
    class GZ_PLUGIN_VISIBLE ActorPlugin : public ModelPlugin {
    public:
        ActorPlugin();
        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;
    private:
        void OnUpdate(const common::UpdateInfo &_info);
        std::vector<event::ConnectionPtr> connections;
    };
}
#endif //CAMERA_SIMULATIONS_ACTOR_PLUGIN_H
