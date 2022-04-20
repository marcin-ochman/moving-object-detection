#include "actor_plugin.h"

namespace gazebo {
    ActorPlugin::ActorPlugin()
    {
        currentTarget = ignition::math::Vector3d(1, 1, 0.15);
    }

    void ActorPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        std::cout<< "Loading plugin"<< std::endl;
        actor = boost::dynamic_pointer_cast<physics::Actor>(_model);
        connections.push_back(event::Events::ConnectWorldUpdateBegin(
                std::bind(&ActorPlugin::OnUpdate, this, std::placeholders::_1)));
    }

    void ActorPlugin::OnUpdate(const common::UpdateInfo &_info)
    {
        std::cout<< "Update" << std::endl;

        ignition::math::Pose3d actorPose = actor->WorldPose();

        std::cout<< "Distance: "<< actorPose.Pos() - currentTarget << std::endl;
        actorPose.Pos() += ignition::math::Vector3d(0.01, 0.01, 0);

        double distanceTraveled = (actorPose.Pos() -
                                   this->actor->WorldPose().Pos()).Length();

        actor->SetWorldPose(actorPose, false, false);
        actor->SetScriptTime(actor->ScriptTime() + distanceTraveled);
    }

    GZ_REGISTER_MODEL_PLUGIN(ActorPlugin)
}
