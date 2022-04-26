#include "actor_plugin.h"

namespace gazebo {
    ActorPlugin::ActorPlugin()
    {
        currentTarget = ignition::math::Vector3d(1, 0, 0.0);
        sign = 1;
        prev_time=0;
    }

    void ActorPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        std::cout<< "Loading plugin"<< std::endl;
        actor = boost::dynamic_pointer_cast<physics::Actor>(_model);
        connections.push_back(event::Events::ConnectWorldUpdateBegin(
                std::bind(&ActorPlugin::OnUpdate, this, std::placeholders::_1)));
    }

    void ActorPlugin::OnUpdate(const common::UpdateInfo &info)
    {
        double dt = (info.simTime - prev_time).Double();
        //        std::cout<< "Update" << std::endl;
        ignition::math::Pose3d actorPose = actor->WorldPose();
        auto distance = (currentTarget - actorPose.Pos()).Length();

        if(distance< 0.05)
        {
            sign *= -1;
            currentTarget.X(currentTarget.X()*sign);
        }

        auto direction = (currentTarget - actorPose.Pos()).Normalize();
        actorPose.Pos() += direction * dt * 0.25;

//        std::cout<< "Distance: "<< actorPose.Pos() - currentTarget << std::endl;
//        actorPose.Pos() += ignition::math::Vector3d(sign*dt, 0.0, 0);

        double distanceTraveled = (actorPose.Pos() -
                                   this->actor->WorldPose().Pos()).Length();

        actor->SetWorldPose(actorPose, false, false);
        actor->SetScriptTime(actor->ScriptTime() + (distanceTraveled * 4.4));
        prev_time += dt;
    }

    GZ_REGISTER_MODEL_PLUGIN(ActorPlugin)
}
