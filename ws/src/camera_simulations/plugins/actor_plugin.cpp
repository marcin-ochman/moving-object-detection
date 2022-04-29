#include "actor_plugin.h"

namespace gazebo {
    constexpr double defaultVelocity = 0.5;
    constexpr double defaultAcceleration = 0.0;

    ActorPlugin::ActorPlugin() : ModelPlugin()
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

        auto getParam = [_sdf]<class T>(const std::string& paramName, T defaultValue) {
            if(_sdf->HasElement(paramName))
                return _sdf->Get<double>(paramName);
            else
                return defaultValue;
        };

        modelMovement.velocity = getParam("velocity", defaultVelocity);
        modelMovement.acceleration = getParam("acceleration", defaultAcceleration);
    }

    void ActorPlugin::OnUpdate(const common::UpdateInfo &info)
    {
        double dt = (info.simTime - prev_time).Double();
        ignition::math::Pose3d actorPose = actor->WorldPose();
        auto distance = (currentTarget - actorPose.Pos()).Length();

        if(distance< 0.05)
        {
            sign *= -1;
            currentTarget.X(currentTarget.X()*sign);
        }

        auto direction = (currentTarget - actorPose.Pos()).Normalize();
        actorPose.Pos() += direction * dt * modelMovement.velocity;

        double distanceTraveled = (actorPose.Pos() -
                                   this->actor->WorldPose().Pos()).Length();

        actor->SetWorldPose(actorPose, true, true);
        actor->SetScriptTime(actor->ScriptTime() + (distanceTraveled ));

        modelMovement.velocity += modelMovement.acceleration * dt;
        prev_time += dt;
    }

    GZ_REGISTER_MODEL_PLUGIN(ActorPlugin)
}
