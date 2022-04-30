#include "actor_plugin.h"

namespace gazebo {
    constexpr double defaultVelocity = 0.5;
    constexpr double defaultAcceleration = 0.0;
    constexpr double defaultTargetDistanceApproximation = 0.05;
    constexpr bool defaultInLoop = true;

    ActorPlugin::ActorPlugin() : ModelPlugin()
    {
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
                return _sdf->Get<T>(paramName);
            else
                return defaultValue;
        };

        modelMovement.velocity = getParam("velocity", defaultVelocity);
        modelMovement.acceleration = getParam("acceleration", defaultAcceleration);
        inLoop = getParam("loop", defaultInLoop);
        targetDistanceApproximation = getParam("target_approximation", defaultTargetDistanceApproximation);

        ReadTrajectory(_sdf);
        currentTarget = modelMovement.trajectoryPoints.begin();
    }

    void ActorPlugin::OnUpdate(const common::UpdateInfo &info)
    {
        if(modelMovement.trajectoryPoints.begin() == modelMovement.trajectoryPoints.end())
            return;
        double dt = (info.simTime - prev_time).Double();
        ignition::math::Pose3d actorPose = actor->WorldPose();
        auto distance = (currentTarget->Pos() - actorPose.Pos()).Length();

        if(distance< targetDistanceApproximation)
        {
            if(std::next(currentTarget)==modelMovement.trajectoryPoints.end())
            {
                if(inLoop)
                    currentTarget = modelMovement.trajectoryPoints.begin();
                else
                    return;
            }
            else
                currentTarget = std::next(currentTarget);
        }

        auto direction = (currentTarget->Pos() - actorPose.Pos()).Normalize();
        actorPose.Pos() += direction * dt * modelMovement.velocity;

        double distanceTraveled = (actorPose.Pos() -
                                   this->actor->WorldPose().Pos()).Length();

        actor->SetWorldPose(actorPose, true, true);
        actor->SetScriptTime(actor->ScriptTime() + (distanceTraveled ));

        modelMovement.velocity += modelMovement.acceleration * dt;
        prev_time += dt;
    }

    void ActorPlugin::ReadTrajectory(sdf::ElementPtr _sdf)
    {
        if (_sdf->HasElement("trajectory"))
        {
            sdf::ElementPtr modelElem =
                    _sdf->GetElement("trajectory")->GetElement("pose");
            while (modelElem)
            {
                modelMovement.trajectoryPoints.push_back(modelElem->Get<ignition::math::Pose3d>());
                modelElem = modelElem->GetNextElement("pose");
            }
        }
    }

    GZ_REGISTER_MODEL_PLUGIN(ActorPlugin)
}
