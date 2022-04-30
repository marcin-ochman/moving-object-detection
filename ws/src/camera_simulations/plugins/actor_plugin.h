#ifndef CAMERA_SIMULATIONS_ACTOR_PLUGIN_H
#define CAMERA_SIMULATIONS_ACTOR_PLUGIN_H

#include <vector>

#include "gazebo/common/Plugin.hh"
#include "gazebo/util/system.hh"
#include "gazebo/physics/physics.hh"

namespace gazebo {
    struct Movement
    {
        double velocity;
        double acceleration;
        std::vector<ignition::math::Pose3d> trajectoryPoints;
    };


    class GZ_PLUGIN_VISIBLE ActorPlugin : public ModelPlugin {
    public:
    ActorPlugin();
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;
    private:

    void OnUpdate(const common::UpdateInfo &_info);
    void ReadTrajectory(sdf::ElementPtr _sdf);
    std::vector <event::ConnectionPtr> connections;

    physics::ActorPtr actor;

    std::vector<ignition::math::Pose3d>::iterator currentTarget;
    double prev_time;

    double targetDistanceApproximation;
    bool inLoop;
    Movement modelMovement;
};
}
#endif //CAMERA_SIMULATIONS_ACTOR_PLUGIN_H
