// GraspLock: weld a brick to another model's link on command, release on command.
// Used for (1) the grasp lock (brick -> gripper/wrist) and (2) the clutch stand-in
// (brick -> the brick or baseplate it is seated on). A brick may hold several welds at
// once (e.g. grasp + clutch during hand-over); each is keyed by (child, parent model).
//
// Why: Gazebo/DART friction "creeps" — a brick held by friction slides along the
// finger pads by 0.1-0.3 mm whenever the arm moves (measured in Phase 4), while stud
// seating needs < ~0.05-0.1 mm. A real 10 N grip on a 2.3 g brick does not slip, so
// locking the grasp removes a simulator artefact. Grasping itself stays physical
// (fingers close by friction, the grasp is verified first); the lock is engaged only
// after that and released before the fingers open, so seating stays physical too.
//
// Usage (world plugin):
//   <plugin filename="lego_grasp_lock" name="lego::GraspLock"><topic>/lego/grasp_lock</topic></plugin>
// Commands (gz.msgs.StringMsg on the topic):
//   "attach <parent_model> <parent_link> <child_model>"  - fixed joint at the current relative pose
//   "detach <child_model> [<parent_model>]"            - remove that weld, or all welds of child
//   "release <model>"                                  - remove every weld the model is part of
//                                                         (as child or parent), e.g. before a reset

#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/DetachableJoint.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/transport/Node.hh>

namespace lego
{
class GraspLock : public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  void Configure(
    const gz::sim::Entity &, const std::shared_ptr<const sdf::Element> & sdf,
    gz::sim::EntityComponentManager &, gz::sim::EventManager &) override
  {
    const std::string topic = sdf->Get<std::string>("topic", "/lego/grasp_lock").first;
    node_.Subscribe(topic, &GraspLock::OnCommand, this);
    gzmsg << "GraspLock listening on [" << topic << "]" << std::endl;
  }

  void PreUpdate(const gz::sim::UpdateInfo &, gz::sim::EntityComponentManager & ecm) override
  {
    std::vector<std::string> commands;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      commands.swap(pending_);
    }
    for (const auto & c : commands) {
      std::istringstream in(c);
      std::string verb;
      in >> verb;
      if (verb == "attach") {
        std::string parent_model, parent_link, child_model;
        in >> parent_model >> parent_link >> child_model;
        Attach(ecm, parent_model, parent_link, child_model);
      } else if (verb == "detach") {
        std::string child_model, parent_model;
        in >> child_model >> parent_model;  // parent optional
        Detach(ecm, child_model, parent_model);
      } else if (verb == "release") {
        std::string model;
        in >> model;
        Release(ecm, model);
      } else {
        gzerr << "GraspLock: unknown command [" << c << "]" << std::endl;
      }
    }
  }

private:
  void OnCommand(const gz::msgs::StringMsg & msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_.push_back(msg.data());
  }

  gz::sim::Entity ModelByName(gz::sim::EntityComponentManager & ecm, const std::string & name)
  {
    return ecm.EntityByComponents(gz::sim::components::Model(), gz::sim::components::Name(name));
  }

  void Attach(
    gz::sim::EntityComponentManager & ecm, const std::string & parent_model,
    const std::string & parent_link, const std::string & child_model)
  {
    const auto key = std::make_pair(child_model, parent_model);
    if (joints_.count(key)) {
      gzwarn << "GraspLock: [" << child_model << "] already attached to [" << parent_model << "]" << std::endl;
      return;
    }
    const auto pm = ModelByName(ecm, parent_model);
    const auto cm = ModelByName(ecm, child_model);
    if (pm == gz::sim::kNullEntity || cm == gz::sim::kNullEntity) {
      gzerr << "GraspLock: model not found (" << parent_model << ", " << child_model << ")" << std::endl;
      return;
    }
    const auto pl = gz::sim::Model(pm).LinkByName(ecm, parent_link);
    const auto cl = gz::sim::Model(cm).CanonicalLink(ecm);
    if (pl == gz::sim::kNullEntity || cl == gz::sim::kNullEntity) {
      gzerr << "GraspLock: link not found (" << parent_link << " / canonical of " << child_model << ")"
            << std::endl;
      return;
    }
    const auto joint = ecm.CreateEntity();
    ecm.CreateComponent(joint, gz::sim::components::DetachableJoint({pl, cl, "fixed"}));
    joints_[key] = joint;
    gzmsg << "GraspLock: attached [" << child_model << "] to [" << parent_model << "::" << parent_link
          << "]" << std::endl;
  }

  void Detach(
    gz::sim::EntityComponentManager & ecm, const std::string & child_model,
    const std::string & parent_model)
  {
    int n = 0;
    for (auto it = joints_.begin(); it != joints_.end();) {
      if (it->first.first == child_model && (parent_model.empty() || it->first.second == parent_model)) {
        ecm.RequestRemoveEntity(it->second);
        it = joints_.erase(it);
        ++n;
      } else {
        ++it;
      }
    }
    gzmsg << "GraspLock: detached " << n << " weld(s) of [" << child_model << "]"
          << (parent_model.empty() ? "" : " from [" + parent_model + "]") << std::endl;
  }

  void Release(gz::sim::EntityComponentManager & ecm, const std::string & model)
  {
    int n = 0;
    for (auto it = joints_.begin(); it != joints_.end();) {
      if (it->first.first == model || it->first.second == model) {
        ecm.RequestRemoveEntity(it->second);
        it = joints_.erase(it);
        ++n;
      } else {
        ++it;
      }
    }
    gzmsg << "GraspLock: released " << n << " weld(s) involving [" << model << "]" << std::endl;
  }

  gz::transport::Node node_;
  std::mutex mutex_;
  std::vector<std::string> pending_;
  std::map<std::pair<std::string, std::string>, gz::sim::Entity> joints_;  // (child, parent) -> joint
};
}  // namespace lego

GZ_ADD_PLUGIN(
  lego::GraspLock, gz::sim::System, lego::GraspLock::ISystemConfigure,
  lego::GraspLock::ISystemPreUpdate)
