// Copyright 2027 SRM. Licensed under the Apache License, Version 2.0.
#include <chrono>
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/decorator_node.h"
#include "srm27_behavior/navigation_spin_tracker.hpp"

namespace srm27_behavior
{
// Reactive controls halt the previous branch after ticking the next one.
// Serialize navigation here so cancellation completes before another goal is sent.
class NavigationTransitionGate : public BT::DecoratorNode
{
public:
  NavigationTransitionGate(const std::string & name, const BT::NodeConfig & config)
  : BT::DecoratorNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<bool>("hold_success", true, "Hold a reached point until preempted"),
      BT::InputPort<double>("timeout_sec", "{@navigation_timeout}", "Per-attempt goal time budget")};
  }

  void halt() override
  {
    auto bb = config().blackboard->rootBlackboard();
    BT::TreeNode * owner = nullptr;
    if (bb->get("navigation_owner", owner) && owner == this) {
      bb->set<BT::TreeNode *>("navigation_owner", nullptr);
      std::shared_ptr<NavigationSpinTracker> tracker;
      if (bb->get("nav_spin_tracker", tracker) && tracker) { tracker->clearGoal(); }
    }
    reached_ = false;
    attempt_started_ = {};
    retry_after_ = {};
    BT::DecoratorNode::halt();
  }

private:
  bool reached_ = false;
  std::chrono::steady_clock::time_point retry_after_{};
  std::chrono::steady_clock::time_point attempt_started_{};
  BT::NodeStatus tick() override
  {
    auto bb = config().blackboard->rootBlackboard();
    bool dry_run = false;
    if (bb->get("dry_run", dry_run) && dry_run) { return BT::NodeStatus::RUNNING; }
    BT::TreeNode * owner = nullptr;
    (void)bb->get("navigation_owner", owner);
    if (owner && owner != this) {
      owner->haltNode();
    }
    bb->set<BT::TreeNode *>("navigation_owner", this);
    setStatus(BT::NodeStatus::RUNNING);
    bool hold = true;
    getInput("hold_success", hold);
    if (hold && reached_) {
      bool arrived = false;
      (void)bb->get("nav_arrived", arrived);
      if (arrived) { return BT::NodeStatus::RUNNING; }
      reached_ = false;
    }
    const auto now = std::chrono::steady_clock::now();
    if (now < retry_after_) { return BT::NodeStatus::RUNNING; }
    if (attempt_started_ != std::chrono::steady_clock::time_point{} &&
        std::chrono::duration<double>(now - attempt_started_).count() >= getInput<double>("timeout_sec").value()) {
      resetChild();
      attempt_started_ = {};
      retry_after_ = now + std::chrono::seconds(1);
      bb->set("nav_fault", std::string("navigation_timeout"));
      return BT::NodeStatus::RUNNING;
    }
    if (attempt_started_ == std::chrono::steady_clock::time_point{}) {
      attempt_started_ = now;
      bb->set("nav_fault", std::string(""));
    }
    const auto result = child_node_->executeTick();
    if (result != BT::NodeStatus::RUNNING) { attempt_started_ = {}; }
    if (result == BT::NodeStatus::FAILURE) {
      resetChild();
      retry_after_ = std::chrono::steady_clock::now() + std::chrono::seconds(1);
      return BT::NodeStatus::RUNNING;
    }
    reached_ = result == BT::NodeStatus::SUCCESS;
    return hold && reached_ ? BT::NodeStatus::RUNNING : result;
  }
};

// Bound an attempted recovery across ticks; halt preserves the cooldown, a new match resets it.
class RecoveryWindow : public BT::DecoratorNode
{
public:
  RecoveryWindow(const std::string & name, const BT::NodeConfig & config)
  : BT::DecoratorNode(name, config) {}
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<double>("timeout_sec", 25.0, "Recovery attempt budget"),
      BT::InputPort<double>("retry_sec", 15.0, "Cooldown after an unsuccessful attempt"),
      BT::InputPort<int>("match_epoch", "{@match_epoch}", "Reset for a new match"),
      BT::OutputPort<bool>("cooling", "Recovery cooldown is active")};
  }
private:
  int epoch_ = -1;
  bool started_ = false;
  std::chrono::steady_clock::time_point start_{}, retry_{};
  BT::NodeStatus tick() override
  {
    const auto now = std::chrono::steady_clock::now();
    const int epoch = getInput<int>("match_epoch").value();
    if (epoch != epoch_) { epoch_ = epoch; started_ = false; retry_ = {}; }
    if (now < retry_) { setOutput("cooling", true); return BT::NodeStatus::FAILURE; }
    setOutput("cooling", false);
    if (started_ && std::chrono::duration<double>(now - start_).count() >= getInput<double>("timeout_sec").value()) {
      resetChild();
      started_ = false;
      retry_ = now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(getInput<double>("retry_sec").value()));
      setOutput("cooling", true);
      return BT::NodeStatus::FAILURE;
    }
    const auto result = child_node_->executeTick();
    if (result == BT::NodeStatus::FAILURE) { started_ = false; }
    else if (!started_) { started_ = true; start_ = now; }
    return result;
  }
};
}  // namespace srm27_behavior

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<srm27_behavior::NavigationTransitionGate>("NavigationTransitionGate");
  factory.registerNodeType<srm27_behavior::RecoveryWindow>("RecoveryWindow");
}
