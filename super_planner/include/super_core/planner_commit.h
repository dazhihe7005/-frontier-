#pragma once

#include <data_structure/cmd_traj.h>
#include <functional>
#include <utility>

namespace super_planner {

// This is the single shared CmdTraj installation path used by both planner
// strategies. The gate validates the planning epoch before invoking install.
// The boolean is true only when a new shared CmdTraj is installed.
using PlannerCommitGate = std::function<bool(const std::function<bool()>&, bool)>;

template <class StateCommit>
bool commitPreparedTrajectory(CmdTraj& shared, CmdTraj& prepared,
                              const PlannerCommitGate& gate,
                              StateCommit&& state_commit) {
  const std::function<bool()> install = [&] {
    shared.installPrepared(prepared);
    std::forward<StateCommit>(state_commit)();
    return true;
  };
  return gate ? gate(install, true) : install();
}

}  // namespace super_planner
