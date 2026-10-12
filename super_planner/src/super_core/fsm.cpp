/**
* This file is part of SUPER
*
* Copyright 2025 Yunfan REN, MaRS Lab, University of Hong Kong, <mars.hku.hk>
* Developed by Yunfan REN <renyf at connect dot hku dot hk>
* for more information see <https://github.com/hku-mars/SUPER>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* SUPER is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* SUPER is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with SUPER. If not, see <http://www.gnu.org/licenses/>.
*/

#include <fsm/fsm.h>
#include <memory>
#include <atomic>
#include <chrono>

using namespace super_utils;

namespace fsm {
    Fsm::~Fsm() {
        write_time_.close();
    }

    void Fsm::WriteTimeToLog() {
        if (!ros_ptr_) return;
        write_time_ << (ros_ptr_->getSimTime() - system_start_time_) << ", ";
        for (long unsigned int i = 0; i < log_module_time.size(); i++) {
            write_time_ << log_module_time[i];
            if (i != log_module_time.size() - 1) {
                write_time_ << ", ";
            }
        }
        write_time_ << endl;
    }

    void Fsm::callReplanOnce() {
        if (stop) return;
        const auto token = lifecycle_.snapshot();
        if (!token) return;
        Vec3f goal_p;
        double goal_yaw = NAN;
        bool new_goal = false;
        bool should_plan = false;
        uint64_t stage = 0, revision = 0;
        lifecycle_.runIfCurrent(*token, [&] {
            if (machine_state_ != FOLLOW_TRAJ || finish_plan) return;
            if (plan_from_rest_) {
                plan_from_rest_ = false;
                return;
            }
            goal_p = gi_.goal_p;
            goal_yaw = gi_.goal_yaw;
            new_goal = gi_.new_goal;
            stage = planning_stage_;
            revision = committed_traj_revision_;
            should_plan = true;
        });
        if (!should_plan) return;
        TimeConsuming replan_once_time("replan_once_time", false);

        const super_planner::PlannerCommitGate gate = [&](const std::function<bool()>& install, bool installs_trajectory) {
            return commitTrajectoryIfCurrent(*token, install, installs_trajectory);
        };
        super_planner::PlannerRunSnapshot run;
        RET_CODE ret_code = executeReplan(goal_p, goal_yaw, new_goal, gate, &run);
        if (ret_code == FAILED) {
//            cout << YELLOW << " -- [Fsm] ReplanOnce failed." << RESET << endl;
        } else { cout << GREEN << " -- [Fsm] ReplanOnce succeed." << RESET << endl; }

        bool publish_trajectory = false;
        applyReplanResultIfCurrent(*token, stage, revision, ret_code, &publish_trajectory);
        if (publish_trajectory) publishPolyTrajIfCurrent(*token);

        const double elapsed = replan_once_time.stop();
        {
            std::lock_guard<std::mutex> lock(replan_log_mutex_);
            log_module_time = std::move(run.module_time);
            if (log_module_time.size() >= 2) log_module_time[log_module_time.size() - 2] = elapsed;
            replan_logs_.push_back(std::move(run.log));
            WriteTimeToLog();
        }
    }

    RET_CODE Fsm::executeReplan(const Vec3f& goal_p, double goal_yaw, bool new_goal,
                                const super_planner::PlannerCommitGate& gate,
                                super_planner::PlannerRunSnapshot* snapshot) {
        Vec3f adjusted_goal = goal_p;
        planner_ptr_->getMap()->getNearestInfCellNot(GridType::OCCUPIED, goal_p, adjusted_goal, 3.0);
        return planner_ptr_->ReplanOnce(adjusted_goal, goal_yaw, new_goal, gate, snapshot);
    }

    void Fsm::callMainFsmOnce() {
        if (stop) return;
        const double now=ros_ptr_->getSimTime();
        if (last_main_clock_>=0.0 && now<last_main_clock_-1e-6) {
            cancelGoal(0); // revoke in-flight plans from the previous clock epoch
            ros_ptr_->warn(" -- [Fsm] CLOCK_EPOCH_RESET: old goal/trajectory invalidated; explicit new goal required.");
            last_main_clock_=now;
            last_odom_warning_=-1.0;
            return;
        }
        last_main_clock_=now;
        rog_map::RobotState robot_state;
        planner_ptr_->getRobotState(robot_state);
        if (!planningInputFresh()) {
            if(last_odom_warning_<0.0 || now-last_odom_warning_>=1.0) {
                last_odom_warning_=now;
                ros_ptr_->warn(" -- [Fsm] No valid odom: received={}, age={} s, timeout={} s",
                               robot_state.rcv,now-robot_state.rcv_time,cfg_.odom_timeout);
            }
            return; // every invocation, not only the once-per-second log branch
        }
        const auto token = lifecycle_.snapshot();
        if (!token) return;
        GoalInfo goal;
        bool should_plan = false;
        bool reset_path = false;
        bool publish_pose = false;
        lifecycle_.runIfCurrent(*token, [&] {
          robot_state_ = robot_state;
          switch (machine_state_) {
            case INIT: {
                if (!started_) {
                    return;
                }
                const double odom_age = ros_ptr_->getSimTime() - robot_state_.rcv_time;
                if (!robot_state_.rcv || odom_age > cfg_.odom_timeout) {
                    cout << YELLOW << " -- [Fsm] No odom (age=" << odom_age
                         << " s, timeout=" << cfg_.odom_timeout << " s)." << RESET << endl;
                }
                ChangeState("MainFsmCallback", WAIT_GOAL);
                break;
            }
            case WAIT_GOAL: {
                if (!gi_.new_goal) {
                    return;
                } else {
                    ChangeState("MainFsmCallback", GENERATE_TRAJ);
                }
                reset_path = true;
                break;
            }
            case GENERATE_TRAJ: {
                if (closeToGoal(0.1)) {
                    ChangeState("MainFsmCallback", WAIT_GOAL);
                    gi_.new_goal = false;
                    finish_plan = true;
                    return;
                }
                goal = gi_;
                should_plan = true;
                break;
            }
            case FOLLOW_TRAJ: {
                publish_pose = true;
                break;
            }
            case EMER_STOP: {
                ChangeState("MainFsmCallback", WAIT_GOAL);
                break;
            }
            default:
                break;
          }
        });
        if (reset_path) resetVisualizedPath();
        if (publish_pose) lifecycle_.runIfCurrent(*token, [&] { publishCurPoseToPath(); });
        if (should_plan) {
                bool publish_trajectory = false;
                bool reject_failed_goal = false;
                const super_planner::PlannerCommitGate gate = [&](const std::function<bool()>& install, bool installs_trajectory) {
                    return commitTrajectoryIfCurrent(*token, install, installs_trajectory);
                };
                super_planner::PlannerRunSnapshot run;
                int retcode = planner_ptr_->PlanFromRest(goal.goal_p, goal.goal_yaw, goal.new_goal, gate, &run);
                const bool goal_valid = run.goal_valid;
                lifecycle_.runIfCurrent(*token, [&] {
                  if (!goal_valid) {
                    cout << YELLOW << " -- [Fsm] Goal is invalid, skip this goal." << RESET << endl;
                    ChangeState("MainFsmCallback", WAIT_GOAL);
                    gi_.new_goal = false;
                    reject_failed_goal = true;
                    return;
                  }
                  if (retcode == SUCCESS || retcode == FINISH) {
                    consecutive_plan_failures_ = 0;
                    first_plan_failure_time_ = -1.0;
                    gi_.new_goal = false;
                    plan_from_rest_ = true;
                    finish_plan = false;
                    if (retcode == FINISH) {
                        finish_plan = true;
                    }

                    publish_trajectory = committed_traj_epoch_ == token->epoch;

                    ChangeState("MainFsmCallback", FOLLOW_TRAJ);
                } else {
                    const double now = ros_ptr_->getSimTime();
                    if (consecutive_plan_failures_++ == 0)
                        first_plan_failure_time_ = now;
                    if (consecutive_plan_failures_ >= 3 &&
                        now - first_plan_failure_time_ >= 0.5) {
                        cout << YELLOW << " -- [Fsm] Repeated PlanFromRest failure; rejecting goal "
                             << token->goal_id << RESET << endl;
                        ChangeState("MainFsmCallback", WAIT_GOAL);
                        gi_.new_goal = false;
                        reject_failed_goal = true;
                    }
                }
                });
                if (reject_failed_goal && lifecycle_.activeGoalId() == token->goal_id)
                    notifyGoalRejected(token->goal_id);
                if (publish_trajectory) publishPolyTrajIfCurrent(*token);
                {
                    std::lock_guard<std::mutex> lock(replan_log_mutex_);
                    replan_logs_.push_back(std::move(run.log));
                }
        }
    }

    bool Fsm::closeToGoal(const double &thresh_dis) {
        /// The close to goal should consider the the local shift
        /// All goal should be in the known free on inf map.
        /// The intermedia points should be in free space.
        double dis = (robot_state_.p - gi_.goal_p).norm();
        return dis < thresh_dis;
    }

    bool Fsm::setGoalPosiAndYaw(const Vec3f &p, const Quatf &q, uint64_t goal_id) {
        if (goal_id == 0) return false;

        auto click_point = p;
        if (cfg_.click_height > -5) {
            click_point.z() = cfg_.click_height;
        }

        Vec3f goal_p;
        if (planner_ptr_->getMap()->getNearestInfCellNot(GridType::OCCUPIED, click_point, goal_p, 3.0)) {
            if ((goal_p - click_point).norm() > cfg_.goal_snap_max_distance) {
                ros_ptr_->warn(" -- [Fsm] Goal shifted {:.2f} m by inflated map; reject instead of flying to a different return point.",
                               (goal_p - click_point).norm());
                return false;
            }
            cout << GREEN << " -- [Fsm] Get goal at " << RESET << goal_p.transpose() << endl;
        } else {
            fmt::print(fg(fmt::color::indian_red), "Goal is deeply occupied, skip this goal.\n");
            return false;
        }
        rog_map::RobotState robot_state;
        planner_ptr_->getRobotState(robot_state);
        if ((robot_state.p - goal_p).norm() <
            0.1) {
            //                print(fg(color::gray), " -- [Rviz] Too close to goal, skip this target.\n");
            return false;
        }
        double goal_yaw = NAN;
        if (cfg_.click_yaw_en) {
            if (isnan(q.w()) || isnan(q.x()) || isnan(q.y()) || isnan(q.z())) {
                goal_yaw = NAN;
                ros_ptr_->info(" -- [Fsm] Receive click goal at: [{}, {}, {}]; goal yaw disabled",
                               goal_p.x(), goal_p.y(), goal_p.z());
            } else {
                goal_yaw = geometry_utils::get_yaw_from_quaternion(q);
                cout << GREEN << " -- [Fsm] Receive click goal at: [" << goal_p.transpose() << "]; goal yaw: "
                     << goal_yaw * 57.3 << " deg" << RESET << endl;
            }

        } else {
            cout << GREEN << " -- [Fsm] Receive click goal at: [" << goal_p.transpose() << "]; goal yaw disabled"
                 << RESET << endl;
        }
        activateGoalState(goal_p, goal_yaw, goal_id,
                          cfg_.goal_command_en &&
                          planner_ptr_->canRetargetCurrentTrajectory(0.4));
        return true;
    }

    void Fsm::activateGoalState(const Vec3f& goal_p, double goal_yaw,
                                uint64_t goal_id, bool can_continue) {
        const auto previous = lifecycle_.snapshot();
        lifecycle_.activate(goal_id, [&] {
            // A SET without CANCEL may keep the current safe command trajectory
            // alive while ReplanOnce starts from its future P/V/A state. The
            // lifecycle epoch still invalidates any in-flight old-goal plan.
            const bool continuous = can_continue && previous &&
                machine_state_ == FOLLOW_TRAJ &&
                committed_traj_epoch_ == previous->epoch;
            gi_.goal_p = goal_p;
            gi_.goal_yaw = goal_yaw;
            gi_.new_goal = true;
            consecutive_plan_failures_ = 0;
            first_plan_failure_time_ = -1.0;
            traj_finish_ = false;
            started_ = true;
            finish_plan = false;
            plan_from_rest_ = false;
            if (continuous) {
                committed_traj_epoch_ = previous->epoch + 1;
            } else if (machine_state_ == FOLLOW_TRAJ || machine_state_ == EMER_STOP) {
                machine_state_ = WAIT_GOAL;
            }
        });
    }

    bool Fsm::cancelGoal(uint64_t requested_goal_id) {
        const bool canceled = lifecycle_.cancel(requested_goal_id, [&] {
            gi_.new_goal = false;
            consecutive_plan_failures_ = 0;
            first_plan_failure_time_ = -1.0;
            traj_finish_ = false;
            plan_from_rest_ = false;
            finish_plan = true;
            started_ = false;
            machine_state_ = WAIT_GOAL;
        });
        if (canceled) {
            // CANCEL invalidates the reference that the command bridge is
            // braking away from. Once stopped, the next goal must plan from
            // current odometry instead of an abandoned moving endpoint.
            if (planner_ptr_) planner_ptr_->discardCanceledTrajectory();
            resetVisualizedPath();
        }
        return canceled;
    }

    bool Fsm::commitTrajectoryIfCurrent(const GoalLifecycleGuard::Token& token,
                                         const std::function<bool()>& install,
                                         bool installs_trajectory) {
        bool installed = false;
        const bool current = lifecycle_.commitIfCurrent(token, [&] {
            installed = install();
            if (installed && installs_trajectory) {
                committed_traj_epoch_ = token.epoch;
                ++committed_traj_revision_;
            }
        });
        if (!current && ros_ptr_) {
            static std::atomic<int64_t> last_warning_ns{0};
            const auto now = std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count();
            auto last = last_warning_ns.load();
            if (now - last > 1000000000LL && last_warning_ns.compare_exchange_strong(last, now))
                ros_ptr_->warn(" -- [Fsm] Discarded stale trajectory commit.");
        }
        return current && installed;
    }

    bool Fsm::applyReplanResultIfCurrent(const GoalLifecycleGuard::Token& token,
                                         uint64_t stage, uint64_t revision, RET_CODE result,
                                         bool* publish_trajectory) {
        bool accepted = false;
        lifecycle_.runIfCurrent(token, [&] {
            if (machine_state_ != FOLLOW_TRAJ || planning_stage_ != stage) return;
            // A newer trajectory in the same goal epoch also supersedes this result.
            if ((result == NEW_TRAJ || result == EMER) &&
                committed_traj_revision_ != revision) return;
            accepted = true;
            if (result == EMER) ChangeState("ReplanTimerCallback", EMER_STOP);
            else if (result == NEW_TRAJ) ChangeState("ReplanTimerCallback", GENERATE_TRAJ);
            else if (result == SUCCESS || result == FINISH) {
                gi_.new_goal = false;
                if (publish_trajectory)
                    *publish_trajectory = committed_traj_epoch_ == token.epoch;
            }
        });
        return accepted;
    }

    void Fsm::ChangeState(const string &call_func, const MACHINE_STATE &new_state) {
        fmt::print(fg(fmt::color::green), " -- [Fsm]: [{}] change state from [{}] to [{}].\n", call_func,
                   MACHINE_STATE_STR[int(machine_state_)], MACHINE_STATE_STR[int(new_state)]);
        if (machine_state_ != new_state) ++planning_stage_;
        machine_state_ = new_state;
    }
}
