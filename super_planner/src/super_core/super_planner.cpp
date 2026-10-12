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

#include <super_core/super_planner.h>
#include <super_core/flight_z_contract.h>
#include <memory>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <super_utils/scope_timer.hpp>
#include <fmt/color.h>

using namespace super_utils;

namespace super_planner {
    SuperPlanner::SuperPlanner
            (const std::string &cfg_path,
             const ros_interface::RosInterface::Ptr &ros_ptr,
             const rog_map::ROGMapROS::Ptr &map_ptr
            ) : cfg_(Config(cfg_path)), ros_ptr_(ros_ptr), map_ptr_(map_ptr) {

        ros_ptr_->setResolution(cfg_.resolution);
        ros_ptr_->setVisualizationEn(cfg_.visualization_en);
        exp_traj_opt_ = std::make_shared<traj_opt::ExpTrajOpt>(cfg_.exp_traj_cfg, ros_ptr_);
        back_traj_opt_ = std::make_shared<traj_opt::BackupTrajOpt>(cfg_.back_traj_cfg, ros_ptr_);
        yaw_traj_opt_ = std::make_shared<traj_opt::YawTrajOpt>(cfg_.yaw_dot_max);
        const auto &rog_map_cfg = map_ptr_->getMapConfig();
        astar_ptr_ = std::make_shared<path_search::Astar>(cfg_path, ros_ptr_, map_ptr_);
        cg_ptr_ = std::make_shared<CorridorGenerator>(ros_ptr_, map_ptr_, cfg_.corridor_bound_dis,
                                                      cfg_.corridor_line_max_length,
                                                      cfg_.resolution, rog_map_cfg.virtual_ground_height,
                                                      rog_map_cfg.virtual_ceil_height,
                                                      cfg_.robot_r,
                                                      cfg_.obs_skip_num,
                                                      cfg_.iris_iter_num,
                                                      cfg_.flight_min_z,
                                                      cfg_.flight_max_z,
                                                      cfg_.horizontal_clearance_extra,
                                                      cfg_.clearance_vertical_band);
        cg_ptr_->SetLineNeighborList(cfg_.seed_line_neighbour);


        time_consuming_.resize(8);

        robot_state_.rcv = false;
        planner_process_start_WT_ = ros_ptr_->getSimTime();
        fov_checker_ = std::make_shared<FOVChecker>(FOVType::OMNI,
                                                    -1.0,
                                                    -35.0,
                                                    35.0);

        const int neighbor_step = floor(cfg_.robot_r / cfg_.resolution);
        astar_ptr_->setFineInfNeighbors(neighbor_step);
    }

    bool SuperPlanner::trajectoryClearInCurrentMap(const Trajectory& trajectory) const {
        if (trajectory.empty()) return false;
        const double duration = trajectory.getTotalDuration();
        const double now_t = std::max(0.0, ros_ptr_->getSimTime() - trajectory.start_WT);
        const double sample_dt = std::min(0.05, cfg_.resolution /
            std::max(0.1, 2.0 * cfg_.exp_traj_cfg.max_vel));
        vec_E<Vec3f> samples;
        Vec3f box_min = Vec3f::Constant(std::numeric_limits<double>::infinity());
        Vec3f box_max = Vec3f::Constant(-std::numeric_limits<double>::infinity());
        const auto collect = [&](double t) {
            const Vec3f position = trajectory.getPos(t);
            if (!position.allFinite() ||
                map_ptr_->getGridType(position) == GridType::OUT_OF_MAP) return false;
            samples.push_back(position);
            box_min = box_min.cwiseMin(position);
            box_max = box_max.cwiseMax(position);
            return true;
        };
        for (double t = std::min(now_t, duration); t < duration; t += sample_dt)
            if (!collect(t)) return false;
        if (!collect(duration)) return false;

        vec_E<Vec3f> occupied;
        const Vec3f margin = Vec3f::Constant(cfg_.robot_r + cfg_.resolution);
        map_ptr_->boxSearch(box_min - margin, box_max + margin,
                            GridType::OCCUPIED, occupied);
        if (occupied.empty()) return true;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        cloud->reserve(occupied.size());
        for (const auto& p : occupied)
            cloud->push_back(pcl::PointXYZ(p.x(), p.y(), p.z()));
        pcl::KdTreeFLANN<pcl::PointXYZ> tree;
        tree.setInputCloud(cloud);
        std::vector<int> index(1);
        std::vector<float> squared_distance(1);
        const double radius_squared = cfg_.robot_r * cfg_.robot_r;
        for (const auto& p : samples) {
            if (tree.nearestKSearch(pcl::PointXYZ(p.x(), p.y(), p.z()), 1,
                                    index, squared_distance) > 0 &&
                squared_distance[0] < radius_squared - 1e-6) {
                ros_ptr_->warn(" -- [SUPER] Rejecting optimized trajectory near [{}, {}, {}]: raw obstacle clearance {} m < {} m.",
                               p.x(), p.y(), p.z(),
                               std::sqrt(squared_distance[0]), cfg_.robot_r);
                return false;
            }
        }
        return true;
    }

    void SuperPlanner::captureRunSnapshot(PlannerRunSnapshot* snapshot) {
        if (!snapshot) return;
        latest_replan.setSfcPc(cg_ptr_->getLatestCloud());
        latest_replan.setComptT(time_consuming_);
        snapshot->log = latest_replan;
        snapshot->module_time = time_consuming_;
        snapshot->goal_valid = gi_.goal_valid;
        std::fill(time_consuming_.begin(), time_consuming_.end(), 0);
    }

    RET_CODE SuperPlanner::PlanFromRest(const Vec3f &goal_p,
                               const double &goal_yaw,
                               const bool &new_goal,
                               const PlannerCommitGate& commit_gate,
                               PlannerRunSnapshot* snapshot) {
        std::lock_guard<std::mutex> guard(replan_lock_);
        const auto result = planFromRestLocked(goal_p, goal_yaw, new_goal, commit_gate);
        captureRunSnapshot(snapshot);
        return result;
    }

    RET_CODE SuperPlanner::planFromRestLocked(const Vec3f &goal_p,
                               const double &goal_yaw,
                               const bool &new_goal,
                               const PlannerCommitGate& commit_gate) {
        robot_state_ = map_ptr_->getRobotState();
        latest_replan.reset();
        latest_replan.setGoal(goal_p, goal_yaw, robot_state_);
        if (robot_state_.rcv == false) {
            ros_ptr_->warn(" -- [SUPER] in [PlanFromRest]: No odom, force return.");
            latest_replan.setRetCode(SUPER_RET_CODE::SUPER_NO_ODOM);
            return FAILED;
        }
        gi_.goal_p = goal_p;
        gi_.goal_yaw = goal_yaw;
        gi_.new_goal = new_goal;
        gi_.goal_valid = true;
        vec_Vec3f viz_pts{goal_p, robot_state_.p};

        {
            TimeConsuming t_viz("viz goal path", false);
            ros_ptr_->vizGoalPath(viz_pts);
            time_consuming_[VISUALIZATION] += t_viz.stop();
        }


        // Rest-to-rest handovers start at the last certified terminal reference,
        // not at a lagging odometry position. This preserves command P/V/A and
        // yaw across nominal goal completion without any bridge interpolation.
        Vec3f start_reference = robot_state_.p;
        rest_reference_yaw_ = NAN;
        cmd_traj_info_.lock();
        const Trajectory old_pos = cmd_traj_info_.posTraj();
        const Trajectory old_yaw = cmd_traj_info_.yawTraj();
        cmd_traj_info_.unlock();
        if (!old_pos.empty()) {
            const double duration = old_pos.getTotalDuration();
            const double elapsed = ros_ptr_->getSimTime() - old_pos.start_WT;
            const StatePVAJ terminal = old_pos.getState(duration);
            if (elapsed < duration - 0.002 || !terminal.allFinite() ||
                terminal.col(1).norm() > 0.02 || terminal.col(2).norm() > 0.05) {
                ros_ptr_->warn(" -- [SUPER] REST_RESTART_REJECTED: previous reference is not stationary.");
                return FAILED;
            }
            if ((terminal.col(0)-robot_state_.p).norm() > 0.65) {
                ros_ptr_->warn(" -- [SUPER] REST_RESTART_REJECTED: tracking error exceeds 0.65 m.");
                return FAILED;
            }
            start_reference = terminal.col(0);
            if (!old_yaw.empty())
                rest_reference_yaw_ = old_yaw.getState(old_yaw.getTotalDuration())(0,0);
        } else if (robot_state_.v.norm() > 0.25) {
            ros_ptr_->warn(" -- [SUPER] REST_RESTART_REJECTED: canceled trajectory is still braking.");
            return FAILED;
        }
        // Preserve the exact continuous start reference. getNearestCellNot()
        // returns a voxel centre, so comparing that point against an arbitrary
        // metric pose with a 1e-4 m tolerance falsely rejects almost every
        // valid start that is not exactly at a voxel centre. Query the raw
        // occupancy state at the continuous start point instead; later
        // corridor/inflation checks still enforce vehicle clearance.
        Vec3f local_star_pt = start_reference;
        const GridType start_grid = map_ptr_->getGridType(start_reference);
        if (start_grid == GridType::OCCUPIED || start_grid == GridType::OUT_OF_MAP) {
            ros_ptr_->error(
                    " -- [SUPER] in [PlanFromRest] Continuous start point is occupied or out of map.");
            latest_replan.setRetCode(SUPER_RET_CODE::SUPER_NO_START_POINT);
            return FAILED;
        }
        latest_replan.setLocalStartP(local_star_pt);

        /// 2) Generate Exp traj
        ExpTraj exp_traj_info;
        BackupTraj back_traj_info;
        ExpTraj empty_exp_traj_info;
        local_start_p_ = local_star_pt;
        RET_CODE exp_ret_code = generateExpTraj(empty_exp_traj_info, exp_traj_info);
        //GenerateRestToRestExpTraj(local_star_pt, exp_traj_info);
        if (exp_ret_code == FAILED) {
            ros_ptr_->warn(" -- [SUPER] in [PlanFromRest] GenerateExpTrajectory failed with {}.",
                           RET_CODE_STR[exp_ret_code].c_str());
            return FAILED;
        } else {
            ros_ptr_->info(" -- [SUPER] in [PlanFromRest] GenerateExpTrajectory SUCCESS.");
        }

        back_traj_info.setEmpty();
        RET_CODE back_ret_code = generateBackupTrajectory(exp_traj_info, back_traj_info);;

        if (back_ret_code == SUCCESS) {
            if (cfg_.print_log) {
                ros_ptr_->info(" -- [SUPER] in [PlanFromRest] generateBackupTrajectory SUCCESS.");
            }

            CmdTraj prepared;
            if (!prepared.setTrajectory(exp_traj_info, back_traj_info)) return FAILED;
            if (!trajectoryInsideFlightZ(prepared.posTraj(), cfg_.flight_min_z, cfg_.flight_max_z)) {
                ros_ptr_->warn(" -- [SUPER] Rejecting candidate outside task flight z bounds.");
                return FAILED;
            }
            if (!trajectoryClearInCurrentMap(prepared.posTraj())) return FAILED;
            ExpTraj committed_exp = exp_traj_info;
            if (!commitPreparedTrajectory(cmd_traj_info_, prepared, commit_gate, [&] {
                    last_exp_traj_info_ = std::move(committed_exp);
                    robot_on_backup_traj_ = false;
                    gi_.new_goal = false;
                })) return FAILED;

            // For visualization
            {
                TimeConsuming t_viz("viz goal VisualizeCommitTrajectory", false);
                ros_ptr_->vizCommittedTraj(cmd_traj_info_.posTraj(), cmd_traj_info_.getBackupTrajStartTT());
                time_consuming_[VISUALIZATION] += t_viz.stop();
                latest_replan.setRetCode(SUPER_RET_CODE::SUPER_SUCCESS_WITH_BACKUP);
            }

            return SUCCESS;
        } else if (back_ret_code == FINISH || back_ret_code == NO_NEED) {
            if (cfg_.print_log) {
                ros_ptr_->info(" -- [SUPER] in [PlanFromRest] generateBackupTrajectory Finish or NO_NEED.");
            }
            CmdTraj prepared;
            prepared.setTrajectory(exp_traj_info);
            if (!trajectoryInsideFlightZ(prepared.posTraj(), cfg_.flight_min_z, cfg_.flight_max_z)) {
                ros_ptr_->warn(" -- [SUPER] Rejecting candidate outside task flight z bounds.");
                return FAILED;
            }
            if (!trajectoryClearInCurrentMap(prepared.posTraj())) return FAILED;
            ExpTraj committed_exp = exp_traj_info;
            if (!commitPreparedTrajectory(cmd_traj_info_, prepared, commit_gate, [&] {
                    robot_on_backup_traj_ = false;
                    last_exp_traj_info_ = std::move(committed_exp);
                    gi_.new_goal = false;
                })) return FAILED;

            // For visualization
            TimeConsuming t_viz("viz goal VisualizeCommitTrajectory", false);
            {
                ros_ptr_->vizCommittedTraj(cmd_traj_info_.posTraj(), -1);
                time_consuming_[VISUALIZATION] += t_viz.stop();
            }
            latest_replan.setRetCode(SUPER_RET_CODE::SUPER_SUCCESS_NO_BACKUP);
            return SUCCESS;
        }
        ros_ptr_->warn(" -- [SUPER] in [PlanFromRest] generateBackupTrajectory return [{}], force return",
                       RET_CODE_STR[back_ret_code].c_str());
        return FAILED;
    }


    RET_CODE SuperPlanner::ReplanOnce(const Vec3f &goal_p,
                             const double &goal_yaw,
                             const bool &new_goal,
                             const PlannerCommitGate& commit_gate,
                             PlannerRunSnapshot* snapshot) {
        std::lock_guard<std::mutex> guard(replan_lock_);
        const auto result = replanOnceLocked(goal_p, goal_yaw, new_goal, commit_gate);
        captureRunSnapshot(snapshot);
        return result;
    }

    RET_CODE SuperPlanner::replanOnceLocked(const Vec3f &goal_p,
                             const double &goal_yaw,
                             const bool &new_goal,
                             const PlannerCommitGate& commit_gate) {
        TimeConsuming replan_total_t("ReplanOnce", false);
        robot_state_ = map_ptr_->getRobotState();

        gi_.goal_p = goal_p;
        gi_.goal_yaw = goal_yaw;
        gi_.new_goal = new_goal;
        gi_.goal_valid = true;
        latest_replan.reset();
        latest_replan.setGoal(goal_p, goal_yaw, robot_state_);

        vec_Vec3f viz_pts{goal_p, robot_state_.p};

        {
            TimeConsuming t_viz("tviz", false);
            ros_ptr_->vizGoalPath(viz_pts);
            time_consuming_[VISUALIZATION] += t_viz.stop();
        }


        /// 1) Replan EXP traj
        ExpTraj exp_traj_info;
        TimeConsuming t_exp("t_exp", false);
        ExpTraj planning_exp_traj_info = last_exp_traj_info_;
        RET_CODE exp_ret_code = generateExpTraj(planning_exp_traj_info, exp_traj_info);
        time_consuming_[GENERATE_EXP_TRAJ] = t_exp.stop();

        if (exp_ret_code == FAILED) {
            ros_ptr_->warn(" -- [SUPER] in [ReplanOnce]: GenerateExpTrajectory failed, force return");
            return FAILED;
        } else if (exp_ret_code == NEW_TRAJ) {
            if (cfg_.print_log) {
                ros_ptr_->info(" -- [SUPER] in [ReplanOnce]: Last epx traj end, switch to new traj.");
            }
            return NEW_TRAJ;
        } else if (exp_ret_code == EMER) {
            ros_ptr_->warn(" -- [SUPER] in [ReplanOnce]: Replan failed, switch to emer.");
            return EMER;
        } else if (exp_ret_code == SUCCESS) {
            if (cfg_.print_log) {
                ros_ptr_->info(" -- [SUPER] in [ReplanOnce]: Replan a new exp traj success.");
            }
        } else if (exp_ret_code == NO_NEED) {
            if (cfg_.print_log)
                ros_ptr_->info(" -- [SUPER] in [ReplanOnce]: No need to replan a new exp traj, use last one.");
        }

        {
            TimeConsuming t_viz("tviz", false);
            ros_ptr_->vizYawTraj(exp_traj_info.posTraj(), exp_traj_info.yawTraj());
            time_consuming_[VISUALIZATION] += t_viz.stop();
        }


        BackupTraj back_traj_info;
        // 2）生成back轨迹
        TimeConsuming t_back("t_back", false);
        RET_CODE back_ret_code = generateBackupTrajectory(exp_traj_info, back_traj_info);
        time_consuming_[GENERATE_BACK_TRAJ] = t_back.stop();

        {
            ft += time_consuming_[EPX_TRAJ_FRONTEND] + time_consuming_[BACK_TRAJ_FRONTEND];
            ft_cnt++;
            bt += time_consuming_[BACK_TRAJ_OPT] + time_consuming_[EXP_TRAJ_OPT];
            bt_cnt++;
        }

        double replan_dt = replan_total_t.stop();
        if (replan_dt > cfg_.replan_forward_dt * 0.9) {
            ros_ptr_->warn(" -- [SUPER] in [ReplanOnce]: Replan overtime, check parameters, replan dt = {}.", replan_dt);
            return FAILED;
        }

        if (back_ret_code == SUCCESS) {
            CmdTraj prepared;
            if (!prepared.setTrajectory(exp_traj_info, back_traj_info)) return FAILED;
            if (!trajectoryInsideFlightZ(prepared.posTraj(), cfg_.flight_min_z, cfg_.flight_max_z)) {
                ros_ptr_->warn(" -- [SUPER] Rejecting candidate outside task flight z bounds.");
                return FAILED;
            }
            if (!trajectoryClearInCurrentMap(prepared.posTraj())) return FAILED;
            ExpTraj committed_exp = exp_traj_info;
            if (!commitPreparedTrajectory(cmd_traj_info_, prepared, commit_gate, [&] {
                    last_exp_traj_info_ = std::move(committed_exp);
                    robot_on_backup_traj_ = false;
                    gi_.new_goal = false;
                })) return FAILED;

            {
                // For visualization
                TimeConsuming t_viz("tviz", false);
                ros_ptr_->vizCommittedTraj(cmd_traj_info_.posTraj(), cmd_traj_info_.getBackupTrajStartTT());
                time_consuming_[VISUALIZATION] += t_viz.stop();
            }

            latest_replan.setRetCode(SUPER_SUCCESS_WITH_BACKUP);
            if (cfg_.print_log)
                ros_ptr_->info(" -- [SUPER] in [ReplanOnce]: Replan a new back traj success, all replan success.");
            return SUCCESS;
        } else if (back_ret_code == NO_NEED) {
            // 这次生成backup轨迹的点没有意义,
            ExpTraj committed_exp = exp_traj_info;
            const std::function<bool()> install_state = [&] {
                robot_on_backup_traj_ = false;
                last_exp_traj_info_ = std::move(committed_exp);
                gi_.new_goal = false;
                return true;
            };
            if (commit_gate ? !commit_gate(install_state, false) : !install_state()) return FAILED;


            {
                TimeConsuming t_viz("tviz", false);
                ros_ptr_->vizCommittedTraj(cmd_traj_info_.posTraj(), -1);
                time_consuming_[VISUALIZATION] += t_viz.stop();

            }

            if (cfg_.print_log)
                ros_ptr_->info(" -- [SUPER] in [ReplanOnce]: No need back traj success, all replan success.");
            latest_replan.setRetCode(SUPER_SUCCESS_NO_BACKUP);
            return SUCCESS;
        } else if (back_ret_code == FINISH) {
            // Which means the exp traj is all in known free, no need for backup traj
            CmdTraj prepared;
            prepared.setTrajectory(exp_traj_info);
            if (!trajectoryInsideFlightZ(prepared.posTraj(), cfg_.flight_min_z, cfg_.flight_max_z)) {
                ros_ptr_->warn(" -- [SUPER] Rejecting candidate outside task flight z bounds.");
                return FAILED;
            }
            if (!trajectoryClearInCurrentMap(prepared.posTraj())) return FAILED;
            ExpTraj committed_exp = exp_traj_info;
            if (!commitPreparedTrajectory(cmd_traj_info_, prepared, commit_gate, [&] {
                    last_exp_traj_info_ = std::move(committed_exp);
                    robot_on_backup_traj_ = false;
                    gi_.new_goal = false;
                })) return FAILED;

            {
                TimeConsuming t_viz("tviz", false);
                ros_ptr_->vizCommittedTraj(cmd_traj_info_.posTraj(), -1);
                time_consuming_[VISUALIZATION] += t_viz.stop();
            }

            if (cfg_.print_log)
                ros_ptr_->info(" -- [SUPER] in [ReplanOnce]: No need back traj success, all replan success.");
            latest_replan.setRetCode(SUPER_SUCCESS_NO_BACKUP);
            return SUCCESS;
        }
        ros_ptr_->warn(" -- [SUPER] in [ReplanOnce]: generateBackupTrajectory return {}, replan Failed return",
                       RET_CODE_STR[back_ret_code].c_str());
        return FAILED;
    }

    void SuperPlanner::getOneHeartbeatTime(double &start_WT_pos, bool &traj_finish) {
        const auto snapshot = cmd_traj_info_.heartbeatSnapshot();
        double eval_t = (ros_ptr_->getSimTime() - snapshot.start_wall_time);
        traj_finish = false;
        double total_dur = snapshot.total_duration;
        if (eval_t > total_dur) {
            traj_finish = true;
            eval_t = total_dur;
        }
        start_WT_pos = snapshot.start_wall_time;
        if (snapshot.backup_available && eval_t > snapshot.backup_start_time) {
            robot_on_backup_traj_ = true;
        } else {
            robot_on_backup_traj_ = false;
        }
    }

    bool SuperPlanner::canRetargetCurrentTrajectory(double /*min_remaining*/) const {
        const auto snapshot = cmd_traj_info_.heartbeatSnapshot();
        const double elapsed = ros_ptr_->getSimTime() - snapshot.start_wall_time;
        // Even a nearly-ended trajectory stays valid until its certified stop.
        // The FSM then starts the next rest trajectory from that same endpoint.
        return snapshot.total_duration > 0.0 && std::isfinite(elapsed) && elapsed >= 0.0;
    }

    void SuperPlanner::discardCanceledTrajectory() {
        std::lock_guard<std::mutex> guard(replan_lock_);
        CmdTraj empty;
        cmd_traj_info_.installPrepared(empty);
        last_exp_traj_info_ = ExpTraj();
        robot_on_backup_traj_ = false;
        rest_reference_yaw_ = NAN;
    }


    Trajectory SuperPlanner::getCommittedPositionTrajectory() {
        return cmd_traj_info_.posTraj();
    }

    Trajectory SuperPlanner::getCommittedYawTrajectory() {
        return cmd_traj_info_.yawTraj();
    }


    void SuperPlanner::getOneCommandFromTraj(StatePVAJ &pvaj,
                                             double &yaw,
                                             double &yaw_dot,
                                             bool &on_backup_traj,
                                             bool &traj_finish) {
        cmd_traj_info_.lock();
        const double &cur_t = ros_ptr_->getSimTime();
        const double &cmd_start_WT = cmd_traj_info_.getStartWallTime();
//        const bool &backup_avilibale = cmd_traj_info_.backupTrajAvilibale();
//        const double &backup_start_TT = cmd_traj_info_.getBackupTrajStartTT();
        const double &total_dur = cmd_traj_info_.getTotalDuration();

        traj_finish = (cur_t - cmd_start_WT) > total_dur;
        const double &eval_t = traj_finish ? total_dur : (cur_t - cmd_start_WT);

//        bool last_round_robot_on_backup_traj = robot_on_backup_traj_;
        robot_on_backup_traj_ = cmd_traj_info_.isTTOnBackupTraj(eval_t);
        on_backup_traj = robot_on_backup_traj_;

        pvaj = cmd_traj_info_.posTraj().getState(eval_t);


        /// Get Yaw planning
        static double last_yaw = map_ptr_->getRobotState().yaw;

        yaw = cmd_traj_info_.getYaw((eval_t))[0];
        yaw_dot = cmd_traj_info_.getYawRate((eval_t))[0];

        if (isnan(yaw)) {
            yaw = last_yaw;
            yaw_dot = 0;
        } else {
            last_yaw = yaw;
        }
        if (isnan(yaw_dot)) {
            yaw_dot = 0;
        }

//        if (last_round_robot_on_backup_traj != robot_on_backup_traj_) {
//            if (last_round_robot_on_backup_traj) {
//                ros_ptr_->info(" -- [CMD] Emergency Stop End ========================");
//            } else {
//                ros_ptr_->info(" -- [CMD] Emergency Stop Start ========================");
//            }
//        }

//        double cur_yaw = geometry_utils::get_yaw_from_quaternion(robot_state_.q);
        cmd_traj_info_.unlock();
    }


    void SuperPlanner::getModuleTimeConsuming(vector<double> &time) {
        std::lock_guard<std::mutex> guard(replan_lock_);
        time = time_consuming_;
        std::fill(time_consuming_.begin(), time_consuming_.end(), 0);
    }


    RET_CODE SuperPlanner::generateExpTraj(ExpTraj &last_exp_traj_info, ExpTraj &out_exp_traj_info) {
        /* 1) Log the exp traj frontend time*/
        TimeConsuming t_exp_frontend("t_exp_frontend", false);

        // use hot init or not, just prepare a guide path, a guide t, init and fina state and sfc for exp traj opt
        StatePVAJ pos_init_state, pos_fina_state;
        PolytopeVec sfc;
        vec_Vec3f guide_path;
        // the guide_stamp saves a TT
        vector<double> guide_stamp;
        double guide_path_end_vel{0.0};
        int reserve_size = cfg_.planning_horizon / cfg_.resolution * 1.2;
        guide_path.reserve(reserve_size);
        guide_stamp.reserve(reserve_size);

        Vec4f init_yaw{last_exp_traj_info.empty() && std::isfinite(rest_reference_yaw_)
                           ? rest_reference_yaw_ : robot_state_.yaw, 0, 0, 0};
        Vec4f fina_yaw{0, 0, 0, 0};


        // alias for last_exp_traj_info
        Trajectory guide_pos_traj, guide_yaw_traj, last_exp_traj;

        // record the wall time (WT) and the trajectory time (TT) at the start of the replan.
        const double replan_process_start_WT = ros_ptr_->getSimTime();
        double replan_process_start_TT, replan_state_TT;

        /* 2) Check last exp traj */
        if (last_exp_traj_info.empty()) {
            /* 2.1) Perform rest2rest exp traj generation */
            // just skip the first part of the guide trajectory
            pos_init_state.setZero();
            pos_init_state.col(0) = local_start_p_;
            replan_process_start_TT = -1;
            replan_state_TT = -1;
        } else {
            guide_pos_traj = cmd_traj_info_.posTraj(); // last_exp_traj;
            guide_yaw_traj = cmd_traj_info_.yawTraj(); //last_exp_traj_info.exp_yaw_traj;
            last_exp_traj = last_exp_traj_info.posTraj();

            replan_process_start_TT = replan_process_start_WT - last_exp_traj.start_WT;
            replan_state_TT = replan_process_start_TT + cfg_.replan_forward_dt;
            /* 2.2) Perform collision check on last exp traj*/
            vector<TimePosPair> last_exp_traj_time_pos;
            vector<double> last_exp_traj_vel;


            // check early exit condition
            // 1) if the replan state is beyond the last cmd traj, return NO_NEED
            if (replan_state_TT >= cmd_traj_info_.getTotalDuration()) {
                out_exp_traj_info = last_exp_traj_info;

                if (robot_on_backup_traj_) {
                    if (cfg_.print_log)
                        ros_ptr_->warn(
                                " -- [SUPER] Replan, emergency stop, return FAILED and wait for plan form rest.");
                    return FAILED;
                }

                if (cfg_.print_log) {
                    ros_ptr_->warn(
                            " -- [generateExpTraj] replan_state_TT >= cmd_traj_info_.pos_traj.getTotalDuration(), return NONEED and wait for plan form rest.");
                }
                return NO_NEED;
            }

            if (!last_exp_traj_info.empty()) {
                if (replan_state_TT >= last_exp_traj.getTotalDuration()) {
                    out_exp_traj_info = last_exp_traj_info;
                    if (cfg_.print_log)
                        ros_ptr_->warn(
                                " -- [generateExpTraj] replan_state_TT >= last_exp_traj.getTotalDuration(), return NONEED and wait for plan form rest.");
                    if (robot_on_backup_traj_) {
                        if (cfg_.print_log)
                            ros_ptr_->warn(
                                    " -- [SUPER] Replan, emergency stop, return FAILED and wait for plan form rest.");
                        return FAILED;
                    } else {
                        return NO_NEED;
                    }
                }

                /// 1) Check a series of early termination conditions.
                if (!gi_.new_goal && last_exp_traj_info.getSFCSize() == 1 && last_exp_traj_info.connectedToGoal()) {
                    if (cfg_.print_log) {
                        ros_ptr_->warn(
                                " -- [SUPER] Replan, last exp have only one corridor and connected to goal return NONEED.");
                    }

                    out_exp_traj_info = last_exp_traj_info;
                    if (robot_on_backup_traj_) {
                        if (cfg_.print_log)
                            ros_ptr_->warn(
                                    " -- [SUPER] Replan, emergency stop, return FAILED and wait for plan form rest.");
                        return FAILED;
                    } else {
                        return NO_NEED;
                    }
                }

                if (!gi_.new_goal &&
                    (gi_.goal_p - last_exp_traj.getPos(replan_state_TT)).norm() < cfg_.resolution * 3) {
                    // Return if the traj close to goal
                    out_exp_traj_info = last_exp_traj_info;
                    out_exp_traj_info.setGoalConnectedFlag(true);

                    ros_ptr_->warn(" -- [SUPER] Replan, close to goal and return NONEED.");
                    if (robot_on_backup_traj_) {
                        ros_ptr_->warn(
                                " -- [SUPER] Replan, emergency stop, return FAILED and wait for plan form rest.");
                        return FAILED;
                    } else {
                        return NO_NEED;
                    }
                }
            }
            /// Ready for replan.
            out_exp_traj_info.setGoalConnectedFlag(false);

            // * 2) Check if in backup trajectory. While in backup trajectory,
            // *    the guide trajectory should be a part of cmd trajectory.
            // TODO: Why cannot directly replan on cmd traj? 241121

            // * 3) Perform collision check on the guide trajectory.
            // TODO 0929 critical change for hot init.
            double eval_t = replan_state_TT; //replan_process_start_TT;
            double guide_pos_traj_total_time = guide_pos_traj.getTotalDuration();

            Vec3f temp_pt, last_sample_pt;
            last_exp_traj_time_pos.clear();
            last_exp_traj_info.setWholeTrajKnownFreeFlag(true);
            last_sample_pt = guide_pos_traj.getPos(eval_t);
            eval_t += cfg_.sample_traj_dt;
            // * 4) 记录replan点在evaluated_pts上的id
            int replan_id = -1;
            for (; eval_t < guide_pos_traj_total_time; eval_t += cfg_.sample_traj_dt) {
                temp_pt = guide_pos_traj.getPos(eval_t);
                if ((temp_pt - last_sample_pt).norm() < cfg_.resolution * 0.8) {
                    continue;
                }

                rog_map::GridType temp_grid = map_ptr_->getInfGridType(temp_pt);

                if (temp_grid == rog_map::GridType::OCCUPIED || temp_grid == rog_map::GridType::OUT_OF_MAP) {
                    last_exp_traj_info.setWholeTrajKnownFreeFlag(false);
                    break;
                }
                if (eval_t > replan_state_TT && replan_id == -1) {
                    replan_id = last_exp_traj_time_pos.size();
                }
                last_exp_traj_time_pos.emplace_back(eval_t, temp_pt);
                last_exp_traj_vel.emplace_back(guide_pos_traj.getVel(eval_t).norm());
                last_sample_pt = temp_pt;
            }


            // * 6) Decide where to split the original exp trajecory and re-plan a new one with an A*,
            // *    If the whole trajectory if free,  the whole trajectory should be receding and if not, or a new goal
            // *    is given, we should only receiding a small distance and replan new trajectory ASAP
            double split_dis = cfg_.receding_dis;
            if (last_exp_traj_info.wholeTrajKnownFree() && !gi_.new_goal && cfg_.receding_dis > 0.0) {
                split_dis = std::numeric_limits<double>::max();
            }


            // * 7）Begin replan process, first get the replan state from the committed trajectory.
            if (!guide_pos_traj.getState(replan_state_TT, pos_init_state)) {
                ros_ptr_->warn(" -- [SUPER] Invalid traj or eval t");
                return FAILED;
            }
            // * Generate guide path with time stampe, for hot trajectory initialization
            // * the guide stamp is time from the replan start t
            guide_stamp.clear();
            guide_path.clear();
            if (split_dis <= 0 || last_exp_traj_time_pos.empty()) {
                /// No need receding, just path search.
                guide_path.push_back(pos_init_state.col(0));
                guide_stamp.push_back(0.0);
                last_exp_traj_time_pos.clear();
                last_exp_traj_time_pos.emplace_back(replan_state_TT, pos_init_state.col(0));
                guide_path_end_vel = robot_state_.v.norm();
            } else {
                temp_pt = last_exp_traj_time_pos.back().second;
                // * 8) Pop all evaluated pts after the sampled point.
                while (map_ptr_->isOccupiedInflate(temp_pt) ||
                       (temp_pt - pos_init_state.col(0)).norm() > split_dis) {
                    last_exp_traj_time_pos.pop_back();
                    last_exp_traj_vel.pop_back();
                    if (last_exp_traj_time_pos.empty()) {
                        ros_ptr_->warn(" -- [SUPER] WARN, all traj is collide in INF2");
                        break;
                    }
                    temp_pt = last_exp_traj_time_pos.back().second;
                }
                if (!last_exp_traj_time_pos.empty()) {
                    for (long unsigned int i = 0; i < last_exp_traj_time_pos.size(); i++) {
                        guide_path.push_back(last_exp_traj_time_pos[i].second);
                        guide_stamp.push_back(last_exp_traj_time_pos[i].first - last_exp_traj_time_pos.front().first);
                        guide_path_end_vel = last_exp_traj_vel[i];
                    }
                } else {
                    guide_path.push_back(pos_init_state.col(0));
                    guide_stamp.push_back(0.0);
                    last_exp_traj_time_pos.emplace_back(replan_state_TT, pos_init_state.col(0));
                    guide_path_end_vel = robot_state_.v.norm();
                }
            }
        }

        // second, geometry part of the guide path
        ///=================The Second Part of Guide Path ================================================

        double guide_path_length = geometry_utils::computePathLength(guide_path);
        double temp_horizon = cfg_.planning_horizon - guide_path_length;

        vector<int> path_passed_waypoint_id;
        vec_Vec3f inside_poly_goals;
        vector<int> sfc_waypoint_ids;

        if (guide_path.empty() ||
            ((guide_path.front() - pos_init_state.col(0)).norm() > 1e-2)) {
            guide_path.insert(guide_path.begin(), pos_init_state.col(0));
            guide_stamp.insert(guide_stamp.begin(), 0.0);
        }

        // if need a geometry path
        if (temp_horizon > cfg_.resolution * 2) {
            /// start point TT + exp_traj start_WT
//            double path_search_start_point_WT = guide_stamp.back() + guide_pos_traj.start_WT;
            // if the goal is close to the last point of the guide path, just add the goal to the guide path
            if ((guide_path.back() - gi_.goal_p).norm() < cfg_.resolution * 5) {
                guide_stamp.push_back(guide_stamp.back() +
                                      (guide_path.back() - gi_.goal_p).norm() / cfg_.exp_traj_cfg.max_vel);
                guide_path.push_back(gi_.goal_p);
                // NO NEED
            } else {
                vec_Vec3f new_path;
                // project goal within the planning horizon
//                const Vec3f dir = (gi_.goal_p - robot_state_.p).normalized();
//                const double dis2goal = (gi_.goal_p - robot_state_.p).norm();
//                Vec3f cadi_p = gi_.goal_p;
//                if(dis2goal > cfg_.planning_horizon) {
//                    double proj_l = cfg_.planning_horizon;
//                    Vec3f cadi_p = robot_state_.p + dir * proj_l;
//                    int max_iter = 100;
//                    while(map_ptr_->isOccupiedInflate(cadi_p) && max_iter-- > 0) {
//                        if(map_ptr_->getNearestInfCellNot(OCCUPIED, cadi_p, cadi_p, 1.0)) {
//                            break;
//                        }
//                        proj_l -= 2.0;
//                        if(proj_l < 1){
//                            ros_ptr_->warn(" -- [SUPER] Project goal failed");
//                            gi_.goal_valid = false;
//                            return FAILED;
//                        }
//                        cadi_p = robot_state_.p + dir * proj_l;
//                    }
//                    if(max_iter <= 0) {
//                        ros_ptr_->warn(" -- [SUPER] Project goal failed");
//                        gi_.goal_valid = false;
//                        return FAILED;
//                    }
//                }
                if (!PathSearch(guide_path.back(), gi_.goal_p, temp_horizon, new_path)) {
                    ros_ptr_->warn(" -- [SUPER] PathSearch for new path failed");
                    return FAILED;
                }
                if (new_path.size() < 2) {
                    ros_ptr_->warn(" -- [SUPER] PathSearch for new path failed");
                    return FAILED;
                }

                // compute total dis
                // backward compute dis for all points
                double total_dis{0.0};
                vector<double> dis(new_path.size());
                Vec3f last_p = new_path.back();
                for (int i = new_path.size() - 2; i >= 0; i--) {
                    auto d = (new_path[i] - last_p).norm();
                    total_dis += d;
                    dis[i+1] = total_dis;
                    last_p = new_path[i];
                }
                total_dis += (new_path.front() - guide_path.back()).norm();
                dis[0] = total_dis;
//                for (int i = 0; i < dis.size(); i++) {
//                    cout << dis[i] << " ";
//                }
//                cout << endl;
                vector<double> stamps(new_path.size(), 0);
                vector<double> dt(new_path.size(), 0);
                double last_stamp = 0;
                for (int i = dis.size() - 1; i >= 0; i--) {
                    double vel;
                    geometry_utils::simplePMTimeAllocator(cfg_.exp_traj_cfg.max_acc, cfg_.exp_traj_cfg.max_vel,
                                                          guide_path_end_vel,
                                                          total_dis,
                                                          dis[i], stamps[i], vel);
                    dt[i] = stamps[i] - last_stamp;
                    last_stamp = stamps[i];
                }
                double time_stamp = guide_stamp.back();

//                for (int i = 0; i < stamps.size(); i++) {
//                    cout << stamps[i] << " ";
//                }
//                cout << endl;
//
//                for (int i = 0; i < dt.size(); i++) {
//                    cout << dt[i] << " ";
//                }
//                cout << endl;

                for (long unsigned int i = 1; i < new_path.size(); i++) {
                    double t = dt[i];
                    time_stamp += t;
                    guide_path.emplace_back(new_path[i]);
                    guide_stamp.emplace_back(time_stamp);
                }
            }
        }

        const bool connected_goal = (guide_path.back().head(2) - gi_.goal_p.head(2)).norm() < cfg_.resolution * 2;
        out_exp_traj_info.setGoalConnectedFlag(connected_goal);

        sfc.clear();
        {
            TimeConsuming t_viz("tviz", false);
            ros_ptr_->vizFrontendPath(guide_path);
            time_consuming_[VISUALIZATION] += t_viz.stop();
        }
        shifted_sfc_start_pt_ = Vec3f(9999,9999,9999);
        bool bool_ret_code = cg_ptr_->SearchPolytopeOnPath(guide_path, sfc, shifted_sfc_start_pt_, cfg_.use_fov_cut);

        if (!bool_ret_code) {
            ros_ptr_->warn(" -- [SUPER] SearchPolytopeOnPath for new path failed");
            return FAILED;
        }
        {
            TimeConsuming t_viz("tviz", false);
            ros_ptr_->vizExpSfc(sfc);
            time_consuming_[VISUALIZATION] += t_viz.stop();
        }

        time_consuming_[EPX_TRAJ_FRONTEND] = t_exp_frontend.stop();


        pos_fina_state.setZero();
        pos_fina_state.col(0) = guide_path.back();
        if (cfg_.goal_vel_en && (gi_.goal_p - robot_state_.p).norm() > cfg_.planning_horizon / 2) {
            pos_fina_state.col(1) = (gi_.goal_p - robot_state_.p).normalized() * cfg_.exp_traj_cfg.max_vel / 2;
        }
        if ((pos_fina_state.col(0) - gi_.goal_p).norm() < cfg_.resolution * 2) {
            pos_fina_state.col(1).setZero();
            pos_fina_state.col(0) = gi_.goal_p;
        }

        // optimize and update exp traj
        bool temp_ret;
        Trajectory out_traj;
        TimeConsuming t_exp_opt("t_exp_opt", false);
        auto original_sfc = sfc;
        temp_ret = exp_traj_opt_->optimize(pos_init_state,
                                           pos_fina_state,
                                           guide_path,
                                           guide_stamp,
                                           sfc,
                                           out_traj);
        time_consuming_[EXP_TRAJ_OPT] = t_exp_opt.stop();
        {
            VecDf init_ts;
            vec_Vec3f init_ps;
            exp_traj_opt_->getInitValue(init_ts, init_ps);
            latest_replan.setExpCondition(init_ts, init_ps, pos_init_state, pos_fina_state, sfc);
        }
        if (!temp_ret) {
            ros_ptr_->warn(" -- [SUPER] OptimizationExpTrajInPolytopes for new path failed");
            return FAILED;
        }
        double replan_total_t = (ros_ptr_->getSimTime() - replan_process_start_WT);
        if (replan_total_t > cfg_.replan_forward_dt) {
            ros_ptr_->warn(" -- [SUPER] Replan over time({})!!!! Return FAILED", replan_total_t);
            return FAILED;
        }

        {
            TimeConsuming t_viz("tviz", false);
            ros_ptr_->vizExpTraj(out_traj);
            time_consuming_[VISUALIZATION] += t_viz.stop();
        }

        double new_traj_WT = replan_process_start_WT;

        replan_process_start_TT = replan_process_start_WT - guide_pos_traj.start_WT;
        Trajectory temp_exp_traj;
        if (!last_exp_traj_info.empty() &&
            !guide_pos_traj.getPartialTrajectoryByTime(replan_process_start_TT, replan_state_TT,
                                                       temp_exp_traj)) {
            ros_ptr_->error(" -- [SUPER] in [generateExpTraj]: getPartialTrajectoryByTime failed, force return");
            return FAILED;
        }
        out_exp_traj_info.setSFC(sfc);
        temp_exp_traj = temp_exp_traj + out_traj;
        temp_exp_traj.start_WT = new_traj_WT; //last_exp_traj_info.replan_start_WT ;

        if (!last_exp_traj_info.empty()) {
            StatePVAJ yaw_replan_state;
            if (!guide_yaw_traj.getState(replan_state_TT, yaw_replan_state)) {
                ros_ptr_->warn(" -- [SUPER] Invalid traj or eval t");
                return FAILED;
            }
            init_yaw = yaw_replan_state.row(0);
        }


        bool free_end{true};
        if (cfg_.goal_yaw_en && !isnan(gi_.goal_yaw) && connected_goal) {
            free_end = false;
            fina_yaw[0] = gi_.goal_yaw;
        }
        Trajectory new_traj, old_traj;

        if (!yaw_traj_opt_->optimize(init_yaw, fina_yaw, out_traj, new_traj, 3, false, free_end)) {
            ros_ptr_->error(" -- [SUPER] in [generateExpTraj]: YawTrajOpt failed, force return");
            return FAILED;
        }
        if (!last_exp_traj_info.empty()) {
            if (!guide_yaw_traj.getPartialTrajectoryByTime(replan_process_start_TT, replan_state_TT,
                                                           old_traj)) {
                ros_ptr_->error(" -- [SUPER] in [generateExpTraj]: getPartialTrajectoryByTime failed, force return");
                return FAILED;
            }
        }

        const auto temp_yaw_traj = old_traj + new_traj;
        // check if part of the exp on last backup
        double on_backup_end_TT{-1}, on_backup_start_TT{-1};
        if (!last_exp_traj_info.empty() && replan_state_TT > cmd_traj_info_.getBackupTrajStartTT()) {
            on_backup_start_TT = cmd_traj_info_.getBackupTrajStartTT() - replan_process_start_TT;
            on_backup_end_TT = replan_state_TT - replan_process_start_TT;
        }
        out_exp_traj_info.setTrajectory(new_traj_WT, temp_exp_traj, temp_yaw_traj, on_backup_start_TT,
                                        on_backup_end_TT);

        latest_replan.setExpYawTraj(temp_yaw_traj);
        latest_replan.setExpTraj(temp_exp_traj);

        return SUCCESS;
    }

    RET_CODE SuperPlanner::generateBackupTrajectory(ExpTraj &ref_exp_traj, BackupTraj &back_traj_info) {
        drone_state_mutex_.lock();
        back_traj_info.setRobotPos(robot_state_.p);
        drone_state_mutex_.unlock();
        TimeConsuming t_back_frontend("t_back_frontend", false);
        double total_dur = ref_exp_traj.getTotalDuration();
        double start_t = ros_ptr_->getSimTime() - ref_exp_traj.getStartWallTime();


        if (start_t > total_dur - 0.01) {
            if (cfg_.print_log) {
                ros_ptr_->info(" -- [SUPER] in [generateBackupTrajectory]: start_t > total_dur, return NO_NEED");
            }
            return NO_NEED;
        }

        Vec3f temp_point;
        double out_t;
        bool all_traj_visible{true};
        // 同时记录每一个点的刹车时间和刹车距离
        vector<double> min_stop_dis;
        vector<TimePosPair> eval_ps;
        Vec3f temp_vel;

        // 记录当前时刻到最远时刻的所有可视部分
        Vec3f last_pos = ref_exp_traj.getPos(start_t);
        for (out_t = start_t; out_t < total_dur; out_t += cfg_.sample_traj_dt) {
            temp_point = ref_exp_traj.getPos(out_t);
            if ((last_pos - temp_point).norm() < cfg_.resolution * 0.8) {
                continue;
            }
            last_pos = temp_point;
            temp_vel = ref_exp_traj.getVel(out_t);
            // Compute initial
            double v_norm = temp_vel.norm();
            min_stop_dis.push_back(v_norm * v_norm / 2.0 / cfg_.exp_traj_cfg.max_acc);
            eval_ps.push_back(std::pair<double, Vec3f>(out_t, temp_point));
            const double min_dis =
                    cfg_.sensing_horizon > 0 ? std::min(cfg_.sensing_horizon, cfg_.safe_corridor_line_max_length)
                                             : cfg_.safe_corridor_line_max_length;
            if (!map_ptr_->isLineFree(back_traj_info.getRobotPos(),
                                      temp_point,
                                      min_dis,
                                      cfg_.seed_line_neighbour)) {
                all_traj_visible = false;
                break;
            }
        }

        if (all_traj_visible) {
            back_traj_info.setEmpty();
            {
                double dur = ref_exp_traj.getTotalDuration();
                Vec3f seed_pt = ref_exp_traj.getPos(dur);
                Line line{back_traj_info.getRobotPos(), seed_pt};
                Polytope temp_poly;
                if (cg_ptr_->GeneratePolytopeFromLine(line, temp_poly)) {
                    back_traj_info.setSFC(temp_poly);
                    {
                        TimeConsuming t_viz("tviz", false);
                        ros_ptr_->vizBackupSfc(temp_poly);
                        time_consuming_[VISUALIZATION] += t_viz.stop();
                    }
                }
            }
            return FINISH;
        }
        Vec3f invisible_p = eval_ps.back().second;
        while (out_t > start_t) {
            out_t -= cfg_.sample_traj_dt;
            Vec3f out_p = ref_exp_traj.getPos(out_t);
            if ((out_p - invisible_p).norm() > cfg_.robot_r) {
                break;
            }
        }

        double seed_point_t = std::max(start_t, out_t);

        // TODO check this logic, comment on Dec. 13
        // if
        // 1) last exp traj has a backup traj
        // 2) last backup WT is larger than this term
        // 3) last exp is collision free
        // if (ref_exp_traj.back_traj_start_TT > 0 &&
        // seed_point_t < ref_exp_traj.back_traj_start_TT) {
        // return NO_NEED;
        // }


        Vec3f seed_point = ref_exp_traj.getPos(seed_point_t);

        Vec3f shifted_robot_p = shifted_sfc_start_pt_.norm()> 999?robot_state_.p:shifted_sfc_start_pt_;
        if (!map_ptr_->getNearestCellNot(GridType::OCCUPIED, shifted_robot_p, shifted_robot_p, 3.0)) {
            ros_ptr_->error(
                    " -- [SUPER] in [PlanFromRest] Local start point is deeply occupied, which should not happened.");
            latest_replan.setRetCode(SUPER_RET_CODE::SUPER_NO_START_POINT);
            return FAILED;
        }

        Line line{shifted_robot_p, seed_point};
        Polytope temp_poly;
        if (!cg_ptr_->GeneratePolytopeFromLine(line, temp_poly)) {
            ros_ptr_->warn(" -- [SUPER] GeneratePolytopeFromLine failed, force return");
            return FAILED;
        }
        Eigen::Vector3d inner;
        Eigen::Matrix3Xd vPoly;
        if (!geometry_utils::findInterior(temp_poly.GetPlanes(), inner)) {
            ros_ptr_->warn(" -- [SUPER] Cannot generate feasible backup sfc, force return");
            vec_Vec3f seed{back_traj_info.getRobotPos(), seed_point};
            return FAILED;
        }

        if (cfg_.use_fov_cut) {
            if (!fov_checker_->cutPolyByFov(robot_state_.p, robot_state_.q, seed_point,
                                            temp_poly)) {
                ros_ptr_->warn(" -- [SUPER] cutPolyByFov failed, force return");
                return FAILED;
            }
        }
        // cut by sensing horizon
        if (cfg_.sensing_horizon > 0 &&
            !fov_checker_->cutPolyBySensingHorizon(robot_state_.p, seed_point, cfg_.sensing_horizon,
                                                   temp_poly)) {
            ros_ptr_->warn(" -- [SUPER] cutPolyBySensingHorizon failed, force return");
            vec_Vec3f seed{back_traj_info.getRobotPos(), seed_point};
            return FAILED;
        }

        back_traj_info.setSFC(temp_poly);

        {
            TimeConsuming t_viz("tviz", false);
            ros_ptr_->vizBackupSfc(temp_poly);
            time_consuming_[VISUALIZATION] += t_viz.stop();
        }

//        Vec3f out_p = temp_point;
//        double t_R = 0.0;
        double eval_t = eval_ps.back().first + cfg_.sample_traj_dt;
        last_pos = eval_ps.back().second;
        while (temp_poly.PointIsInside(eval_ps.back().second) && eval_t < total_dur) {
            Vec3f cur_pos = ref_exp_traj.getPos(eval_t);

            if ((cur_pos - last_pos).norm() < cfg_.resolution * 0.8) {
                eval_t += cfg_.sample_traj_dt;
                continue;
            }
            temp_vel = ref_exp_traj.getVel(out_t);
            double v_norm = temp_vel.norm();
            min_stop_dis.push_back(v_norm * v_norm / 2.0 / cfg_.exp_traj_cfg.max_acc);
            eval_ps.emplace_back(eval_t, cur_pos);
            last_pos = cur_pos;
            eval_t += cfg_.sample_traj_dt;
        }
        eval_ps.pop_back();
        seed_point = eval_ps.back().second;
        seed_point_t = eval_ps.back().first;

        //        bool use_new{true};
        //        if (use_new) {
        double t0 = ros_ptr_->getSimTime() -
                    ref_exp_traj.getStartWallTime() + 0.01;
        double te = seed_point_t;
        //            cout << "t0: " << t0 << endl;
        //            cout << "te: " << te << endl;
        //            cout << "exp_traj_dur: " << ref_exp_traj.optimized_exp_traj.getTotalDuration() << endl;
        double vel_e_n = ref_exp_traj.getVel(te).norm();
        double heu_ts = std::max((t0 + te) / 2, te - vel_e_n / cfg_.back_traj_cfg.max_acc);
        double heu_dur = te - heu_ts;
        // MINCO maps each piece duration back to an unconstrained variable.
        // A near-zero or negative stopping interval makes that inverse map
        // singular (or NaN), so it cannot represent a backup trajectory.
        // Keep the currently committed trajectory and try the next replan.
        if (!std::isfinite(t0) || !std::isfinite(te) ||
            !std::isfinite(heu_ts) || !std::isfinite(heu_dur) ||
            te - t0 < 0.1 || heu_dur < 0.1) {
            ros_ptr_->warn(
                " -- [SUPER] Skipping degenerate backup interval: t0={} te={} ts={} duration={} speed={}",
                t0, te, heu_ts, heu_dur, vel_e_n);
            return OPT_FAILED;
        }
        Vec3f heu_p = seed_point;
        time_consuming_[BACK_TRAJ_FRONTEND] = t_back_frontend.stop();
        TimeConsuming t_back_opt("t_back_opt", false);
        double opt_ts = heu_ts;
        Trajectory temp_pos_traj;
        bool temp_ret = back_traj_opt_->optimize(ref_exp_traj.posTraj(),
                                                 t0,
                                                 te,
                                                 heu_ts,
                                                 heu_p,
                                                 heu_dur,
                                                 back_traj_info.getSFC(),
                                                 temp_pos_traj,
                                                 opt_ts);
        time_consuming_[BACK_TRAJ_OPT] = t_back_opt.stop();

        {
            double init_ts;
            VecDf init_times;
            vec_Vec3f init_ps;
            back_traj_opt_->getInitValue(init_ts, init_times, init_ps);
            latest_replan.setBackupCondition(init_ts, init_times, init_ps,
                                             t0, te,
                                             back_traj_info.getSFC());
        }

        if (!temp_ret) {
            ros_ptr_->warn(" -- [SUPER] OptimizationBakTrajInPolytopes failed, force return");
            back_traj_info.setEmpty();
            return OPT_FAILED;
        } else {
            Vec4f yaw_init_vec = ref_exp_traj.getYawState(opt_ts).row(0);
            Vec4f yaw_goal{0, 0, 0, 0};
            bool free_end{true};
            if (cfg_.goal_yaw_en) {
                if (!isnan(gi_.goal_yaw)) {
                    free_end = false;
                    yaw_goal[0] = gi_.goal_yaw;
                }
            }
            Trajectory temp_yaw_traj;
            if (!yaw_traj_opt_->optimize(yaw_init_vec, yaw_goal, temp_pos_traj,
                                         temp_yaw_traj, 3, false, free_end)) {
                ros_ptr_->error(" -- [SUPER] in [generateBackupTrajectory] YawTrajOpt FAILD.");
                return OPT_FAILED;
            }


            if (opt_ts < t0) {
                ros_ptr_->error(" -- [SUPER] opt_ts {} < t0 {}", opt_ts, t0);
                return OPT_FAILED;
            }
            double new_ts_WT = ref_exp_traj.getStartWallTime() + opt_ts;
            const auto &committed_ts_WT = cmd_traj_info_.getBackupTrajStartTT();
            if (committed_ts_WT < cmd_traj_info_.getTotalDuration() && new_ts_WT < committed_ts_WT) {
                ros_ptr_->error(" -- [SUPER] new_ts_WT {} < committed_ts_WT {}", new_ts_WT, committed_ts_WT);
                return OPT_FAILED;
            }


            {
                TimeConsuming t_viz("tviz", false);
                ros_ptr_->vizBackupTraj(temp_pos_traj);
                time_consuming_[VISUALIZATION] += t_viz.stop();
            }

            back_traj_info.setTrajectory(new_ts_WT, opt_ts, temp_pos_traj, temp_yaw_traj);
            latest_replan.setBackupTraj(temp_pos_traj);
            latest_replan.setBackupYawTraj(temp_yaw_traj);
            return SUCCESS;
        }
        ros_ptr_->warn(" -- [SUPER] Cannot find backup traj start point.");
        return FAILED;
    }

    int SuperPlanner::getNearestFurtherGoalPoint(const vec_E<Vec3f> &goals, const Vec3f &start_pt) {
        if (goals.size() == 1) {
            return 0;
        }
        Vec3f a = start_pt, b;
        int min_id = 0;
        double min_dis = 1e10;
        for (long unsigned int i = 0; i < goals.size() - 1; i++) {
            b = goals[i];
            double dis = geometry_utils::pointLineSegmentDistance(start_pt, a, b);
            if (dis < min_dis) {
                min_dis = dis;
                min_id = i;
            }
            a = b;
        }
        return min_id;
    }

    bool
    SuperPlanner::PathSearch(const Vec3f &start_pt, const Vec3f &goal,
                             const double &searching_horizon,
                             vec_Vec3f &path) {
        using namespace path_search;
        if (searching_horizon <= 0.0) {
            ros_ptr_->error(" -- [SUPER] Goal waypoints empty or searching horizon negative, force return.");
            return false;
        }

        // 1) check and shift pts
        // 		For start point, must be collision free
        rog_map::GridType start_type;
        start_type = map_ptr_->getGridType(start_pt);

        /// If the start_pt is obstacle in prob map, just shift it to the nearest free point.
        if (start_type == rog_map::GridType::OCCUPIED ||
            start_type == rog_map::GridType::OUT_OF_MAP) {
            ros_ptr_->warn(
                    " -- [SUPER] The start point in obstacle, this should not happen since the start point should be shift before pathsearch.");
            return false;
        }
        vec_E<Vec3f> start_point_escape_path;

        int flag_es = ON_PROB_MAP | (cfg_.frontend_in_known_free ? UNKNOWN_AS_OCCUPIED : UNKNOWN_AS_FREE);
        vec_Vec3f out_path;
        RET_CODE ret_es = astar_ptr_->escapePathSearch(start_pt, flag_es, out_path);
        if (ret_es != NO_NEED) {
            if (ret_es != REACH_HORIZON && ret_es != REACH_GOAL) {
                ros_ptr_->error(
                        " -- [SUPER] Escape path search failed with [{}], force return.",
                        RET_CODE_STR[ret_es].c_str());
                return false;
            } else {
                start_point_escape_path = out_path;
            }
        }

        Vec3f shifted_start_pt = start_pt;

        if (!start_point_escape_path.empty()) {
            shifted_start_pt = start_point_escape_path.back();
        }

        Vec3f temp_goal_point, temp_start_point;
        temp_start_point = shifted_start_pt;
        double temp_plannning_horizon = searching_horizon;
        //            int start_id = getNearestFurtherGoalPoint(goal_waypoints, start_pt);

        int flag = ON_INF_MAP | (cfg_.frontend_in_known_free ? UNKNOWN_AS_OCCUPIED : UNKNOWN_AS_FREE) | DONT_USE_INF_NEIGHBOR;

        RET_CODE ret_code = astar_ptr_->pointToPointPathSearch(temp_start_point, goal, flag, temp_plannning_horizon,
                                                               path, cfg_.astar_search_timeout);

        if(ret_code == INIT_ERROR){
            gi_.goal_valid = false;
            return false;
        }
        //add may23, if failed on inf map, use prob map try again

        if (ret_code == NO_PATH) {
            flag = ON_PROB_MAP | (cfg_.frontend_in_known_free ? UNKNOWN_AS_OCCUPIED : UNKNOWN_AS_FREE) |
                   USE_INF_NEIGHBOR;
            fmt::print(fg(fmt::color::indian_red) | fmt::emphasis::bold,
                       " -- [Astar] Path search failed on inf map, try again on prob map.\n");
            ret_code = astar_ptr_->pointToPointPathSearch(temp_start_point, goal, flag, temp_plannning_horizon,
                                                          path, cfg_.astar_search_timeout);
            if (ret_code == SUCCESS || ret_code == REACH_HORIZON || ret_code == REACH_GOAL) {
                fmt::print(fg(fmt::color::lime_green) | fmt::emphasis::bold,
                           " -- [Astar] Path search on prob map success.\n");
            } else {
                fmt::print(fg(fmt::color::indian_red) | fmt::emphasis::bold,
                           " -- [Astar] Path search failed on prob map still failed.\n");
            }
        }
        if (ret_code != REACH_HORIZON && ret_code != REACH_GOAL) {
            ros_ptr_->error(
                    " -- [SUPER] Path search failed with [{}], force return.\n", RET_CODE_STR[ret_code].c_str());
            return false;
        }
        if (!start_point_escape_path.empty()) {
            path.insert(path.begin(), start_point_escape_path.begin(),
                        start_point_escape_path.end());
        }

        if (path.empty()) {
            ros_ptr_->warn(
                    " -- [SUPER] Path search failed with empty segments, force return.");
            return false;
        }
        path.insert(path.begin(), start_pt);
        if (ret_code == REACH_GOAL) {
            path.push_back(goal);
        }
        return true;
    }


    void SuperPlanner::getRobotState(rog_map::RobotState &out) {
        out = map_ptr_->getRobotState();
    }
}
