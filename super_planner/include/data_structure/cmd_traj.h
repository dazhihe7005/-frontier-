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


#ifndef CMD_TRAJ_H
#define CMD_TRAJ_H

#include <data_structure/exp_traj.h>
#include <data_structure/backup_traj.h>
#include <data_structure/base/trajectory.h>
#include <utility>


namespace super_planner {
    using geometry_utils::Trajectory;

#define LOCK_G std::lock_guard<std::mutex> lock(mtx_);

    class CmdTraj{
        /* The update and query lock */
        mutable std::mutex mtx_;

        /* The optimized positional trajectory */
        Trajectory pos_traj_{};

        /* The optimized yaw trajectory */
        Trajectory yaw_traj_{};

        double start_WT_{0.0};
        double backup_traj_start_TT_{0.0};
        /* some part of exp traj may belong to last backup, record this */
        double on_backup_start_TT_{-1}, on_backup_end_TT_{-1};
        bool first_part_exp_has_backup_traj_{false};


        /* some flags */
        bool flag_empty_{true};
        bool flag_backup_traj_avilibale_{false};

        void checkFirstPartBackupTraj(const ExpTraj & exp) {
            double tmp_s, tmp_e;
            if(exp.getFirstPartBackupTraj(tmp_s, tmp_e)) {
                on_backup_end_TT_ = tmp_e;
                on_backup_start_TT_ = tmp_s;
                first_part_exp_has_backup_traj_ = true;
            }else {
                first_part_exp_has_backup_traj_ =false;
            }
        }

    public:
        struct HeartbeatSnapshot {
            double start_wall_time;
            double total_duration;
            double backup_start_time;
            bool backup_available;
        };

        explicit  CmdTraj() = default;

        HeartbeatSnapshot heartbeatSnapshot() const {
            std::lock_guard<std::mutex> lock(mtx_);
            return {pos_traj_.start_WT, pos_traj_.getTotalDuration(),
                    backup_traj_start_TT_, flag_backup_traj_avilibale_};
        }

        // The expensive setTrajectory work is performed on a private CmdTraj.
        // Installation exchanges only prepared fields while holding the shared
        // trajectory lock. Callers acquire lifecycle before this lock.
        void installPrepared(CmdTraj &prepared) {
            LOCK_G
            using std::swap;
            swap(pos_traj_, prepared.pos_traj_);
            swap(yaw_traj_, prepared.yaw_traj_);
            swap(start_WT_, prepared.start_WT_);
            swap(backup_traj_start_TT_, prepared.backup_traj_start_TT_);
            swap(on_backup_start_TT_, prepared.on_backup_start_TT_);
            swap(on_backup_end_TT_, prepared.on_backup_end_TT_);
            swap(first_part_exp_has_backup_traj_, prepared.first_part_exp_has_backup_traj_);
            swap(flag_empty_, prepared.flag_empty_);
            swap(flag_backup_traj_avilibale_, prepared.flag_backup_traj_avilibale_);
        }

        void setEmpty() {
            flag_empty_ = true;
        }

        bool empty() const {
            return flag_empty_;
        }

        void lock() {
            mtx_.lock();
        }

        void unlock() {
            mtx_.unlock();
        }


        bool setTrajectory(const ExpTraj&exp_traj,
            const BackupTraj & backup_traj) {
            LOCK_G
            Trajectory tmp_pos_traj, tmp_yaw_traj;
            backup_traj_start_TT_ = backup_traj.getStartTT();
            if (!exp_traj.getPartialTrajectoryByTrajectoryTime(0, backup_traj_start_TT_,
                                                               tmp_pos_traj, tmp_yaw_traj)) {
                fmt::print(fg(fmt::color::indian_red)," -- [SUPER] in [PlanFromRest] getPartialTrajectoryByTime failed.\n");
                return false;
            }
            pos_traj_ = tmp_pos_traj + backup_traj.posTraj();
            yaw_traj_ = tmp_yaw_traj + backup_traj.yawTraj();

            start_WT_ = pos_traj_.start_WT;

            flag_empty_ = false;
            flag_backup_traj_avilibale_ = true;
            checkFirstPartBackupTraj(exp_traj);
            return true;
        }

        void setTrajectory(const ExpTraj&exp_traj) {
            LOCK_G
            pos_traj_ = exp_traj.posTraj();
            yaw_traj_ = exp_traj.yawTraj();
            start_WT_ = pos_traj_.start_WT;
            flag_empty_ = false;
            backup_traj_start_TT_ = 99999999;
            flag_backup_traj_avilibale_ = false;
            checkFirstPartBackupTraj(exp_traj);
        }

        bool isTTOnBackupTraj(const double & checkTT) const {
            if(!backupTrajAvilibale()) {
                return false;
            }

            if(first_part_exp_has_backup_traj_ &&
                (checkTT<on_backup_end_TT_ && checkTT>on_backup_start_TT_)) {
                return true;
            }

            if(checkTT > backup_traj_start_TT_) {
                return true;
            }

            return false;
        }

        bool backupTrajAvilibale() const {
            return flag_backup_traj_avilibale_;
        }


        double getTotalDuration() const {
            return pos_traj_.getTotalDuration();
        }

        double getBackupTrajStartTT() const {
            return backup_traj_start_TT_;
        }

        Vec3f getPos(const double & t)const {
            return pos_traj_.getPos(t);
        }

        Vec3f getVel(const double & t)const {
            return pos_traj_.getVel(t);
        }

        Vec3f getYaw(const double & t)const {
            return yaw_traj_.getPos(t);
        }

        Vec3f getYawRate(const double & t)const {
            return yaw_traj_.getVel(t);
        }

        StatePVAJ getYawState(const double &t)const {
            return yaw_traj_.getState(t);
        }

        const Trajectory & posTraj() const {
            return pos_traj_;
        }

        const Trajectory & yawTraj() const {
            return yaw_traj_;
        }


        const double & getStartWallTime() const {
            return pos_traj_.start_WT;
        }

        bool getPartialTrajectoryByTrajectoryTime(const double & start_t,
            const double & end_t,
            Trajectory & partial_pos_traj,
            Trajectory & partial_yaw_traj) {

            if(!pos_traj_.getPartialTrajectoryByTime(start_t,end_t,partial_pos_traj)) {
                return false;
            }

            if(!yaw_traj_.getPartialTrajectoryByTime(start_t, end_t,partial_yaw_traj)) {
                return false;
            }

            return true;
        }

    };
}

#endif //EXP_TRAJ_H
