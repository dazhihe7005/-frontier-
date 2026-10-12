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


#ifndef SUPER_FSM_CONFIG_HPP
#define SUPER_FSM_CONFIG_HPP


#include <super_core/config.hpp>
#include <vector>
#include <cstring>
#include <utils/header/yaml_loader.hpp>

namespace fsm {
    using namespace traj_opt;
    using namespace super_planner;
    static constexpr int MPC_PVAJ_MODE = 1;
    static constexpr int MPC_POLYTRAJ_MODE = 2;

    class Config {
    public:
        bool timer_en{true};

        // Fsm Params
        bool click_goal_en{}, goal_command_en{}, visualization_en{};
        double replan_rate{}, resolution{};
        double click_height{};
        double goal_snap_max_distance{3.0};
        double odom_timeout{2.0};

        bool click_yaw_en{};
        string cmd_topic, mpc_cmd_topic, click_goal_topic, goal_command_topic;
        double yaw_dot_max{};

        Config() = default;

        Config(const std::string & cfg_path) {
            yaml_loader::YamlLoader loader(cfg_path);
            vector<double> tem_gain;
            loader.LoadParam("fsm/timer_en", timer_en, false);
            loader.LoadParam("fsm/click_goal_en", click_goal_en, false);
            loader.LoadParam("fsm/goal_command_en", goal_command_en, false);
            loader.LoadParam("fsm/click_yaw_en", click_yaw_en, false);
            loader.LoadParam("fsm/replan_rate", replan_rate, 10.0);
            loader.LoadParam("fsm/click_height", click_height, 1.5);
            loader.LoadParam("fsm/goal_snap_max_distance",
                             goal_snap_max_distance, 3.0);
            if (!std::isfinite(goal_snap_max_distance) ||
                goal_snap_max_distance < 0.0)
                throw std::invalid_argument("invalid goal snap distance");
            loader.LoadParam("rog_map/ros_callback/odom_timeout", odom_timeout, 2.0);
            loader.LoadParam("fsm/cmd_topic", cmd_topic, string("/planning/pos_cmd"));
            loader.LoadParam("fsm/mpc_cmd_topic", mpc_cmd_topic, string("/planning_cmd/mpc"));
            loader.LoadParam("fsm/click_goal_topic", click_goal_topic, string("/planning/click_goal_topic"));
            loader.LoadParam("fsm/goal_command_topic", goal_command_topic, string("/mine_uav/super/goal_command"));


            loader.LoadParam("super_planner/yaw_dot_max", yaw_dot_max, 1.0, true);
            loader.LoadParam("super_planner/visualization_en", visualization_en, false, true);
            loader.LoadParam("rog_map/resolution", resolution, 0.1, true);

        }
    };
}

#endif //SUPER_FSM_CONFIG_H
