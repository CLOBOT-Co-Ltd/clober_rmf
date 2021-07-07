#ifndef __C_SCHEDULER_H__
#define __C_SCHEDULER_H__

#include <tc_ros/CUtils.h>
#include <tc_ros/ortools/base/logging.h>
#include <tc_ros/ortools/linear_solver/linear_solver.h>
#include <tc_ros/gurobi_c.h>
#include <tc_ros/gurobi_c++.h>

#include <algorithm>
#include <fstream>
#include <istream>
#include <typeinfo>
#include <iostream>

using namespace std;
using namespace operations_research;

namespace TCSERVER {

class CScheduler {
   public:
    CScheduler();
    ~CScheduler();

    void ClearRobotSet();
    
    void SetRobot(string id, string start, string goal);

    void SetTimePeriod(int t);

    void LoadData(string filename);

    void StartMIP();
    
    void MakeVariables(string robot_name);

    void MakeStepVariables(string robot_name);

    void MakeFlowConservation(string robot_name);

    void SetObjective(string robot_name, string goal_name);

    void GetSolution(string robot_name, vector<string> &sol);

    void MakeInitVariables(string robot_name);

    vector<string> Solve(string robot_name);

    vector<string> ConvertNodePath(string start, vector<string> state_path);

    void MakeArcCollisionCapacity();

    void MakeNodeCollisionCapacity();

    void MakeOccupiedPath(string robot_name,  int idx, vector<CNode> path);

    void MakeOccupiedNode(string robot_name, string nodeId);


   private:
    CUtils utils_;

    unique_ptr<MPSolver> solver_;

    map<string, RobotVar> t_robotset_;

    // unique_ptr<GRBEnv> env_;
    GRBEnv env;
    unique_ptr<GRBModel> model_;


    vector<int> t_timeset_;
    // node set //
    vector<string> t_nodeset_;
    // arc set //
    vector<string> t_arcset_;
    // node connection set //
    map<string, map<string, vector<string>>> t_connect_;
};
};      // namespace TCSERVER
#endif  //__C_SCHEDULER_H__