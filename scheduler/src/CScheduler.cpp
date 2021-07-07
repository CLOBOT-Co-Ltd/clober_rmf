#include <tc_ros/CScheduler.h>

namespace TCSERVER
{

    CScheduler::CScheduler()
    {  
    }

    CScheduler::~CScheduler()
    {
    }

    void CScheduler::ClearRobotSet() {
        t_robotset_.clear();
    }

    void CScheduler::SetRobot(string id, string start, string goal)
    {
        RobotVar robot;
        robot.id_ = id;
        robot.start_ = start;
        robot.goal_ = goal;

        t_robotset_.insert(make_pair(id, robot));
    }

    void CScheduler::SetTimePeriod(int t)
    {
        for (int i = 0; i < t; i++)
        {
            t_timeset_.push_back(i);
        }
    }

    void CScheduler::LoadData(string filename)
    {
        // 파일 로드 //
        utils_.ParseTestData(filename, t_nodeset_, t_arcset_, t_connect_);
    }

    void CScheduler::StartMIP()
    {
        env.set("LogFile","CScheduler.log");
        env.start();

        auto model = make_unique<GRBModel>(GRBModel(env));
        model_ = std::move(model);
    }

    void CScheduler::MakeVariables(string robot_name)
    {
        for (int t = 0; t < t_timeset_.size(); t++)
        {
            for (int a = 0; a < t_arcset_.size(); a++)
            {
                string name = robot_name + "_step_" + to_string(t) + "_" + t_arcset_[a];
                // cout << "make variable name : " << name << endl;
                GRBVar x_step = model_->addVar(0.0, 1.0, 0.0, GRB_BINARY, name);
            }      
        }
        model_->update();
    }

    void CScheduler::MakeStepVariables(string robot_name)
    {
        for (int t = 0; t < t_timeset_.size(); t++)
        {
            vector<GRBVar> step_var;
            GRBLinExpr lhsExpr;

            for (int a = 0; a < t_arcset_.size(); a++)
            {
                string name = robot_name + "_step_" + to_string(t) + "_" + t_arcset_[a];
                // cout << "step variable name : " << name << endl;
                GRBVar x_step = model_->getVarByName(name);
                step_var.push_back(x_step);
            }

            string const_name = robot_name + "_step_" + to_string(t) + "_const";
            const double coeff_ = 1.0;

            for(int i=0; i<step_var.size(); i++){
                lhsExpr.addTerms(&coeff_, &step_var[i], 1);
            }

            model_->addConstr(lhsExpr, GRB_EQUAL, 1, const_name);            
        }
    }

    void CScheduler::MakeInitVariables(string robot_name)
    {
        string start_node = "";

        auto findIt = t_robotset_.find(robot_name);
        if (findIt != t_robotset_.end())
        {
            start_node = findIt->second.start_;
        }

        if (!start_node.empty())
        {
            vector<string> arc_list;
            if (t_connect_.find(start_node) != t_connect_.end())
            {
                for (int i = 0; i < t_connect_[start_node]["stay"].size(); i++)
                {
                    arc_list.push_back(t_connect_[start_node]["stay"][i]);
                }
                for (int i = 0; i < t_connect_[start_node]["out"].size(); i++)
                {
                    arc_list.push_back(t_connect_[start_node]["out"][i]);
                }
            }

            vector<GRBVar> init_var;
            GRBVar var_name;

            for (int i = 0; i < arc_list.size(); i++)
            {
                const string name = robot_name + "_step_0_" + arc_list[i];
                cout << "init variable name : " << name << endl;
                
                GRBVar x_step = model_->getVarByName(name);
                init_var.push_back(x_step);
            }

            // constraints
            string const_name = robot_name+"_init_const";
            GRBLinExpr lhsExpr;

            const double coeff_ = 1.0;
            for(int i=0; i<init_var.size(); i++){
                lhsExpr.addTerms(&coeff_, &init_var[i], 1);
            }
            model_->addConstr(lhsExpr, GRB_EQUAL, 1, const_name);
        }
    }


    void CScheduler::MakeOccupiedNode(string robot_name, string nodeId){
        
        vector<string> arc_list;
        if (t_connect_.find(nodeId) != t_connect_.end())
        {
            for (int i = 0; i < t_connect_[nodeId]["stay"].size(); i++)
            {
                arc_list.push_back(t_connect_[nodeId]["stay"][i]);
            }
            for (int i = 0; i < t_connect_[nodeId]["out"].size(); i++)
            {
                arc_list.push_back(t_connect_[nodeId]["out"][i]);
            }
        }

        vector<GRBVar> init_var;
        GRBVar var_name;

        for (int i = 0; i < arc_list.size(); i++)
        {
            const string name = robot_name + "_step_1_" + arc_list[i];
            cout << "step 1 variable name : " << name << endl;
            
            GRBVar x_step = model_->getVarByName(name);
            init_var.push_back(x_step);
        }

        // constraints
        string const_name = robot_name+"_next_const";
        GRBLinExpr lhsExpr;

        const double coeff_ = 1.0;
        for(int i=0; i<init_var.size(); i++){
            lhsExpr.addTerms(&coeff_, &init_var[i], 1);
        }
        model_->addConstr(lhsExpr, GRB_EQUAL, 1, const_name);

    }



    void CScheduler::MakeOccupiedPath(string robot_name, int idx, vector<CNode> path)
    {
        for (int t = 0; t < t_timeset_.size(); t++)
        {
            string from; 
            string to;
            string next;
            
            string var_name;
            string arc_name;

            if (idx  < path.size()-1)
            {
                from = path[idx].nodeID_;
                from.erase(from.begin());

                to = path[idx + 1].nodeID_;
                to.erase(to.begin());
            }
            else
            {
                idx = path.size();
                from = path[idx-1].nodeID_;
                from.erase(from.begin());

                to = path[idx-1].nodeID_;
                to.erase(to.begin());
            }

            
            arc_name.push_back('a');
            arc_name.insert(arc_name.end(), from.begin(), from.end());
            arc_name.push_back('-');
            arc_name.insert(arc_name.end(), to.begin(), to.end());

            var_name = robot_name + "_step_" + to_string(t) + "_" + arc_name;
            cout <<"occupy path collision var name : "<<var_name<<endl;

            GRBVar mpVar;
            mpVar = model_->getVarByName(var_name);

            // constraints
            GRBLinExpr lhsExpr;
            const double coeff_ = 1.0;
            string const_name = robot_name + "_path_"+var_name+"const";

            lhsExpr.addTerms(&coeff_, &mpVar, 1);
            model_->addConstr(lhsExpr, GRB_EQUAL, 1, const_name);

            idx +=1;
        }
    }

    void CScheduler::MakeArcCollisionCapacity()
    {
        vector<pair<pair<string, string>, pair<GRBVar, GRBVar>>> colliVar;

        for (int t = 0; t < t_timeset_.size(); t++)
        {
            for (int a = 0; a < t_arcset_.size(); a++)
            {
                string name = t_arcset_[a];

                string from = "";
                string to = "";

                // from to 이름 찾아서 바꾸기
                auto fIt = name.find('a');
                auto tIt = name.find('-');
                if (fIt != string::npos && tIt != string::npos)
                {

                    for (int f = fIt + 1; f < tIt; f++)
                    {
                        from.push_back(name[f]);
                    }

                    for (int t = tIt + 1; t < name.size(); t++)
                    {
                        to.push_back(name[t]);
                    }
                }

                if (from != to)
                {
                    string compname;
                    compname.push_back('a');
                    compname.insert(compname.end(), to.begin(), to.end());
                    compname.push_back('-');
                    compname.insert(compname.end(), from.begin(), from.end());

                    // cout << "name : " << name << ", compname : " << compname << endl;

                    for (auto r = t_robotset_.begin(); r != t_robotset_.end(); r++)
                    {
                        for (auto k = t_robotset_.begin(); k != t_robotset_.end(); k++)
                        {

                            GRBVar var_name;
                            GRBVar var_compname;
                            var_name = model_->getVarByName(r->first + "_step_" + to_string(t) + "_" + name);
                            var_compname = model_->getVarByName(k->first + "_step_" + to_string(t) + "_" + compname);

                            if (var_name.get(GRB_StringAttr_VarName) != "")
                            {
                                if (var_compname.get(GRB_StringAttr_VarName) != "")
                                {
                                    colliVar.push_back(make_pair(make_pair(r->first, k->first), make_pair(var_name, var_compname)));
                                    // cout << var_name.get(GRB_StringAttr_VarName) <<" , "<<var_compname.get(GRB_StringAttr_VarName) << endl;
                                }
                            }
                        }
                    }
                }
            }
        }

        // constraints
        // cout << "collivar size : " << colliVar.size() << endl;
        for (int i = 0; i < colliVar.size(); i++)
        {
            string const_name = colliVar[i].first.first + "_" + colliVar[i].first.second + "_collision_" + to_string(i) + "_const";
            GRBLinExpr lhsExpr;

            const double coeff_ = 1.0;
            lhsExpr.addTerms(&coeff_, &colliVar[i].second.first, 1);
            lhsExpr.addTerms(&coeff_, &colliVar[i].second.second, 1);

            model_->addConstr(lhsExpr, GRB_LESS_EQUAL, 1, const_name);
        }
    }

    void CScheduler::MakeNodeCollisionCapacity()
    {
        // 각 노드로 진입하는 arc (stay, in)의 모든 시간과 로봇에 대해서 합이 1보다 작거나 같아야 함.
        for (auto it = t_connect_.begin(); it != t_connect_.end(); it++)
        {

            vector<string> arclist;

            for (int i = 0; i < it->second["in"].size(); i++)
            {
                arclist.push_back(it->second["in"][i]);
            }

            for (int i = 0; i < it->second["stay"].size(); i++)
            {
                arclist.push_back(it->second["stay"][i]);
            }

            for (int t = 0; t < t_timeset_.size(); t++)
            {
                vector<GRBVar> nodeCapVar;

                for (int a = 0; a < arclist.size(); a++)
                {
                    for (auto rIt = t_robotset_.begin(); rIt != t_robotset_.end(); rIt++)
                    {
                        GRBVar var_name;
                        const string name = rIt->first + "_step_" + to_string(t) + "_" + arclist[a];

                        var_name = model_->getVarByName(name);
                        if (var_name.get(GRB_StringAttr_VarName) != "")
                        {
                            nodeCapVar.push_back(var_name);
                            // cout << "node capacity variable name : " << name << endl;
                        }
                    }
                }

                // constraint
                string const_name = it->first + "_capacity_collision_" + to_string(t) + "_const";
                // cout << "node collision size : " << nodeCapVar.size() << endl;

                GRBLinExpr lhsExpr;
                const double coeff_ = 1.0;

                for(int i=0; i<nodeCapVar.size(); i++){
                    lhsExpr.addTerms(&coeff_, &nodeCapVar[i], 1);
                }

                model_->addConstr(lhsExpr, GRB_LESS_EQUAL, 1, const_name);
            }
        }
    }

    void CScheduler::MakeFlowConservation(string robot_name)
    {
        vector<FlowVar> vec_flow;

        for (int t = 1; t < t_timeset_.size(); t++)
        {
            for (auto it = t_connect_.begin(); it != t_connect_.end(); it++)
            {
                string name;
                FlowVar flow;

                for (int i = 0; i < it->second["in"].size(); i++)
                {
                    name = robot_name + "_step_" + to_string(t - 1) + "_" + it->second["in"][i];
                    flow.prev_.push_back(name);
                }
                for (int i = 0; i < it->second["stay"].size(); i++)
                {
                    name = robot_name + "_step_" + to_string(t - 1) + "_" + it->second["stay"][i];
                    flow.prev_.push_back(name);
                }
                for (int i = 0; i < it->second["out"].size(); i++)
                {
                    name = robot_name + "_step_" + to_string(t) + "_" + it->second["out"][i];
                    flow.now_.push_back(name);
                }
                for (int i = 0; i < it->second["stay"].size(); i++)
                {
                    name = robot_name + "_step_" + to_string(t) + "_" + it->second["stay"][i];
                    flow.now_.push_back(name);
                }

                vec_flow.push_back(flow);
            }
        }

        for (int i = 0; i < vec_flow.size(); i++)
        {
            vector<GRBVar> p_vec;
            vector<GRBVar> n_vec;

            for (int p = 0; p < vec_flow[i].prev_.size(); p++)
            {
                GRBVar var_name;
                var_name = model_->getVarByName(vec_flow[i].prev_[p]);

                p_vec.push_back(var_name);
            }

            for (int n = 0; n < vec_flow[i].now_.size(); n++)
            {
                GRBVar var_name;
                var_name = model_->getVarByName(vec_flow[i].now_[n]);

                n_vec.push_back(var_name);
            }

            string const_name = robot_name + "_flow_" + to_string(i) + "_const";
            GRBLinExpr lhsExpr, rhsExpr;
            const double coeff_ = 1.0;
            for(int i=0; i<p_vec.size(); i++){
                lhsExpr.addTerms(&coeff_, &p_vec[i], 1);
            }
            for(int i=0; i<n_vec.size(); i++){
                rhsExpr.addTerms(&coeff_, &n_vec[i], 1);
            }

            model_->addConstr(lhsExpr, GRB_EQUAL, rhsExpr, const_name);
        }
    }

    void CScheduler::SetObjective(string robot_name, string goal_name)
    {
        vector<string> arc_list;
        if (t_connect_.find(goal_name) != t_connect_.end())
        {
            for (int i = 0; i < t_connect_[goal_name]["stay"].size(); i++)
            {
                arc_list.push_back(t_connect_[goal_name]["stay"][i]);
            }
        }

        vector<GRBVar> objVars;
        for (int i = 0; i < arc_list.size(); i++)
        {
            for (int t = 0; t < t_timeset_.size(); t++)
            {
                const string name = robot_name + "_step_" + to_string(t) + "_" + arc_list[i];
                // cout << "objective variable name : " << name << endl;

                GRBVar var_name;
                var_name = model_->getVarByName(name);

                objVars.push_back(var_name);
            }
        }  

        GRBLinExpr objective;
        const double coeff_ = 1.0;
        for(int c=0; c<objVars.size(); c++){
            objective.addTerms(&coeff_, &objVars[c], 1);
        }
        model_->setObjective(objective, GRB_MAXIMIZE);
    }

    void CScheduler::GetSolution(string robot_name, vector<string> &sol)
    {
        GRBVar* v_ = model_->getVars();
        cout << model_->get(GRB_IntAttr_NumVars) << endl;

        for(int i = 0; i < model_->get(GRB_IntAttr_NumVars); i++)
        {

            if(v_[i].get(GRB_DoubleAttr_X) == 1)
            {
                if(v_[i].get(GRB_StringAttr_VarName).find(robot_name) != string::npos)
                {
                    // cout << "sol: " << v_[i].get(GRB_StringAttr_VarName) << " , " << v_[i].get(GRB_DoubleAttr_X) << endl;
                    sol.push_back(v_[i].get(GRB_StringAttr_VarName));
                }
            }
        }
    }

    vector<string> CScheduler::Solve(string robot_name)
    {   
        model_->optimize();
        LOG(INFO) << "Objective value = " << model_->get(GRB_DoubleAttr_ObjVal);

        vector<string> sol;
        GetSolution(robot_name, sol);

        vector<string> path;

        // 해가 있는 경우,
        if( sol.size() > 0 ){
            path = ConvertNodePath(t_robotset_[robot_name].start_, sol);
            env.resetParams();
        }

        // 해가 없으면 empty list 를 반환하게 됨.
        return path;
    }

    vector<string> CScheduler::ConvertNodePath(string start, vector<string> state_path)
    {
        vector<string> nodePath;

        nodePath.push_back(start);

        for (int i = 0; i < state_path.size(); i++)
        {
            // afrom-to 로 노드 경로 만들기
            string from = "";
            string to = "";

            auto fIt = state_path[i].find('a');
            auto tIt = state_path[i].find('-');
            if (fIt != string::npos && tIt != string::npos)
            {
                for (int f = fIt + 1; f < tIt; f++)
                {
                    from.push_back(state_path[i][f]);
                }

                for (int t = tIt + 1; t < state_path[i].size(); t++)
                {
                    to.push_back(state_path[i][t]);
                }
            }

            if (from == to)
            {
                continue;
            }

            string name = "n" + to;
            nodePath.push_back(name);
        }
        return nodePath;
    }
} // namespace TCSERVER