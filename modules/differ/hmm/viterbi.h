#ifndef ROAD_MATCH_SERVICE_VITERBI_H
#define ROAD_MATCH_SERVICE_VITERBI_H

#include "bind.h"
#include "hmm_probability.h"

#include <list>

class MostLikelySequence {
public:
    //保存最后匹配到的道路列表
    list<shared_ptr<Bind>> sequence_;

    //对应的概率值
    list<unordered_map<shared_ptr<Bind>, double>> probability_sequence_;

    // 保存每个节点 的前一个最大概率节点
    list<unordered_map<shared_ptr<Bind>, shared_ptr<Bind>>> back_pointer_sequence_;

    void clear() {
        sequence_.clear();
        probability_sequence_.clear();
        back_pointer_sequence_.clear();
    }
};

//
class ForwardStepResult {
public:
    unordered_map<shared_ptr<Bind>, double> probability_;

    //保存上个最大概率的状态， 可以回溯找到路径
    unordered_map<shared_ptr<Bind>, shared_ptr<Bind>> back_pointers_;
};

class Viterbi {
public:

    static Viterbi *GetInstance() {
        static Viterbi instance_;
        return &instance_;
    }

    bool Compute(const list<shared_ptr<CadidatesStep>> &steplist,
                 bool keepMessageHistory,
                 list<shared_ptr<KDRoad>> &res_list,
                 IManager *mesh_manage, bool slip);

    bool InitalProbability(shared_ptr<CadidatesStep> firstTimeStep,
                           unordered_map<shared_ptr<Bind>, double> &probablity);

private:

    bool HmmBreak(unordered_map<shared_ptr<Bind>, double> probability);

    void ForwardStep(shared_ptr<CadidatesStep> prevStep,
                     shared_ptr<CadidatesStep> curStep,
                     const unordered_map<shared_ptr<Bind>, double> &probability,
                     shared_ptr<ForwardStepResult> forward_result, IManager *mesh_manage);

    shared_ptr<Bind> MostLikelyState(unordered_map<shared_ptr<Bind>, double> probability);

    void RetrieveMostLikelySequence(list<unordered_map<shared_ptr<Bind>, shared_ptr<Bind>>> &backPointerSequence,
                                    shared_ptr<Bind> lastState,
                                    list<shared_ptr<Bind>> &sequence,
                                    IManager *mesh_manage);

    bool BuildFullPath(const list<shared_ptr<Bind>> &road_list,
                       list<shared_ptr<KDRoad>> &res_list, IManager *mesh_manage);

    void PathCheck(const list<shared_ptr<KDRoad>> &res_list,
                            IManager* mesh_manager,
                            list<shared_ptr<KDRoad>>& res1_list);

    bool BuildCurStateDir(shared_ptr<Bind> prevStep,
                          shared_ptr<Bind> curStep,
                          IManager *mesh_manage);

private:
    unordered_map<string, list<shared_ptr<KDRoad>>> path_map_;
};


#endif //ROAD_MATCH_SERVICE_VITERBI_H
