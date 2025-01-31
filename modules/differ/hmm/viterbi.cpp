#include "viterbi.h"
#include "pathengine/path_engine.h"
#include "hmm_probability.h"

#include "glog/logging.h"
#include "glog/log_severity.h"

#include "util/distance.h"
#include "util/match_geo_util.h"
#include "util/mesh_feature_key_util.h"
#include "geom/geo_util.h"

#include <iomanip>

//#define DEBUG_MATCHING

bool Viterbi::Compute(const list<shared_ptr<CadidatesStep>> &steplist,
                      bool keepMessageHistory,
                      list<shared_ptr<KDRoad>> &res_list,
                      IManager *mesh_manage, bool slip) {
    if (steplist.empty())
        return false;

    auto step_it = steplist.begin();
    shared_ptr<CadidatesStep> step = *step_it;
    unordered_map<shared_ptr<Bind>, double> probablity;
    if (!InitalProbability(step, probablity))
        return false;

    if (HmmBreak(probablity))
        return false;

    MostLikelySequence mostLikelySequence;

    ++step_it;

    bool is_connect = true;
    for (; step_it != steplist.end(); ++step_it) {
        shared_ptr<CadidatesStep> prev_time_step = step;
        step = *step_it;

        shared_ptr<ForwardStepResult> forward_result = make_shared<ForwardStepResult>();

        ForwardStep(prev_time_step, step, probablity, forward_result, mesh_manage);

        if (HmmBreak(forward_result->probability_)) {
            if (!mostLikelySequence.probability_sequence_.empty()) {
                shared_ptr<Bind> most_likely_state = MostLikelyState(mostLikelySequence.probability_sequence_.back());

                RetrieveMostLikelySequence(mostLikelySequence.back_pointer_sequence_,
                                           most_likely_state,
                                           mostLikelySequence.sequence_,
                                           mesh_manage);
                BuildFullPath(mostLikelySequence.sequence_, res_list, mesh_manage);
                mostLikelySequence.clear();
            }
            InitalProbability(step, probablity);
            is_connect = false;
            continue;
        }
        probablity = forward_result->probability_;

        if (!forward_result->back_pointers_.empty())
            mostLikelySequence.back_pointer_sequence_.emplace_back(forward_result->back_pointers_);

        mostLikelySequence.probability_sequence_.emplace_back(forward_result->probability_);

    }
    if (!mostLikelySequence.probability_sequence_.empty()) {
        shared_ptr<Bind> most_likely_state = MostLikelyState(mostLikelySequence.probability_sequence_.back());

        RetrieveMostLikelySequence(mostLikelySequence.back_pointer_sequence_,
                                   most_likely_state,
                                   mostLikelySequence.sequence_,
                                   mesh_manage);

        BuildFullPath(mostLikelySequence.sequence_, res_list, mesh_manage);
    }

    if(!is_connect&&(res_list.size()<3 || slip))
        res_list.clear();


    return true;
}

// 初始化概率
bool Viterbi::InitalProbability(shared_ptr<CadidatesStep> firstTimeStep,
                                unordered_map<shared_ptr<Bind>, double> &prob) {
    double probability = -1;
    for (auto state : firstTimeStep->candidates_) {
        probability = HmmProbability::EmissionLogProbability(state);
        prob.insert(make_pair(state, probability));
    }
    return true;
}

bool Viterbi::HmmBreak(unordered_map<shared_ptr<Bind>, double> message) {
    for (auto iter : message) {
        if (iter.second != DoubleNegInfinity) {
            return false;
        }
    }
    return true;
}

void Viterbi::ForwardStep(shared_ptr<CadidatesStep> prevStep,
                          shared_ptr<CadidatesStep> curStep,
                          const unordered_map<shared_ptr<Bind>, double> &probability_,
                          shared_ptr<ForwardStepResult> forward_result,
                          IManager *mesh_manage) {
    for (auto curState : curStep->candidates_) {
        //连通性概率
        double maxLogProbability = DoubleNegInfinity;
        shared_ptr<Bind> maxPrevState;
        //找出之前的step候选项到当前step可能性最大的

        for (auto prevState : prevStep->candidates_) {
            double logProbability = HmmProbability::TransitionLogProbability(prevState,
                                                                             curState, path_map_, mesh_manage);

//            double angleDiffProbability =
//                    HmmProbability::AngleDiffLogProbability(prevState, curState, last_angle, path_map_, mesh_manage);
//
//            logProbability += angleDiffProbability;

            auto prob_it = probability_.find(prevState);
            if (prob_it != probability_.end()) {
                logProbability += prob_it->second;
            } else
                continue;

            if (logProbability > maxLogProbability) {
                maxLogProbability = logProbability;
                maxPrevState = prevState;
            }
        }

        if (maxPrevState == nullptr)
            continue;

#ifdef DEBUG_MATCHING
        cout<<setprecision(15)<<curState->match_road_->id_<<" "<<maxPrevState->match_road_->id_
            <<" "<<maxLogProbability
            <<" ("<<curState->query_point_->lng_<<","<<curState->query_point_->lat_<<")"<<endl;
#endif
        //距离概率
        double emission_probability = HmmProbability::EmissionLogProbability(curState);

        forward_result->probability_.insert(make_pair(curState, maxLogProbability + emission_probability));

        forward_result->back_pointers_.insert(make_pair(curState, maxPrevState));
    }
}

shared_ptr<Bind> Viterbi::MostLikelyState(unordered_map<shared_ptr<Bind>, double> message) {
    double max_probability = DoubleNegInfinity;
    shared_ptr<Bind> most_likely_state = nullptr;
    for (auto state_it : message) {
        double probability = state_it.second;
        if (probability > max_probability) {
            max_probability = probability;
            most_likely_state = state_it.first;
        }
    }
    return most_likely_state;
}

void Viterbi::RetrieveMostLikelySequence(list<unordered_map<shared_ptr<Bind>, shared_ptr<Bind>>> &backPointerSequence,
                                         shared_ptr<Bind> lastState,
                                         list<shared_ptr<Bind>> &sequence,
                                         IManager *mesh_manage) {
    sequence.emplace_back(lastState);
    int cnt = 0;
    backPointerSequence.reverse();

    shared_ptr<Bind> prevState = nullptr;

    for (auto bind_map : backPointerSequence) {
        cnt++;
        for (auto iter : bind_map) {

            if (lastState->match_road_->id_ == iter.first->match_road_->id_ &&
                lastState->match_road_->mesh_id_ == iter.first->match_road_->mesh_id_) {

                if (lastState->match_road_->id_ != iter.second->match_road_->id_ ||
                    lastState->match_road_->mesh_id_ != iter.second->match_road_->mesh_id_) {

                    lastState = iter.second;
                    sequence.emplace_back(lastState);
                    break;
                } else {
                    lastState = iter.second;
                    break;
                }
            }
        }
    }
    sequence.reverse();
}

bool Viterbi::BuildFullPath(const list<shared_ptr<Bind>> &bind_list,
                            list<shared_ptr<KDRoad>> &res_list,
                            IManager *mesh_manage) {
    list<shared_ptr<KDRoad>> temp_list;
    PathEngine engine;
    auto bind_it = bind_list.begin();
    while (bind_it != bind_list.end()) {
        shared_ptr<Bind> band1 = *bind_it;
        ++bind_it;
        if (bind_it == bind_list.end())
            break;
//        res_list.emplace_back(band1->match_road_);
        shared_ptr<Bind> band2 = *bind_it;
        if (engine.IsRoadConnect(mesh_manage, band1->match_road_, band2->match_road_)) {
            temp_list.emplace_back(band1->match_road_);
        } else {
            std::list<shared_ptr<KDRoad>> sub_list;
            if (engine.FindPath(mesh_manage, band1->s2e_, band1->match_road_,band1->snapped_point_,
                                band2->match_road_, band2->snapped_point_, sub_list)) {
                auto road_cnt = sub_list.size();
                int index = 0;
                for (auto road : sub_list) {
                    index++;
                    if (index == road_cnt)
                        break;
                    temp_list.emplace_back(road);
                }
            } else {
                return false;
            }
        }
    }

    temp_list.emplace_back(bind_list.back()->match_road_);

    PathCheck(temp_list, mesh_manage, res_list);

    return true;
}

void Viterbi::PathCheck(const list<shared_ptr<KDRoad>> &res_list,
                        IManager* mesh_manager,
                        list<shared_ptr<KDRoad>>& res1_list) {
    vector<shared_ptr<KDRoad>> res_vector;
    for (auto road: res_list) {
        res_vector.emplace_back(road);
    }

    if(res_vector.size() <= 2) {
        for (size_t i = 0; i < res_vector.size(); ++i) {
            res1_list.emplace_back(res_vector[i]);
        }
        return;
    }

    for (size_t i = 0; i < res_vector.size() - 2; ++i) {
        auto road1 = res_vector[i];
        auto road2 = res_vector[i+1];
        auto road3 = res_vector[i+2];
        if(i == 0)
            res1_list.emplace_back(road1);
        if(PathEngine::IsRoadConnect(mesh_manager, road1, road3)) {
            res1_list.emplace_back(road3);
            ++i;
        } else {
            res1_list.emplace_back(road2);
        }
    }

    res1_list.emplace_back(res_vector[res_vector.size()-1]);
}

bool Viterbi::BuildCurStateDir(shared_ptr<Bind> prevState,
                               shared_ptr<Bind> curState,
                                IManager *mesh_manage) {
    PathEngine::ConnType connType =
            PathEngine::GetConnectType(mesh_manage, prevState->match_road_, curState->match_road_);
    if (connType == PathEngine::HEAD_TAIL || connType == PathEngine::TAIL_TAIL) {
        curState->s2e_ = 0;
    } else if (connType == PathEngine::HEAD_HEAD || connType == PathEngine::TAIL_HEAD) {
        curState->s2e_ = 1;
    }

    return true;
}
