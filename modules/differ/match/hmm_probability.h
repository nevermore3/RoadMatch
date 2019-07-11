//
// Created by Liu Jian on 2019/3/5.
//

#ifndef ROAD_MATCH_SERVICE_HMMPROBABILITY_H
#define ROAD_MATCH_SERVICE_HMMPROBABILITY_H

#include "bind.h"
class MeshManage;

class HmmProbability {
public:
    static double EmissionLogProbability(shared_ptr<Bind> roadPosition);

    static double TransitionLogProbability(shared_ptr<Bind> sourcePosition,
                                           shared_ptr<Bind> targetPosition,
                                           unordered_map<string, list<shared_ptr<KDRoad>>>& path_map,MeshManage *meshManage);
private:
    static double NormalizedTransitionMetric(shared_ptr<Bind> sourcePosition,
                                             shared_ptr<Bind> targetPosition,
                                             unordered_map<string, list<shared_ptr<KDRoad>>>& path_map,MeshManage *meshManage);

    static double GetRoutelength(shared_ptr<Bind> sourcePosition,
                                 shared_ptr<Bind> targetPosition,
                                 unordered_map<string, list<shared_ptr<KDRoad>>>& path_map,MeshManage *meshManage);

    static string GetPathKey(shared_ptr<Bind> sourcePosition,
                             shared_ptr<Bind> targetPosition);

    static double BuilePathLength(const list<shared_ptr<KDRoad>>& road_list,
                           shared_ptr<Bind> sourcePosition,
                           shared_ptr<Bind> targetPosition);
};


#endif //ROAD_MATCH_SERVICE_HMMPROBABILITY_H
