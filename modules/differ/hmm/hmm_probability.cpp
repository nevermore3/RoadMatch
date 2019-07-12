//
// Created by Liu Jian on 2019/3/5.
//

#include "hmm_probability.h"
#include "geom/geo_util.h"
#include "data_manager/mesh_manager.h"
#include "pathengine/path_engine.h"

#define measurementErrorSigma 20.0
#define transitionProbabilityBeta 0.00959442
#define timeDifference(x) ( (x * 1.0) / 400.0)

#define PI 3.141592653589793
#define normalDistribution(sigma,x) ((1.0 / (sqrt(2.0 * PI) * (sigma) )) * exp(-0.5*pow(x / (sigma) , 2)))
#define exponentialDistribution(beta,x) (1.0 / (beta) * exp(-(x)/(beta))
#define logExponentialDistribution(beta,x) (log(1.0 / (beta) ) - ((x) / (beta)))
#define linearDistance(x,y) (geo::geo_util::getDistance((x)->lng_,(x)->lat_,(y)->lng_,(y)->lat_))

//距离越近,值越大
double HmmProbability::EmissionLogProbability(shared_ptr<Bind> roadPosition) {

 //   return normalDistribution(measurementErrorSigma,(roadPosition->distance_/100));
    int dist = roadPosition->distance_;
    return log(normalDistribution(measurementErrorSigma,(dist/100)));
}

double HmmProbability::TransitionLogProbability(shared_ptr<Bind> sourcePosition,
                                                shared_ptr<Bind> targetPosition,
                                                unordered_map<string, list<shared_ptr<KDRoad>>>& path_map,
                                                MeshManager *meshManage) {
    double transitionMetric = NormalizedTransitionMetric(sourcePosition,
                                                         targetPosition,
                                                         path_map,meshManage);

    if (transitionMetric == -1) {
        return -INFINITY;
    } else {
 //       return exponentialDistribution(transitionProbabilityBeta,transitionMetric));
        return logExponentialDistribution(transitionProbabilityBeta,transitionMetric);
    }
}

double HmmProbability::NormalizedTransitionMetric(shared_ptr<Bind> sourcePosition,
                                                  shared_ptr<Bind> targetPosition,
                                                  unordered_map<string, list<shared_ptr<KDRoad>>>& path_map,
                                                  MeshManager *meshManage) {
    double time_diff = 1;  // to be re set

    double linear_distance = linearDistance(sourcePosition->query_point_,targetPosition->query_point_);

    double route_length= GetRoutelength(sourcePosition, targetPosition, path_map,meshManage);

    if (route_length == -1) {
        return -1;
    }else{
        return fabs(linear_distance - route_length) / (linear_distance / 16.666666666667 *1000 / 40);
    }

}

double HmmProbability::GetRoutelength(shared_ptr<Bind> sourcePosition,
                                      shared_ptr<Bind> targetPosition,
                                      unordered_map<string, list<shared_ptr<KDRoad>>>& path_map,
                                      MeshManager *meshManage) {
    shared_ptr<KDRoad> source = sourcePosition->match_road_;
    shared_ptr<KDRoad> target = targetPosition->match_road_;
    //同一条link的处理
    if (source->id_ == target->id_ && source->mesh_id_ == target->mesh_id_) {
        if (source->direction_ == 2) {
            // 起始点在终点之前
            if (sourcePosition->pos_index_ > targetPosition->pos_index_)
                return -1;
            else {
                return linearDistance(sourcePosition->snapped_point_,
                        targetPosition->snapped_point_);
            }
        } else if (source->direction_ == 3) {
            if (sourcePosition->pos_index_ < targetPosition->pos_index_)
                return  -1;
            else {
                return linearDistance(sourcePosition->snapped_point_,
                                        targetPosition->snapped_point_);
            }
        } else {
            return linearDistance(sourcePosition->snapped_point_,
                                    targetPosition->snapped_point_);
        }
    }

    string path_key = GetPathKey(sourcePosition, targetPosition);
    auto road_list_it = path_map.find(path_key);
    if(road_list_it != path_map.end()) {
        list<shared_ptr<KDRoad>>& road_list = road_list_it->second;
        return BuilePathLength(road_list, sourcePosition, targetPosition, meshManage);
    } else {
        PathEngine engine;
        list<shared_ptr<KDRoad>> road_list;
        if (engine.FindPath(meshManage, sourcePosition->match_road_,sourcePosition->snapped_point_,
                            targetPosition->match_road_, targetPosition->snapped_point_, road_list)) {
            double length = BuilePathLength(road_list, sourcePosition, targetPosition, meshManage);
            path_map.insert(make_pair(path_key, road_list));
            return length;
        } else {
            return -1;
        }
    }
}

string HmmProbability::GetPathKey(shared_ptr<Bind> sourcePosition,
                                  shared_ptr<Bind> targetPosition) {
    string src = sourcePosition->match_road_->mesh_id_ +
                        to_string(sourcePosition->match_road_->id_);
    string des = targetPosition->match_road_->mesh_id_ +
                 to_string(targetPosition->match_road_->id_);
    return (src + des);
}

double HmmProbability::BuilePathLength(const list<shared_ptr<KDRoad>>& road_list,
                                       shared_ptr<Bind> sourcePosition,
                                       shared_ptr<Bind> targetPosition,
                                       MeshManager *meshManage) {
    double length = 0;

    auto road_it = road_list.begin();
    auto next_it = road_it;
    PathEngine::ConnType last_conn_type = PathEngine::TAIL_HEAD;
    while(road_it != road_list.end()) {
        ++next_it;
        auto road = *road_it;
        if(next_it == road_list.end()) {
            if (road->direction_ == 3)
                length += targetPosition->length_to_end_;
            else if (road->direction_ == 2)
                length += targetPosition->length_to_start_;
            else {
                if (last_conn_type == PathEngine::HEAD_HEAD ||
                        last_conn_type == PathEngine::TAIL_HEAD) {
                    length += targetPosition->length_to_start_;
                } else {
                    length += targetPosition->length_to_end_;
                }

            }
            break;
        }
        PathEngine::ConnType conn_type =
                PathEngine::GetConnectType(meshManage, (*road_it), (*next_it));

        if(conn_type == PathEngine::UN_CONN)
            break;
        if(conn_type == PathEngine::TAIL_HEAD) {
            if (road->id_ == sourcePosition->match_road_->id_ &&
                road->mesh_id_ == sourcePosition->match_road_->mesh_id_) {
                if (road->direction_ == 3)
                    length += sourcePosition->length_to_start_;
                else
                    length += sourcePosition->length_to_end_;

            } else if (road->id_ == targetPosition->match_road_->id_ &&
                       road->mesh_id_ == targetPosition->match_road_->mesh_id_) {
                if (road->direction_ == 3)
                    length += targetPosition->length_to_end_;
                else
                    length += targetPosition->length_to_start_;
            } else
                length += road->length_;
        } else if (conn_type == PathEngine::TAIL_TAIL) {
            if (road->id_ == sourcePosition->match_road_->id_ &&
                road->mesh_id_ == sourcePosition->match_road_->mesh_id_) {
                if (road->direction_ == 3)
                    length += sourcePosition->length_to_start_;
                else
                    length += sourcePosition->length_to_end_;

            } else if (road->id_ == targetPosition->match_road_->id_ &&
                       road->mesh_id_ == targetPosition->match_road_->mesh_id_) {
                if (road->direction_ == 3)
                    length += targetPosition->length_to_end_;
                else
                    length += targetPosition->length_to_start_;
            } else
                length += road->length_;
        }  else if (conn_type == PathEngine::HEAD_HEAD) {
            if (road->id_ == sourcePosition->match_road_->id_ &&
                road->mesh_id_ == sourcePosition->match_road_->mesh_id_) {
                if (road->direction_ == 3)
                    length += sourcePosition->length_to_start_;
                else
                    length += sourcePosition->length_to_start_;

            } else if (road->id_ == targetPosition->match_road_->id_ &&
                       road->mesh_id_ == targetPosition->match_road_->mesh_id_) {
                if (road->direction_ == 3)
                    length += targetPosition->length_to_end_;
                else
                    length += targetPosition->length_to_start_;
            } else
                length += road->length_;
        } else if (conn_type == PathEngine::HEAD_TAIL) {
            if (road->id_ == sourcePosition->match_road_->id_ &&
                road->mesh_id_ == sourcePosition->match_road_->mesh_id_) {
                if (road->direction_ == 3)
                    length += sourcePosition->length_to_start_;
                else
                    length += sourcePosition->length_to_start_;

            } else if (road->id_ == targetPosition->match_road_->id_ &&
                       road->mesh_id_ == targetPosition->match_road_->mesh_id_) {
                if (road->direction_ == 3)
                    length += targetPosition->length_to_end_;
                else
                    length += targetPosition->length_to_start_;
            } else
                length += road->length_;
        }

        road_it = next_it;
        last_conn_type = conn_type;
    }

//    for (auto road : road_list) {
//        if (road->id_ == sourcePosition->match_road_->id_ &&
//                road->mesh_id_ == sourcePosition->match_road_->mesh_id_) {
//            if (road->direction_ == 3)
//                length += sourcePosition->length_to_start_;
//            else
//                length += sourcePosition->length_to_end_;
//
//        } else if (road->id_ == targetPosition->match_road_->id_ &&
//                   road->mesh_id_ == targetPosition->match_road_->mesh_id_) {
//            if (road->direction_ == 3)
//                length += targetPosition->length_to_end_;
//            else
//                length += targetPosition->length_to_start_;
//        } else
//            length += road->length_;
//    }
    return length;
}