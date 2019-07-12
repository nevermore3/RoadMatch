//
// Created by Liu Jian on 2019/1/16.
//

#include "path_engine.h"
#include "util/geometry_util.h"
#include "geom/geo_util.h"
#include "glog/logging.h"
#include "util/distance.h"
#include "util/mesh_feature_key_util.h"

#define ASTAR 1.0

FunctionBufferFilter::FunctionBufferFilter(){
//    func_buffer_map_.insert(make_pair(0,100000));
    func_buffer_map_.insert(make_pair(1,20000));
    func_buffer_map_.insert(make_pair(2,10000));
    func_buffer_map_.insert(make_pair(3,5000));
    func_buffer_map_.insert(make_pair(4,2000));
    func_buffer_map_.insert(make_pair(5,1000));
}
bool FunctionBufferFilter::CheckRoadValid(int fun,double dis){
    //避免异常
    if(fun <= 0 || fun > 6)
        return true;

    double check_dis = func_buffer_map_[fun];
    if(dis > check_dis)
        return false;

    return true;
}

PathEngine::PathEngine() {
    search_count_ = 5000;
    fc_valid_ = true;
}

void PathEngine::SetSearchCount(int count){
    search_count_ = count;
}

void PathEngine::SetFunctionClassFilterValid(bool valid){
    fc_valid_ = valid;
}

bool PathEngine::Init(MeshManager* mesh_manage, shared_ptr<KDRoad> road_src,
                      shared_ptr<KDRoad> road_dst) {
    if(nullptr == road_src || nullptr == road_dst || nullptr == mesh_manage)
        return false;

    des_link_ = road_dst;
    src_link_ = road_src;

    if (0 == road_src->direction_ || 1 == road_src->direction_) {
        shared_ptr<KDRoadNode> node1 = mesh_manage->GetMesh(road_src->mesh_id_)->road_nodes_[road_src->f_node_id_];
        if(!SetSource(mesh_manage, road_src, node1))
            return false;

        shared_ptr<KDRoadNode> node2 = mesh_manage->GetMesh(road_src->mesh_id_)->road_nodes_[road_src->t_node_id_];
        if(!SetSource(mesh_manage, road_src, node2))
            return false;
    }

    if (2 == road_src->direction_) {
        shared_ptr<KDRoadNode> node = mesh_manage->GetMesh(road_src->mesh_id_)->road_nodes_[road_src->t_node_id_];
        if(!SetSource(mesh_manage, road_src, node))
            return false;
    }

    if (3 == road_src->direction_) {
        shared_ptr<KDRoadNode> node = mesh_manage->GetMesh(road_src->mesh_id_)->road_nodes_[road_src->f_node_id_];
        if(!SetSource(mesh_manage, road_src, node))
            return false;
    }

    return true;
}

void PathEngine::Clear() {
    cout<<"path_close_map size="<<path_close_map_.size()<<endl;
    cout<<"path_open_map_ size="<<path_open_map_.size()<<endl;

    path_close_map_.clear();
    path_open_map_.clear();

    while (!path_heap_.empty()) {
        path_heap_.pop();
    }
}

bool PathEngine::Recall(shared_ptr<SearchLink> current_link, std::list<shared_ptr<KDRoad>>& result) {
    result.push_back(current_link->get_kdroad());
    while (nullptr != current_link->get_parent()) {
        shared_ptr<SearchLink> link = current_link->get_parent();
        result.push_front(link->get_kdroad());
        current_link = link;
    }

    return !result.empty();
}

bool PathEngine::SetSource(MeshManager *mesh_manage, shared_ptr<KDRoad> road_src, shared_ptr<KDRoadNode> node) {
    shared_ptr<SearchLink> src = make_shared<SearchLink>();
    src->set_length(road_src->length_);
    src->set_kdroad(road_src);

    if (node->adj_id_ != -1) {
        shared_ptr<MeshObj> adjmesh = mesh_manage->GetMesh(node->adj_mesh_id_);
        node = adjmesh->road_nodes_[node->adj_id_];
        if(node == nullptr)
            return false;
    }
    int32_t weight = road_src->length_ +
                     (int32_t)geo::geo_util::getDistance(node->coord_.lng_, node->coord_.lat_,
                                                         des_coord_->lng_,des_coord_->lat_) * ASTAR;
    src->set_weight(weight);
    src->set_kdnode(node);
    path_heap_.push(src);

    return true;
}

bool PathEngine::FindPath(MeshManager* mesh_manage,
                          shared_ptr<KDRoad> road_src,
                          shared_ptr<KDCoord> src_coord,
                          shared_ptr<KDRoad> road_dst,
                          shared_ptr<KDCoord> des_coord,
                          std::list<shared_ptr<KDRoad>>& result) {
    if(nullptr == road_src || nullptr == road_dst || nullptr == mesh_manage)
        return false;

    des_link_ = road_dst;
    src_coord_ = src_coord;
    src_link_ = road_src;
    des_coord_ = des_coord;

    if (0 == road_src->direction_ || 1 == road_src->direction_) {
        shared_ptr<KDRoadNode> node1 = mesh_manage->GetMesh(road_src->mesh_id_)->road_nodes_[road_src->f_node_id_];
        if(!SetSource(mesh_manage, road_src, node1))
            return false;

        shared_ptr<KDRoadNode> node2 = mesh_manage->GetMesh(road_src->mesh_id_)->road_nodes_[road_src->t_node_id_];
        if(!SetSource(mesh_manage, road_src, node2))
            return false;
    }

    if (2 == road_src->direction_) {
        shared_ptr<KDRoadNode> node = mesh_manage->GetMesh(road_src->mesh_id_)->road_nodes_[road_src->t_node_id_];
        if(!SetSource(mesh_manage, road_src, node))
            return false;
    }

    if (3 == road_src->direction_) {
        shared_ptr<KDRoadNode> node = mesh_manage->GetMesh(road_src->mesh_id_)->road_nodes_[road_src->f_node_id_];
        if(!SetSource(mesh_manage, road_src, node))
            return false;
    }

    bool res = false;
    int loopcnt = 0;
    while (!path_heap_.empty() && loopcnt++ < search_count_) {
        shared_ptr<SearchLink> current_link = path_heap_.top();
        path_heap_.pop();
        if(current_link->get_kdroad()->id_ == des_link_->id_
           && current_link->get_kdroad()->mesh_id_ == des_link_->mesh_id_ ) {
            res = Recall( current_link, result);
            break;
        }
        ExtendPath(mesh_manage, current_link, true);

    }

    Clear();

    return res;
}

bool PathEngine::FindPath(MeshManager* mesh_manage,
                          std::list<shared_ptr<KDRoad>>& result) {
    if(nullptr == mesh_manage)
        return false;
    bool res = false;
    int loopcnt = 0;
    while (!path_heap_.empty() && loopcnt++ < search_count_) {
        shared_ptr<SearchLink> current_link = path_heap_.top();
        path_heap_.pop();

        if(current_link->get_kdroad()->id_ == des_link_->id_
           && current_link->get_kdroad()->mesh_id_ == des_link_->mesh_id_) {
            res = Recall( current_link, result);
            break;
        }

        ExtendPath(mesh_manage, current_link, true);
    }
    Clear();

    return res;
}

double PathEngine::GetFilterQueryDistance(const KDCoord &node) {
    double from_dis = Distance::distance(node.lng_, node.lat_, src_coord_->lng_, src_coord_->lat_);
    double to_dis = Distance::distance(node.lng_, node.lat_, des_coord_->lng_, des_coord_->lat_);
    double min_dis = from_dis < to_dis ? from_dis : to_dis;
    return min_dis / 100;
}

void PathEngine::ExtendPath(MeshManager* mesh_manage, shared_ptr<SearchLink> cur_link, bool forward) {
    shared_ptr<KDRoadNode> cur_node_ = cur_link->get_kdnode();
    if (nullptr == cur_node_)
        return;
    shared_ptr<MeshObj> mesh = mesh_manage->GetMesh(cur_node_->mesh_id_);
    if (nullptr == mesh)
        return;

    double filter_query_dis = GetFilterQueryDistance(cur_node_->coord_);

    for (auto out_road : cur_node_->from_roads_) {
        if (out_road->id_ == cur_link->get_kdroad()->id_ &&
            out_road->mesh_id_ == cur_link->get_kdroad()->mesh_id_)
            continue;

        //按距离排除
        if(fc_valid_){
            if(!FunctionBufferFilter::GetInstance()->CheckRoadValid(out_road->f_class_,filter_query_dis))
                continue;
        }

        string road_key = MeshFeatureKeyUtil::BuildKey(out_road);
        auto iter = path_close_map_.find(road_key);
        if (iter != path_close_map_.end()) {
            if (iter->second->get_length() < cur_link->get_length() + out_road->length_) {
                continue;
            }
        }

        shared_ptr<SearchLink> search_link = make_shared<SearchLink>();
        search_link->set_kdroad(out_road);

        if(out_road->t_node_id_ == cur_node_->id_) {
            shared_ptr<KDRoadNode> outnode = mesh->road_nodes_[out_road->f_node_id_];
            if(outnode == nullptr)
                continue;

            if (outnode->adj_id_ != -1) {
                shared_ptr<MeshObj> adjmesh = mesh_manage->GetMesh(outnode->adj_mesh_id_);
                outnode = adjmesh->road_nodes_[outnode->adj_id_];
                if(outnode == nullptr)
                    continue;
            }
            search_link->set_kdnode(outnode);
        } else {
            shared_ptr<KDRoadNode> outnode = mesh->road_nodes_[out_road->t_node_id_];
            if(outnode == nullptr)
                continue;

            if (outnode->adj_id_ != -1) {
                shared_ptr<MeshObj> adjmesh = mesh_manage->GetMesh(outnode->adj_mesh_id_);
                outnode = adjmesh->road_nodes_[outnode->adj_id_];
                if(outnode == nullptr)
                    continue;
            }
            search_link->set_kdnode(outnode);
        }

        if(out_road->id_ == des_link_->id_ && out_road->mesh_id_ == des_link_->mesh_id_) {
            int length = cur_link->get_length() +
                         (int32_t)geo::geo_util::getDistance(cur_node_->coord_.lng_,
                                                             cur_node_->coord_.lat_,
                                                             des_coord_->lng_,des_coord_->lat_);
            search_link->set_length(length);

            search_link->set_weight(length);
        } else {
            search_link->set_length(cur_link->get_length() + out_road->length_);

            int32_t weight = search_link->get_length() +
                             (int32_t)geo::geo_util::getDistance(search_link->get_kdnode()->coord_.lng_,
                                                                 search_link->get_kdnode()->coord_.lat_,
                                                                 des_coord_->lng_,des_coord_->lat_) * ASTAR;

            search_link->set_weight(weight);
        }

        search_link->set_parent(cur_link);

        if(path_open_map_.find(road_key) == path_open_map_.end()) {
            path_open_map_.insert(make_pair(road_key, search_link));
            path_heap_.push(search_link);
        }
    }

    path_close_map_.insert(std::make_pair(MeshFeatureKeyUtil::BuildKey(cur_link->get_kdroad()), cur_link));
}

bool PathEngine::IsRoadConnect(MeshManager *mesh_manage,
                               const shared_ptr<KDRoad> road1,
                               const shared_ptr<KDRoad> road2) {
    bool res = false;
    if (road1->mesh_id_ == road2->mesh_id_) {
        if (road1->t_node_id_ == road2->f_node_id_ ||
            road1->t_node_id_ == road2->t_node_id_ ||
            road1->f_node_id_ == road2->f_node_id_ ||
            road1->f_node_id_ == road2->t_node_id_)
            res = true;
    } else {
        shared_ptr<MeshObj> mesh_obj = mesh_manage->GetMesh(road1->mesh_id_);

        shared_ptr<KDRoadNode> f_node1 = mesh_obj->road_nodes_[road1->f_node_id_];
        shared_ptr<KDRoadNode> t_node1 = mesh_obj->road_nodes_[road1->t_node_id_];

        if (f_node1->adj_id_ != -1) {
            if (f_node1->adj_id_ == road2->f_node_id_ ||
                f_node1->adj_id_ == road2->t_node_id_)
                res = true;
        }

        if (t_node1->adj_id_ != -1) {
            if (t_node1->adj_id_ == road2->f_node_id_ ||
                t_node1->adj_id_ == road2->t_node_id_)
                res = true;
        }
    }

    return res;
}

PathEngine::ConnType PathEngine::GetConnectType(MeshManager *mesh_manage,
                               shared_ptr<KDRoad> road1,
                               shared_ptr<KDRoad> road2) {
    if (road1->mesh_id_ == road2->mesh_id_) {

        if (road1->t_node_id_ == road2->f_node_id_)
            return TAIL_HEAD;
        if (road1->t_node_id_ == road2->t_node_id_)
            return TAIL_TAIL;
        if (road1->f_node_id_ == road2->f_node_id_)
            return HEAD_HEAD;
        if (road1->f_node_id_ == road2->t_node_id_)
            return HEAD_TAIL;
    } else {
        shared_ptr<MeshObj> mesh_obj = mesh_manage->GetMesh(road1->mesh_id_);

        shared_ptr<KDRoadNode> f_node1 = mesh_obj->road_nodes_[road1->f_node_id_];
        shared_ptr<KDRoadNode> t_node1 = mesh_obj->road_nodes_[road1->t_node_id_];

        if (f_node1->adj_id_ != -1) {
            if (f_node1->adj_id_ == road2->f_node_id_)
                return HEAD_HEAD;

            if (f_node1->adj_id_ == road2->t_node_id_)
                return HEAD_TAIL;
        }

        if (t_node1->adj_id_ != -1) {
            if (t_node1->adj_id_ == road2->f_node_id_ )
                return TAIL_HEAD;

            if (t_node1->adj_id_ == road2->t_node_id_ )
                return TAIL_TAIL;
        }
    }

    return UN_CONN;
}