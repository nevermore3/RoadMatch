//
// Created by Liu Jian on 2019/1/16.
//

#ifndef ROAD_MATCH_SERVICE_PATH_ENGINE_H
#define ROAD_MATCH_SERVICE_PATH_ENGINE_H

#include "data_manager/data_types.h"
#include "data_manager/mesh_manager.h"

#include <list>
#include <queue>

class SearchLink {
public:
    SearchLink() {};
    ~SearchLink() {};

    int32_t get_length() const { return length_; }
    void set_length(int32_t length) { length_ = length;}

    int32_t get_weight() const { return weight_; }
    void set_weight(int32_t weight) { weight_ = weight;}

    shared_ptr<KDRoad> get_kdroad() { return link_; }
    void set_kdroad(shared_ptr<KDRoad> link) { link_ = link; }

    shared_ptr<KDRoadNode> get_kdnode() { return node_; }
    void set_kdnode(shared_ptr<KDRoadNode> node) { node_ = node; }

    shared_ptr<SearchLink> get_parent() { return parent_; }
    void set_parent(shared_ptr<SearchLink> parent) { parent_ = parent; }

    bool operator < (const SearchLink& other) {
        return get_length() > other.get_length();
    }

private:
    int32_t length_;
    int32_t weight_;
    shared_ptr<KDRoad> link_;
    shared_ptr<KDRoadNode> node_;
    shared_ptr<SearchLink> parent_;
};

class LengthComp{
public:
    bool operator ()(shared_ptr<SearchLink> &link1,shared_ptr<SearchLink> &link2){
        return link1->get_weight() > link2->get_weight();//最小值优先
    }
};

class AttrLength {
public:
    string mesh_id_;
    int64_t id_;
    double length_;
};

class FunctionBufferFilter{
public:
    static FunctionBufferFilter* GetInstance(){
        static FunctionBufferFilter fun_buf_filter;
        return & fun_buf_filter;
    }
    bool CheckRoadValid(int fun,double dis);
protected:
    FunctionBufferFilter();
private:
    map<int,double> func_buffer_map_;
};

class PathEngine {
public:
    PathEngine();

    enum ConnType {
        HEAD_HEAD = 0,
        HEAD_TAIL = 1,
        TAIL_TAIL = 2,
        TAIL_HEAD = 3,
        UN_CONN   = 4
    };


    void SetSearchCount(int count);

    void SetFunctionClassFilterValid(bool valid);

    bool Init(MeshManager *mesh_manage, shared_ptr<KDRoad> road_src,
              shared_ptr<KDRoad> road_dst);

    bool FindPath(MeshManager *mesh_manage, std::list<shared_ptr<KDRoad>> &result);

    bool FindPath(MeshManager *mesh_manage,
                  shared_ptr<KDRoad> road_src,
                  shared_ptr<KDCoord> src_coord,
                  shared_ptr<KDRoad> road_dst,
                  shared_ptr<KDCoord> des_coord,
                  std::list<shared_ptr<KDRoad>> &result);

    static bool IsRoadConnect(MeshManager *mesh_manage,
                              shared_ptr<KDRoad> road1,
                              shared_ptr<KDRoad> road2);

    static ConnType GetConnectType(MeshManager *mesh_manage,
                                  shared_ptr<KDRoad> road1,
                                  shared_ptr<KDRoad> road2);

private:
    bool SetSource(MeshManager *mesh_manage, shared_ptr<KDRoad> road_src, shared_ptr<KDRoadNode> node);

    void ExtendPath(MeshManager *mesh_manage, shared_ptr<SearchLink> link, bool forward);

    bool Recall(shared_ptr<SearchLink> link_,
                std::list<shared_ptr<KDRoad>> &result);

    double GetFilterQueryDistance(const KDCoord &node);

    void Clear();

private:
    bool fc_valid_;
    int search_count_;
    shared_ptr<KDRoad> des_link_;
    shared_ptr<KDCoord> des_coord_;
    shared_ptr<KDRoad> src_link_;
    shared_ptr<KDCoord> src_coord_;
    shared_ptr<KDRoad> match_link_;
    priority_queue<shared_ptr<SearchLink>, std::vector<shared_ptr<SearchLink>>, LengthComp> path_heap_;
    std::map<string, shared_ptr<SearchLink>> path_close_map_;
    std::map<string, shared_ptr<SearchLink>> path_open_map_;


};

#endif //ROAD_MATCH_SERVICE_PATH_ENGINE_H
