//
// Created by liujian on 19-7-10.
//

#include "diff_controller.h"

#include "data_manager/data_types.h"
#include "data_manager/data_manager.h"
#include "data_manager/mesh_manager.h"

#include "pathengine/path_engine.h"

bool DiffController::Differing() {
    DataManager data_manager;
    data_manager.extent_ =  KDExtent(115.384, 117.5, 39.322, 41.2);
    data_manager.base_data_manager_ = MeshManager::GetInstance();

    if(!data_manager.LoadData()) {
        return false;
    }

    shared_ptr<KDRoad> road_src = data_manager.base_data_manager_->GetRoad("J50F005020", 1887);
    shared_ptr<KDCoord> src_coord = make_shared<KDCoord>();
    src_coord->lng_ =  116.40222222222;
    src_coord->lat_ =  39.640555555556;

    shared_ptr<KDRoad> road_dst = data_manager.base_data_manager_->GetRoad("J50F004019", 12888);
    shared_ptr<KDCoord> des_coord = make_shared<KDCoord>();
    des_coord->lng_ =  116.36333333333;
    des_coord->lat_ =  39.721944444444;

    PathEngine engine;
    std::list<shared_ptr<KDRoad>> result;
    engine.FindPath((MeshManager*)data_manager.base_data_manager_, road_src, src_coord, road_dst, des_coord, result);

    cout<<"result: "<<result.size()<<endl;

//    bool FindPath(MeshManage *mesh_manage,
//                  shared_ptr<KDRoad> road_src,
//                  shared_ptr<KDCoord> src_coord,
//                  shared_ptr<KDRoad> road_dst,
//                  shared_ptr<KDCoord> des_coord,
//                  std::list<shared_ptr<KDRoad>> &result);
    return true;
}