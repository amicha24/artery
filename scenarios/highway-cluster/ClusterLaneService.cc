#include "ClusterLaneService.h"
#include "cluster_msgs/Clustering_m.h"
#include "artery/traci/VehicleController.h"

using namespace omnetpp;
using namespace vanetza;

Define_Module(ClusterLaneService)

void ClusterLaneService::initialize()
{
    ItsG5BaseService::initialize();
    mVehicleController = &getFacilities().get_mutable<traci::VehicleController>();
}

void ClusterLaneService::indicate(const vanetza::btp::DataIndication& ind, omnetpp::cPacket* packet)
{
    Enter_Method("ClusterLaneService indicate");
    auto clusterLaneMessage = check_and_cast<const Clustering*>(packet);

    const std::string id = mVehicleController->getVehicleId();
    auto& vehicle_api = mVehicleController->getTraCI()->vehicle;
    if (vehicle_api.getRoadID(id) == clusterLaneMessage->getEdgeName()) {
        if (vehicle_api.getLaneIndex(id) != clusterLaneMessage->getLaneIndex()) {
            slowDown();
        }
    }

    delete clusterLaneMessage;
}

void ClusterLaneService::slowDown()
{
    mVehicleController->setMaxSpeed(5 * units::si::meter_per_second);
}
