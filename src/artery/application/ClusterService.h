/*
* Artery V2X Simulation Framework
* Copyright 2014-2019 Raphael Riebl et al.
* Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
*/

#ifndef ARTERY_CLUSTERSERVICE_H_
#define ARTERY_CLUSTERSERVICE_H_

#include "artery/application/ItsG5Service.h"
#include "artery/application/NetworkInterface.h"
#include "artery/application/Sampling.h"
#include "artery/utility/Geometry.h"
#include <vanetza/units/velocity.hpp>
#include <vanetza/units/angle.hpp>
#include <vanetza/units/length.hpp>
#include <omnetpp/simtime.h>
#include <set>
#include <map>

// forward declarations
namespace traci { class VehicleController; }

namespace artery
{

class LocalDynamicMap;
class Timer;
class VehicleDataProvider;

enum class ClusterRole {
    UNCLUSTERED,
    CLUSTER_HEAD,
    CLUSTER_MEMBER
};

struct ClusterMember {
    uint32_t stationId;
    omnetpp::SimTime lastSeen;
    vanetza::units::Velocity speed;
    vanetza::units::Angle heading;
    Position position;
};

class ClusterService : public ItsG5Service
{
    public:
        ClusterService();
        ~ClusterService();

        void indicate(const vanetza::btp::DataIndication&, omnetpp::cPacket*, const NetworkInterface&) override;
        void trigger() override;
        void receiveSignal(omnetpp::cComponent*, omnetpp::simsignal_t, omnetpp::cObject*, omnetpp::cObject*) override;

    protected:
        void initialize() override;
        void finish() override;
        void handleMessage(omnetpp::cMessage*) override;

    private:
        void performClusterMaintenance();
        void formCluster();
        void joinCluster(uint32_t clusterHeadId);
        void leaveCluster();
        bool shouldBecomeClusterHead();
        bool isEligibleForClustering(const LocalDynamicMap::Cam& cam);
        
        void processClusterMessage(omnetpp::cPacket* packet);
        void sendClusterAdvertisement();
        void sendJoinRequest(uint32_t clusterHeadId);
        void sendLeaveNotification();
        
        std::vector<uint32_t> findNearbyVehicles();
        bool isVehicleInClusterRange(const LocalDynamicMap::Cam& cam);
        uint32_t selectClusterHead(const std::vector<uint32_t>& candidates);
        void updateClusterMember(uint32_t stationId, const LocalDynamicMap::Cam& cam);
        void removeExpiredMembers();
        
        vanetza::units::Length mClusterRange;
        vanetza::units::Velocity mMaxSpeedDifference;
        vanetza::units::Angle mMaxHeadingDifference;
        omnetpp::SimTime mMaintenanceInterval;
        omnetpp::SimTime mMemberTimeout;
        
        ClusterRole mRole;
        uint32_t mClusterId;
        uint32_t mClusterHeadId;
        std::map<uint32_t, ClusterMember> mClusterMembers;
        omnetpp::SimTime mLastMaintenance;
        
        const LocalDynamicMap* mLocalDynamicMap;
        const VehicleDataProvider* mVehicleDataProvider;
        const Timer* mTimer;
        traci::VehicleController* mVehicleController;
        
        omnetpp::cMessage* mMaintenanceTimer;
        omnetpp::cMessage* mAdvertisementTimer;
};

} // namespace artery

#endif /* ARTERY_CLUSTERSERVICE_H_ */
