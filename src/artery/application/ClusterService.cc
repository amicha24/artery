/*
* Artery V2X Simulation Framework
* Copyright 2014-2019 Raphael Riebl et al.
* Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
*/

#include "ClusterService.h"
#include "ClusteringMessage_m.h"
#include "artery/application/LocalDynamicMap.h"
#include "artery/application/Timer.h"
#include "artery/application/VehicleDataProvider.h"
#include "artery/traci/VehicleController.h"
#include <omnetpp/cpacket.h>
#include <vanetza/btp/data_request.hpp>
#include <vanetza/dcc/profile.hpp>
#include <vanetza/geonet/interface.hpp>
#include <cmath>
#include <boost/units/systems/si/length.hpp>
#include <boost/units/systems/si/velocity.hpp>
#include <boost/units/systems/si/plane_angle.hpp>

using namespace omnetpp;
using namespace vanetza;

namespace artery
{

static const simsignal_t scSignalCamReceived = cComponent::registerSignal("CamReceived");
static const simsignal_t scSignalClusterFormed = cComponent::registerSignal("ClusterFormed");
static const simsignal_t scSignalClusterJoined = cComponent::registerSignal("ClusterJoined");

Define_Module(ClusterService)

ClusterService::ClusterService() :
    mClusterRange(100.0 * boost::units::si::meter),
    mMaxSpeedDifference(10.0 * boost::units::si::meter_per_second),
    mMaxHeadingDifference(30.0 * boost::units::si::radian),
    mMaintenanceInterval(2.0),
    mMemberTimeout(5.0),
    mRole(ClusterRole::UNCLUSTERED),
    mClusterId(0),
    mClusterHeadId(0),
    mLocalDynamicMap(nullptr),
    mVehicleDataProvider(nullptr),
    mTimer(nullptr),
    mVehicleController(nullptr),
    mMaintenanceTimer(nullptr),
    mAdvertisementTimer(nullptr)
{
}

ClusterService::~ClusterService()
{
    cancelAndDelete(mMaintenanceTimer);
    cancelAndDelete(mAdvertisementTimer);
}

void ClusterService::initialize()
{
    ItsG5Service::initialize();
    
    mClusterRange = par("clusterRange").doubleValue() * boost::units::si::meter;
    mMaxSpeedDifference = par("maxSpeedDifference").doubleValue() * boost::units::si::meter_per_second;
    mMaxHeadingDifference = par("maxHeadingDifference").doubleValue() * boost::units::si::radian;
    mMaintenanceInterval = par("maintenanceInterval").doubleValue();
    mMemberTimeout = par("memberTimeout").doubleValue();
    
    mLocalDynamicMap = &getFacilities().get_const<LocalDynamicMap>();
    mVehicleDataProvider = &getFacilities().get_const<VehicleDataProvider>();
    mTimer = &getFacilities().get_const<Timer>();
    mVehicleController = &getFacilities().get_const<traci::VehicleController>();
    
    mMaintenanceTimer = new cMessage("Cluster Maintenance");
    mAdvertisementTimer = new cMessage("Cluster Advertisement");
    
    scheduleAt(simTime() + mMaintenanceInterval, mMaintenanceTimer);
    
    subscribe(scSignalCamReceived);
    subscribe(scSignalClusterFormed);
    subscribe(scSignalClusterJoined);
}

void ClusterService::finish()
{
    ItsG5Service::finish();
}

void ClusterService::handleMessage(cMessage* msg)
{
    Enter_Method("handleMessage");
    
    if (msg == mMaintenanceTimer) {
        performClusterMaintenance();
        scheduleAt(simTime() + mMaintenanceInterval, mMaintenanceTimer);
    } else if (msg == mAdvertisementTimer) {
        if (mRole == ClusterRole::CLUSTER_HEAD) {
            sendClusterAdvertisement();
        }
        scheduleAt(simTime() + 1.0, mAdvertisementTimer);
    }
}

void ClusterService::indicate(const btp::DataIndication& ind, cPacket* packet, const NetworkInterface& net)
{
    Enter_Method("indicate");
    
    auto clusterMessage = dynamic_cast<ClusterMessage*>(packet);
    if (clusterMessage) {
        processClusterMessage(clusterMessage);
    }
    
    delete(packet);
}

void ClusterService::trigger()
{
    Enter_Method("trigger");
}

void ClusterService::performClusterMaintenance()
{
    Enter_Method("performClusterMaintenance");
    
    removeExpiredMembers();
    
    switch (mRole) {
        case ClusterRole::UNCLUSTERED:
            if (shouldBecomeClusterHead()) {
                formCluster();
            } else {
                auto nearbyVehicles = findNearbyVehicles();
                if (!nearbyVehicles.empty()) {
                    uint32_t clusterHead = selectClusterHead(nearbyVehicles);
                    if (clusterHead != 0) {
                        joinCluster(clusterHead);
                    }
                }
            }
            break;
            
        case ClusterRole::CLUSTER_HEAD:
            if (!shouldBecomeClusterHead()) {
                leaveCluster();
            } else {
                if (!mAdvertisementTimer->isScheduled()) {
                    scheduleAt(simTime() + 0.5, mAdvertisementTimer);
                }
            }
            break;
            
        case ClusterRole::CLUSTER_MEMBER:
            auto clusterHeadCam = mLocalDynamicMap->getCam(mClusterHeadId);
            if (!clusterHeadCam || !isVehicleInClusterRange(*clusterHeadCam)) {
                leaveCluster();
            }
            break;
    }
    
    mLastMaintenance = simTime();
}

bool ClusterService::shouldBecomeClusterHead()
{
    auto nearbyVehicles = findNearbyVehicles();
    
    uint32_t myStationId = mVehicleDataProvider->getStationId();
    for (uint32_t stationId : nearbyVehicles) {
        if (stationId > myStationId) {
            return false;
        }
    }
    
    return !nearbyVehicles.empty();
}

std::vector<uint32_t> ClusterService::findNearbyVehicles()
{
    std::vector<uint32_t> nearbyVehicles;
    
    LocalDynamicMap::CamPredicate nearbyPredicate = [&](const LocalDynamicMap::Cam& cam) {
        return isEligibleForClustering(cam);
    };
    
    
    return nearbyVehicles;
}

bool ClusterService::isEligibleForClustering(const LocalDynamicMap::Cam& cam)
{
    const auto& bc = cam->cam.camParameters.basicContainer;
    const auto& hfc = cam->cam.camParameters.highFrequencyContainer;
    
    if (hfc.present != HighFrequencyContainer_PR_basicVehicleContainerHighFrequency) {
        return false;
    }
    
    const auto& bvc = hfc.choice.basicVehicleContainerHighFrequency;
    const auto& vdp = *mVehicleDataProvider;
    
    double lat1 = bc.referencePosition.latitude / 10000000.0;
    double lon1 = bc.referencePosition.longitude / 10000000.0;
    double lat2 = vdp.latitude().value();
    double lon2 = vdp.longitude().value();
    
    double dx = (lon2 - lon1) * 111320.0;
    double dy = (lat2 - lat1) * 110540.0;
    double distance = sqrt(dx*dx + dy*dy);
    
    if (distance > mClusterRange.value()) {
        return false;
    }
    
    if (bvc.speed.speedValue != SpeedValue_unavailable) {
        vanetza::units::Velocity otherSpeed = bvc.speed.speedValue * 0.01 * boost::units::si::meter_per_second;
        if (abs(otherSpeed - vdp.speed()) > mMaxSpeedDifference) {
            return false;
        }
    }
    
    double heading1 = bvc.heading.headingValue * 0.1;
    double heading2 = vdp.heading().value();
    double headingDiff = abs(heading1 - heading2);
    if (headingDiff > 180.0) {
        headingDiff = 360.0 - headingDiff;
    }
    
    if (headingDiff > mMaxHeadingDifference.value() * 180.0 / 3.14159) {
        return false;
    }
    
    return true;
}

void ClusterService::formCluster()
{
    Enter_Method("formCluster");
    
    mRole = ClusterRole::CLUSTER_HEAD;
    mClusterId = mVehicleDataProvider->getStationId();
    mClusterHeadId = mVehicleDataProvider->getStationId();
    mClusterMembers.clear();
    
    EV_INFO << "Vehicle " << mVehicleDataProvider->getStationId() << " formed cluster " << mClusterId << "\n";
    emit(scSignalClusterFormed, mClusterId);
    
    scheduleAt(simTime() + 0.1, mAdvertisementTimer);
}

void ClusterService::joinCluster(uint32_t clusterHeadId)
{
    Enter_Method("joinCluster");
    
    mRole = ClusterRole::CLUSTER_MEMBER;
    mClusterHeadId = clusterHeadId;
    mClusterId = clusterHeadId;
    
    EV_INFO << "Vehicle " << mVehicleDataProvider->getStationId() << " joined cluster " << mClusterId << "\n";
    emit(scSignalClusterJoined, mClusterId);
    
    sendJoinRequest(clusterHeadId);
}

void ClusterService::leaveCluster()
{
    Enter_Method("leaveCluster");
    
    if (mRole != ClusterRole::UNCLUSTERED) {
        EV_INFO << "Vehicle " << mVehicleDataProvider->getStationId() << " left cluster " << mClusterId << "\n";
        
        if (mRole == ClusterRole::CLUSTER_MEMBER) {
            sendLeaveNotification();
        }
        
        mRole = ClusterRole::UNCLUSTERED;
        mClusterId = 0;
        mClusterHeadId = 0;
        mClusterMembers.clear();
        
        cancelEvent(mAdvertisementTimer);
    }
}

uint32_t ClusterService::selectClusterHead(const std::vector<uint32_t>& candidates)
{
    uint32_t bestCandidate = 0;
    
    for (uint32_t stationId : candidates) {
        auto cam = mLocalDynamicMap->getCam(stationId);
        if (cam && isVehicleInClusterRange(*cam)) {
            if (stationId > bestCandidate) {
                bestCandidate = stationId;
            }
        }
    }
    
    return bestCandidate;
}

bool ClusterService::isVehicleInClusterRange(const LocalDynamicMap::Cam& cam)
{
    const auto& bc = cam->cam.camParameters.basicContainer;
    const auto& vdp = *mVehicleDataProvider;
    
    double lat1 = bc.referencePosition.latitude / 10000000.0;
    double lon1 = bc.referencePosition.longitude / 10000000.0;
    double lat2 = vdp.latitude().value();
    double lon2 = vdp.longitude().value();
    
    double dx = (lon2 - lon1) * 111320.0;
    double dy = (lat2 - lat1) * 110540.0;
    double distance = sqrt(dx*dx + dy*dy);
    
    return distance <= mClusterRange.value();
}

void ClusterService::sendClusterAdvertisement()
{
    Enter_Method("sendClusterAdvertisement");
    
    auto message = new ClusterMessage("Cluster Advertisement");
    message->setClusterId(mClusterId);
    message->setMemberId(mVehicleDataProvider->getStationId());
    message->setPositionX(mVehicleDataProvider->position().x.value());
    message->setPositionY(mVehicleDataProvider->position().y.value());
    message->setPositionZ(0.0);
    message->setTimestamp(simTime().dbl());
    message->setStatus("cluster_head");
    message->setByteLength(64);
    
    btp::DataRequestB req;
    req.destination_port = host_cast(getPortNumber(vanetza::geonet::ChannelNumber::CCH));
    req.gn.transport_type = geonet::TransportType::GBC;
    req.gn.traffic_class.tc_id(static_cast<unsigned>(dcc::Profile::DP2));
    req.gn.communication_profile = geonet::CommunicationProfile::ITS_G5;
    req.gn.its_aid = 16480;
    
    geonet::Area destination;
    geonet::Circle destination_shape;
    destination_shape.r = mClusterRange;
    destination.shape = destination_shape;
    destination.position.latitude = mVehicleDataProvider->latitude();
    destination.position.longitude = mVehicleDataProvider->longitude();
    req.gn.destination = destination;
    
    auto& networks = getFacilities().get_const<NetworkInterfaceTable>();
    auto network = networks.select(vanetza::geonet::ChannelNumber::CCH);
    if (network) {
        request(req, message, network.get());
    }
}

void ClusterService::sendJoinRequest(uint32_t clusterHeadId)
{
    Enter_Method("sendJoinRequest");
    
    auto message = new ClusterMessage("Cluster Join Request");
    message->setClusterId(clusterHeadId);
    message->setMemberId(mVehicleDataProvider->getStationId());
    message->setPositionX(mVehicleDataProvider->position().x.value());
    message->setPositionY(mVehicleDataProvider->position().y.value());
    message->setPositionZ(0.0);
    message->setTimestamp(simTime().dbl());
    message->setStatus("join_request");
    message->setByteLength(64);
    
    btp::DataRequestB req;
    req.destination_port = host_cast(getPortNumber(vanetza::geonet::ChannelNumber::CCH));
    req.gn.transport_type = geonet::TransportType::GBC;
    req.gn.traffic_class.tc_id(static_cast<unsigned>(dcc::Profile::DP2));
    req.gn.communication_profile = geonet::CommunicationProfile::ITS_G5;
    req.gn.its_aid = 16480;
    
    geonet::Area destination;
    geonet::Circle destination_shape;
    destination_shape.r = mClusterRange;
    destination.shape = destination_shape;
    destination.position.latitude = mVehicleDataProvider->latitude();
    destination.position.longitude = mVehicleDataProvider->longitude();
    req.gn.destination = destination;
    
    auto& networks = getFacilities().get_const<NetworkInterfaceTable>();
    auto network = networks.select(vanetza::geonet::ChannelNumber::CCH);
    if (network) {
        request(req, message, network.get());
    }
}

void ClusterService::sendLeaveNotification()
{
    Enter_Method("sendLeaveNotification");
    
    auto message = new ClusterMessage("Cluster Leave Notification");
    message->setClusterId(mClusterId);
    message->setMemberId(mVehicleDataProvider->getStationId());
    message->setPositionX(mVehicleDataProvider->position().x.value());
    message->setPositionY(mVehicleDataProvider->position().y.value());
    message->setPositionZ(0.0);
    message->setTimestamp(simTime().dbl());
    message->setStatus("leave");
    message->setByteLength(64);
    
    btp::DataRequestB req;
    req.destination_port = host_cast(getPortNumber(vanetza::geonet::ChannelNumber::CCH));
    req.gn.transport_type = geonet::TransportType::GBC;
    req.gn.traffic_class.tc_id(static_cast<unsigned>(dcc::Profile::DP2));
    req.gn.communication_profile = geonet::CommunicationProfile::ITS_G5;
    req.gn.its_aid = 16480;
    
    geonet::Area destination;
    geonet::Circle destination_shape;
    destination_shape.r = mClusterRange;
    destination.shape = destination_shape;
    destination.position.latitude = mVehicleDataProvider->latitude();
    destination.position.longitude = mVehicleDataProvider->longitude();
    req.gn.destination = destination;
    
    auto& networks = getFacilities().get_const<NetworkInterfaceTable>();
    auto network = networks.select(vanetza::geonet::ChannelNumber::CCH);
    if (network) {
        request(req, message, network.get());
    }
}

void ClusterService::processClusterMessage(cPacket* packet)
{
    Enter_Method("processClusterMessage");
    
    auto clusterMessage = dynamic_cast<ClusterMessage*>(packet);
    if (!clusterMessage) return;
    
    uint32_t senderId = clusterMessage->getMemberId();
    uint32_t clusterId = clusterMessage->getClusterId();
    std::string status = clusterMessage->getStatus();
    
    EV_INFO << "Received cluster message from " << senderId << " status: " << status << "\n";
    
    if (status == "cluster_head" && mRole == ClusterRole::UNCLUSTERED) {
        auto senderCam = mLocalDynamicMap->getCam(senderId);
        if (senderCam && isEligibleForClustering(*senderCam)) {
            joinCluster(senderId);
        }
    } else if (status == "join_request" && mRole == ClusterRole::CLUSTER_HEAD) {
        auto senderCam = mLocalDynamicMap->getCam(senderId);
        if (senderCam && isEligibleForClustering(*senderCam)) {
            updateClusterMember(senderId, *senderCam);
            EV_INFO << "Added member " << senderId << " to cluster " << mClusterId << "\n";
        }
    } else if (status == "leave" && mRole == ClusterRole::CLUSTER_HEAD) {
        mClusterMembers.erase(senderId);
        EV_INFO << "Removed member " << senderId << " from cluster " << mClusterId << "\n";
    }
}

void ClusterService::updateClusterMember(uint32_t stationId, const LocalDynamicMap::Cam& cam)
{
    ClusterMember member;
    member.stationId = stationId;
    member.lastSeen = simTime();
    
    const auto& bc = cam->cam.camParameters.basicContainer;
    const auto& hfc = cam->cam.camParameters.highFrequencyContainer;
    
    if (hfc.present == HighFrequencyContainer_PR_basicVehicleContainerHighFrequency) {
        const auto& bvc = hfc.choice.basicVehicleContainerHighFrequency;
        member.speed = bvc.speed.speedValue * 0.01 * boost::units::si::meter_per_second;
        member.heading = bvc.heading.headingValue * 0.1 * boost::units::si::radian;
    }
    
    member.position = Position(bc.referencePosition.longitude / 10000000.0, 
                              bc.referencePosition.latitude / 10000000.0);
    
    mClusterMembers[stationId] = member;
}

void ClusterService::removeExpiredMembers()
{
    auto now = simTime();
    for (auto it = mClusterMembers.begin(); it != mClusterMembers.end();) {
        if (now - it->second.lastSeen > mMemberTimeout) {
            EV_INFO << "Removing expired member " << it->first << " from cluster\n";
            it = mClusterMembers.erase(it);
        } else {
            ++it;
        }
    }
}

void ClusterService::receiveSignal(cComponent* source, simsignal_t signal, cObject*, cObject*)
{
    Enter_Method("receiveSignal");
    
    if (signal == scSignalCamReceived) {
    }
}

} // namespace artery
