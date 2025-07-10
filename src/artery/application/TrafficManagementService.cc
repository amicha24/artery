/*
 * Artery V2X Simulation Framework
 * Copyright 2025 Devin AI
 * Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
 */

#include "TrafficManagementService.h"
#include "artery/application/LocalDynamicMap.h"
#include "artery/application/Timer.h"
#include "artery/application/VehicleDataProvider.h"
#include "artery/application/SampleBufferAlgorithm.h"
#include "artery/application/Asn1PacketVisitor.h"
#include <vanetza/btp/data_request.hpp>
#include <vanetza/btp/ports.hpp>
#include <vanetza/dcc/profile.hpp>
#include <vanetza/geonet/interface.hpp>
#include <vanetza/facilities/cam_functions.hpp>
#include <vanetza/units/acceleration.hpp>
#include <vanetza/units/time.hpp>
#include <boost/units/base_units/metric/hour.hpp>
#include <boost/units/systems/si/length.hpp>
#include <boost/units/systems/si/time.hpp>
#include <boost/units/systems/si/prefixes.hpp>

static const auto hour = 3600.0 * boost::units::si::seconds;
static const auto km_per_hour = boost::units::si::kilo * boost::units::si::meter / hour;

using namespace omnetpp;
using namespace vanetza;

namespace artery
{

static const simsignal_t scSignalTrafficManagement = cComponent::registerSignal("TrafficManagement");

Define_Module(TrafficManagementService)

TrafficManagementService::TrafficManagementService() :
    mCongestionSpeedThreshold(30.0),
    mHighDensityThreshold(10.0),
    mRecommendedSpeed(50.0),
    mAnalysisInterval(5.0),
    mMessageValidityDuration(60.0),
    mLocalDynamicMap(nullptr),
    mVehicleDataProvider(nullptr),
    mTimer(nullptr),
    mAnalysisTimer(nullptr),
    mCongestionDetected(false),
    mSequenceNumber(0)
{
}

TrafficManagementService::~TrafficManagementService()
{
    cancelAndDelete(mAnalysisTimer);
}

void TrafficManagementService::initialize()
{
    ItsG5BaseService::initialize();
    
    mCongestionSpeedThreshold = par("congestionSpeedThreshold").doubleValue();
    mHighDensityThreshold = par("highDensityThreshold").doubleValue();
    mRecommendedSpeed = par("recommendedSpeed").doubleValue();
    mAnalysisInterval = par("analysisInterval").doubleValue();
    mMessageValidityDuration = par("messageValidityDuration").doubleValue();
    
    mLocalDynamicMap = &getFacilities().get_const<LocalDynamicMap>();
    mVehicleDataProvider = &getFacilities().get_const<VehicleDataProvider>();
    mTimer = &getFacilities().get_const<Timer>();
    
    mSpeedSampler.setDuration(SimTime(30.0));
    mSpeedSampler.setInterval(SimTime(1.0));
    
    mAnalysisTimer = new cMessage("Traffic Analysis");
    scheduleAt(simTime() + mAnalysisInterval, mAnalysisTimer);
    
    subscribe(scSignalTrafficManagement);
}

void TrafficManagementService::finish()
{
    ItsG5BaseService::finish();
}

void TrafficManagementService::handleMessage(cMessage* msg)
{
    Enter_Method("handleMessage");
    
    if (msg == mAnalysisTimer) {
        trigger();
        scheduleAt(simTime() + mAnalysisInterval, mAnalysisTimer);
    }
}

void TrafficManagementService::indicate(const btp::DataIndication& indication, std::unique_ptr<vanetza::UpPacket> packet)
{
    Enter_Method("indicate");
    
    Asn1PacketVisitor<vanetza::asn1::Denm> visitor;
    const vanetza::asn1::Denm* denm = boost::apply_visitor(visitor, *packet);
    
    if (denm) {
        EV_INFO << "Received traffic management DENM message\n";
    }
}

void TrafficManagementService::trigger()
{
    Enter_Method("trigger");
    
    mSpeedSampler.feed(mVehicleDataProvider->speed(), mVehicleDataProvider->updated());
    
    bool congestionNow = detectCongestion();
    bool slowTraffic = detectSlowTraffic();
    
    if (congestionNow && !mCongestionDetected) {
        mCongestionDetected = true;
        mLastCongestionMessage = simTime();
        
        auto message = createCongestionMessage();
        auto request = createTrafficRequest();
        
        sendDenm(std::move(message), request);
        
        EV_INFO << "Traffic congestion detected - broadcasting DENM\n";
    }
    
    if (shouldRecommendSpeedChange()) {
        mLastSpeedRecommendation = simTime();
        
        auto message = createSpeedRecommendationMessage();
        auto request = createTrafficRequest();
        
        sendDenm(std::move(message), request);
        
        EV_INFO << "Broadcasting speed recommendation\n";
    }
    
    if (!congestionNow && mCongestionDetected) {
        mCongestionDetected = false;
        EV_INFO << "Traffic congestion cleared\n";
    }
}

bool TrafficManagementService::detectCongestion()
{
    return checkVehicleDensity() && checkAverageSpeed();
}

bool TrafficManagementService::detectSlowTraffic()
{
    using vanetza::units::Velocity;
    
    const auto& speedSamples = mSpeedSampler.buffer();
    if (speedSamples.empty()) return false;
    
    static const Velocity slowThreshold { mCongestionSpeedThreshold * km_per_hour };
    
    return speedSamples.latest().value < slowThreshold;
}

bool TrafficManagementService::checkVehicleDensity()
{
    using vanetza::facilities::distance;
    using vanetza::units::Length;
    using vanetza::units::si::meter;
    
    const Length analysisRadius { 100.0 * meter };
    
    LocalDynamicMap::CamPredicate nearbyVehicles = [&](const LocalDynamicMap::Cam& msg) {
        const auto& bc = msg->cam.camParameters.basicContainer;
        return distance(bc.referencePosition, 
                       mVehicleDataProvider->latitude(), 
                       mVehicleDataProvider->longitude()) <= analysisRadius;
    };
    
    unsigned vehicleCount = mLocalDynamicMap->count(nearbyVehicles);
    double density = vehicleCount / 1.0;
    
    return density >= mHighDensityThreshold;
}

bool TrafficManagementService::checkAverageSpeed()
{
    using vanetza::units::Velocity;
    
    const auto& speedSamples = mSpeedSampler.buffer();
    if (speedSamples.duration() < SimTime(10.0)) return false;
    
    static const Velocity congestionThreshold { mCongestionSpeedThreshold * km_per_hour };
    const Velocity avgSpeed = average(speedSamples);
    
    return avgSpeed <= congestionThreshold;
}

bool TrafficManagementService::shouldRecommendSpeedChange()
{
    if (simTime() - mLastSpeedRecommendation < SimTime(30.0)) return false;
    
    return detectSlowTraffic() || mCongestionDetected;
}

vanetza::asn1::Denm TrafficManagementService::createCongestionMessage()
{
    auto message = vanetza::asn1::allocate<vanetza::asn1::Denm>();
    
    message->header.protocolVersion = 2;
    message->header.messageID = ItsPduHeader__messageID_denm;
    message->header.stationID = mVehicleDataProvider->station_id();
    
    message->denm.management.actionID.originatingStationID = mVehicleDataProvider->station_id();
    message->denm.management.actionID.sequenceNumber = ++mSequenceNumber;
    message->denm.management.detectionTime = mTimer->getCurrentTime();
    message->denm.management.referenceTime = mTimer->getCurrentTime();
    
    message->denm.management.relevanceDistance = vanetza::asn1::allocate<RelevanceDistance_t>();
    *message->denm.management.relevanceDistance = RelevanceDistance_lessThan1000m;
    message->denm.management.relevanceTrafficDirection = vanetza::asn1::allocate<RelevanceTrafficDirection_t>();
    *message->denm.management.relevanceTrafficDirection = RelevanceTrafficDirection_allTrafficDirections;
    message->denm.management.validityDuration = vanetza::asn1::allocate<ValidityDuration_t>();
    *message->denm.management.validityDuration = static_cast<ValidityDuration_t>(mMessageValidityDuration.dbl());
    message->denm.management.stationType = StationType_unknown;
    
    message->denm.situation = vanetza::asn1::allocate<SituationContainer_t>();
    message->denm.situation->informationQuality = 7;
    message->denm.situation->eventType.causeCode = CauseCodeType_trafficCondition;
    message->denm.situation->eventType.subCauseCode = 1;
    
    return message;
}

vanetza::asn1::Denm TrafficManagementService::createSpeedRecommendationMessage()
{
    auto message = vanetza::asn1::allocate<vanetza::asn1::Denm>();
    
    message->header.protocolVersion = 2;
    message->header.messageID = ItsPduHeader__messageID_denm;
    message->header.stationID = mVehicleDataProvider->station_id();
    
    message->denm.management.actionID.originatingStationID = mVehicleDataProvider->station_id();
    message->denm.management.actionID.sequenceNumber = ++mSequenceNumber;
    message->denm.management.detectionTime = mTimer->getCurrentTime();
    message->denm.management.referenceTime = mTimer->getCurrentTime();
    
    message->denm.management.relevanceDistance = vanetza::asn1::allocate<RelevanceDistance_t>();
    *message->denm.management.relevanceDistance = RelevanceDistance_lessThan500m;
    message->denm.management.relevanceTrafficDirection = vanetza::asn1::allocate<RelevanceTrafficDirection_t>();
    *message->denm.management.relevanceTrafficDirection = RelevanceTrafficDirection_allTrafficDirections;
    message->denm.management.validityDuration = vanetza::asn1::allocate<ValidityDuration_t>();
    *message->denm.management.validityDuration = 30;
    message->denm.management.stationType = StationType_unknown;
    
    message->denm.situation = vanetza::asn1::allocate<SituationContainer_t>();
    message->denm.situation->informationQuality = 5;
    message->denm.situation->eventType.causeCode = CauseCodeType_trafficCondition;
    message->denm.situation->eventType.subCauseCode = 2;
    
    return message;
}

vanetza::btp::DataRequestB TrafficManagementService::createTrafficRequest()
{
    namespace geonet = vanetza::geonet;
    using vanetza::units::si::seconds;
    using vanetza::units::si::meter;

    vanetza::btp::DataRequestB request;
    request.destination_port = btp::ports::DENM;
    request.gn.traffic_class.tc_id(static_cast<unsigned>(dcc::Profile::DP2));
    request.gn.transport_type = geonet::TransportType::GBC;
    request.gn.communication_profile = geonet::CommunicationProfile::ITS_G5;
    request.gn.its_aid = aid::DEN;

    geonet::Area destination;
    geonet::Circle destination_shape;
    destination_shape.r = 500.0 * meter;
    destination.shape = destination_shape;
    destination.position.latitude = mVehicleDataProvider->latitude();
    destination.position.longitude = mVehicleDataProvider->longitude();
    request.gn.destination = destination;

    return request;
}

void TrafficManagementService::sendDenm(vanetza::asn1::Denm&& message, vanetza::btp::DataRequestB& request)
{
    using namespace vanetza;
    using DenmConvertible = vanetza::convertible::byte_buffer_impl<vanetza::asn1::Denm>;
    std::unique_ptr<geonet::DownPacket> payload { new geonet::DownPacket };
    std::unique_ptr<vanetza::convertible::byte_buffer> denm { new DenmConvertible { std::make_shared<vanetza::asn1::Denm>(std::move(message)) } };
    payload->layer(OsiLayer::Application) = vanetza::ByteBufferConvertible { std::move(denm) };
    this->request(request, std::move(payload));
}

void TrafficManagementService::receiveSignal(cComponent* source, simsignal_t signal, cObject*, cObject*)
{
    Enter_Method("receiveSignal");
    
    if (signal == scSignalTrafficManagement) {
        EV_INFO << "Received traffic management signal\n";
    }
}

} // namespace artery
