/*
 * Artery V2X Simulation Framework
 * Copyright 2025 Devin AI
 * Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
 */

#ifndef ARTERY_TRAFFICMANAGEMENTSERVICE_H_
#define ARTERY_TRAFFICMANAGEMENTSERVICE_H_

#include "artery/application/ItsG5BaseService.h"
#include "artery/application/Sampling.h"
#include <vanetza/units/velocity.hpp>
#include <vanetza/asn1/denm.hpp>
#include <vanetza/btp/data_request.hpp>
#include <omnetpp/simtime.h>
#include <memory>

namespace artery
{

class LocalDynamicMap;
class Timer;
class VehicleDataProvider;

class TrafficManagementService : public ItsG5BaseService
{
public:
    TrafficManagementService();
    ~TrafficManagementService();

    void indicate(const vanetza::btp::DataIndication&, std::unique_ptr<vanetza::UpPacket>) override;
    void trigger() override;
    void receiveSignal(omnetpp::cComponent*, omnetpp::simsignal_t, omnetpp::cObject*, omnetpp::cObject*) override;

protected:
    void initialize() override;
    void finish() override;
    void handleMessage(omnetpp::cMessage*) override;

private:
private:
    bool detectCongestion();
    bool detectSlowTraffic();
    bool shouldRecommendSpeedChange();
    
    vanetza::asn1::Denm createCongestionMessage();
    vanetza::asn1::Denm createSpeedRecommendationMessage();
    vanetza::btp::DataRequestB createTrafficRequest();
    void sendDenm(vanetza::asn1::Denm&& message, vanetza::btp::DataRequestB& request);
    
    bool checkVehicleDensity();
    bool checkAverageSpeed();
    double mCongestionSpeedThreshold;
    double mHighDensityThreshold;
    double mRecommendedSpeed;
    omnetpp::SimTime mAnalysisInterval;
    omnetpp::SimTime mMessageValidityDuration;
    
    const LocalDynamicMap* mLocalDynamicMap;
    const VehicleDataProvider* mVehicleDataProvider;
    const Timer* mTimer;
    SkipEarlySampler<vanetza::units::Velocity> mSpeedSampler;
    
    omnetpp::cMessage* mAnalysisTimer;
    bool mCongestionDetected;
    omnetpp::SimTime mLastCongestionMessage;
    omnetpp::SimTime mLastSpeedRecommendation;
    unsigned mSequenceNumber;
};

} // namespace artery

#endif /* ARTERY_TRAFFICMANAGEMENTSERVICE_H_ */
