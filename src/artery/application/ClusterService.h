/*
* Artery V2X Simulation Framework
* Copyright 2014-2019 Raphael Riebl et al.
* Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
*/

#ifndef ARTERY_CLUSTERSERVICE_H_
#define ARTERY_CLUSTERSERVICE_H_

#include "artery/application/ItsG5Service.h"
#include "artery/application/NetworkInterface.h"

// forward declaration
namespace traci { class VehicleController; }

namespace artery
{

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
        omnetpp::cMessage* m_self_msg;
        traci::VehicleController* mVehicleController = nullptr;
};

} // namespace artery

#endif /* ARTERY_CLUSTERSERVICE_H_ */
