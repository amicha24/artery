#ifndef CLUSTERLANESERVICE_H_
#define CLUSTERLANESERVICE_H_

#include "artery/application/ItsG5Service.h"

// forward declaration
namespace traci { class VehicleController; }

class ClusterLaneService : public artery::ItsG5Service
{
    protected:
        void indicate(const vanetza::btp::DataIndication&, omnetpp::cPacket*) override;
        void initialize() override;

    private:
        void slowDown();

        traci::VehicleController* mVehicleController = nullptr;
};

#endif /* CLUSTERLANESERVICE_H_ */
