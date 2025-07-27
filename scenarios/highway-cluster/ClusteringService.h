#ifndef CLUSTERINGSERVICE_H_
#define CLUSTERINGSERVICE_H_

#include "artery/application/ItsG5Service.h"

// forward declaration
namespace traci { class VehicleController; }

class ClusteringService : public artery::ItsG5Service
{
    public:
        void trigger() override;

    protected:
        void initialize() override;
        const traci::VehicleController* mVehicleController = nullptr;
};

#endif /* CLUSTERINGSERVICE_H_ */
