#ifndef ARTERY_VEHICLECLUSTERINGSERVICE_H_
#define ARTERY_VEHICLECLUSTERINGSERVICE_H_

#include "artery/application/ItsG5Service.h"
#include "artery/application/LocalDynamicMap.h"
#include "artery/application/VehicleDataProvider.h"
#include "artery/application/Timer.h"
#include <omnetpp/cmessage.h>
#include <unordered_map>
#include <vector>

namespace artery
{

class VehicleClusteringService : public ItsG5Service
{
    public:
        VehicleClusteringService();
        ~VehicleClusteringService() override;

    protected:
        void initialize() override;
        void finish() override;
        void handleMessage(omnetpp::cMessage*) override;
        void trigger() override;
        void receiveSignal(omnetpp::cComponent*, omnetpp::simsignal_t, omnetpp::cObject*, omnetpp::cObject*) override;

    private:
        enum class Mode { Spatial, Speed };

        struct Member {
            uint32_t stationId;
            double lat;
            double lon;
            double speedKmh;
            omnetpp::SimTime lastSeen;
        };
        struct Cluster {
            int id;
            std::vector<Member> members;
            uint32_t headStationId = 0;
            double centroidLat = 0.0;
            double centroidLon = 0.0;
            double medianSpeedKmh = 0.0;
        };

        Mode mMode = Mode::Spatial;
        omnetpp::SimTime mInterval;
        double mEpsilonMeters = 50.0;
        int mMinPts = 3;
        double mSpeedToleranceKmh = 5.0;
        omnetpp::SimTime mWindowSeconds = 1.0;
        bool mIncludeSelf = true;
        bool mAllowSingletons = false;
        bool mHeadStickiness = true;
        omnetpp::SimTime mReassignmentCooldown = 2.0;

        const LocalDynamicMap* mLdm = nullptr;
        const VehicleDataProvider* mVdp = nullptr;
        const Timer* mTimer = nullptr;

        omnetpp::cMessage* mTick = nullptr;
        bool mDirty = false;
        std::unordered_map<int, uint32_t> mHeads;
        omnetpp::SimTime mLastAssignmentChange = omnetpp::SimTime::ZERO;

        std::vector<Member> collectMembers() const;
        std::vector<Cluster> clusterSpatial(const std::vector<Member>&);
        std::vector<Cluster> clusterSpeed(const std::vector<Member>&);
        static double geoDistanceMeters(double lat1, double lon1, double lat2, double lon2);
        static double median(std::vector<double> v);
        void chooseHeads(std::vector<Cluster>&);
        void emitStats(const std::vector<Cluster>&);
};

} // namespace artery

#endif /* ARTERY_VEHICLECLUSTERINGSERVICE_H_ */
