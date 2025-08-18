#include "artery/application/VehicleClusteringService.h"

#include <vanetza/facilities/cam_functions.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>

using namespace omnetpp;

namespace artery
{

static const simsignal_t scSignalCamReceived = cComponent::registerSignal("CamReceived");
static const simsignal_t scClusterCount = cComponent::registerSignal("ClusterCount");
static const simsignal_t scClusterHead = cComponent::registerSignal("ClusterHead");
static const simsignal_t scClusterSize = cComponent::registerSignal("ClusterSize");

Define_Module(VehicleClusteringService)

VehicleClusteringService::VehicleClusteringService()
{
}

VehicleClusteringService::~VehicleClusteringService()
{
    cancelAndDelete(mTick);
}

void VehicleClusteringService::initialize()
{
    ItsG5Service::initialize();

    const std::string mode = par("mode").stdstringValue();
    mMode = (mode == "speed") ? Mode::Speed : Mode::Spatial;
    mInterval = par("clusteringInterval");
    mEpsilonMeters = par("epsilonRadius");
    mMinPts = par("minPts");
    mSpeedToleranceKmh = par("speedTolerance");
    mIncludeSelf = par("includeSelf");
    mAllowSingletons = par("allowSingletons");
    mHeadStickiness = par("headStickiness");
    mReassignmentCooldown = par("reassignmentCooldown");

    mLdm = &getFacilities().get_const<LocalDynamicMap>();
    mVdp = &getFacilities().get_const<VehicleDataProvider>();

    subscribe(scSignalCamReceived);
    mTick = new cMessage("VehicleClustering tick");
    scheduleAt(simTime() + mInterval, mTick);
}

void VehicleClusteringService::finish()
{
    unsubscribe(scSignalCamReceived);
    ItsG5Service::finish();
}

void VehicleClusteringService::handleMessage(cMessage* msg)
{
    if (msg == mTick) {
        trigger();
        scheduleAt(simTime() + mInterval, mTick);
    } else {
        ItsG5Service::handleMessage(msg);
    }
}

void VehicleClusteringService::trigger()
{
    auto members = collectMembers();
    if (members.empty()) {
        emit(scClusterCount, 0L);
        return;
    }

    std::vector<Cluster> clusters = (mMode == Mode::Spatial) ? clusterSpatial(members) : clusterSpeed(members);
    chooseHeads(clusters);
    emitStats(clusters);
}

void VehicleClusteringService::receiveSignal(cComponent* /*source*/, simsignal_t signal, cObject* /*obj*/, cObject* /*details*/)
{
    if (signal == scSignalCamReceived) {
        // no-op; subscribed to maintain awareness of neighbor updates
    }
}

std::vector<VehicleClusteringService::Member> VehicleClusteringService::collectMembers() const
{
    std::vector<Member> out;
    const SimTime now = simTime();

    if (mIncludeSelf) {
        Member self;
        self.stationId = mVdp->station_id();
        self.lat = mVdp->latitude().value();
        self.lon = mVdp->longitude().value();
        self.speedKmh = mVdp->speed().value() * 3.6;
        self.lastSeen = now;
        out.push_back(self);
    }

    const auto& entries = mLdm->allEntries();
    for (const auto& kv : entries) {
        const auto& cam = kv.second.cam();
        const auto& bc = cam.cam.camParameters.basicContainer;
        const auto& hf = cam.cam.camParameters.highFrequencyContainer;
        Member m;
        m.stationId = cam.header.stationID;
        static constexpr double kMicroDeg = 1e6;
        m.lat = static_cast<double>(bc.referencePosition.latitude) / kMicroDeg;
        m.lon = static_cast<double>(bc.referencePosition.longitude) / kMicroDeg;
        m.speedKmh = vanetza::facilities::speed_value_kmh(hf.basicVehicleContainerHighFrequency.speed);
        m.lastSeen = now;
        out.push_back(m);
    }
    return out;
}

static constexpr double kPi = 3.14159265358979323846;
static constexpr double kHuge = 1e100;
static inline double deg2rad(double d)
{
    return d * kPi / 180.0;
}

double VehicleClusteringService::geoDistanceMeters(double lat1, double lon1, double lat2, double lon2)
{
    static const double R = 6371000.0;
    const double dLat = deg2rad(lat2 - lat1);
    const double dLon = deg2rad(lon2 - lon1);
    const double a = std::sin(dLat / 2) * std::sin(dLat / 2) + std::cos(deg2rad(lat1)) * std::cos(deg2rad(lat2)) * std::sin(dLon / 2) * std::sin(dLon / 2);
    const double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));
    return R * c;
}

std::vector<VehicleClusteringService::Cluster> VehicleClusteringService::clusterSpatial(const std::vector<Member>& mbs)
{
    const double eps = mEpsilonMeters;
    const int minPts = mMinPts;
    const int n = static_cast<int>(mbs.size());
    std::vector<int> labels(n, -1);
    int clusterId = 0;

    auto regionQuery = [&](int i) {
        std::vector<int> neighbors;
        for (int j = 0; j < n; ++j) {
            if (i == j)
                continue;
            if (geoDistanceMeters(mbs[i].lat, mbs[i].lon, mbs[j].lat, mbs[j].lon) <= eps) {
                neighbors.push_back(j);
            }
        }
        return neighbors;
    };

    for (int i = 0; i < n; ++i) {
        if (labels[i] != -1)
            continue;
        auto neighbors = regionQuery(i);
        if (static_cast<int>(neighbors.size()) + 1 < minPts) {
            labels[i] = mAllowSingletons ? clusterId++ : -2;
            continue;
        }
        labels[i] = clusterId;
        std::vector<int> seeds = neighbors;
        for (size_t k = 0; k < seeds.size(); ++k) {
            int p = seeds[k];
            if (labels[p] == -2)
                labels[p] = clusterId;
            if (labels[p] != -1)
                continue;
            labels[p] = clusterId;
            auto n2 = regionQuery(p);
            if (static_cast<int>(n2.size()) + 1 >= minPts) {
                seeds.insert(seeds.end(), n2.begin(), n2.end());
            }
        }
        clusterId++;
    }

    std::vector<Cluster> clusters;
    clusters.resize(clusterId);
    for (int cid = 0; cid < clusterId; ++cid) {
        clusters[cid].id = cid;
    }
    for (int i = 0; i < n; ++i) {
        int cid = labels[i];
        if (cid >= 0)
            clusters[cid].members.push_back(mbs[i]);
    }
    return clusters;
}

double VehicleClusteringService::median(std::vector<double> v)
{
    if (v.empty())
        return 0.0;
    std::sort(v.begin(), v.end());
    size_t m = v.size() / 2;
    if (v.size() % 2)
        return v[m];
    return 0.5 * (v[m - 1] + v[m]);
}

std::vector<VehicleClusteringService::Cluster> VehicleClusteringService::clusterSpeed(const std::vector<Member>& mbs)
{
    std::vector<double> speeds;
    speeds.reserve(mbs.size());
    for (auto& m : mbs)
        speeds.push_back(m.speedKmh);
    double med = median(speeds);
    const double tol = mSpeedToleranceKmh;

    Cluster c;
    c.id = 0;
    for (auto& m : mbs) {
        if (std::abs(m.speedKmh - med) <= tol)
            c.members.push_back(m);
    }
    if (!mAllowSingletons && static_cast<int>(c.members.size()) < mMinPts) {
        return {};
    }
    c.medianSpeedKmh = med;
    return {c};
}

void VehicleClusteringService::chooseHeads(std::vector<Cluster>& clusters)
{
    const SimTime now = simTime();
    for (auto& c : clusters) {
        if (c.members.empty())
            continue;
        if (mMode == Mode::Spatial) {
            double latSum = 0.0, lonSum = 0.0;
            for (auto& m : c.members) {
                latSum += m.lat;
                lonSum += m.lon;
            }
            c.centroidLat = latSum / c.members.size();
            c.centroidLon = lonSum / c.members.size();
            double best = kHuge;
            uint32_t head = 0;
            for (auto& m : c.members) {
                double d = geoDistanceMeters(m.lat, m.lon, c.centroidLat, c.centroidLon);
                if (d < best || (std::abs(d - best) < 1e-6 && m.stationId < head)) {
                    best = d;
                    head = m.stationId;
                }
            }
            c.headStationId = head;
        } else {
            double best = kHuge;
            uint32_t head = 0;
            for (auto& m : c.members) {
                double d = std::abs(m.speedKmh - c.medianSpeedKmh);
                if (d < best || (std::abs(d - best) < 1e-6 && m.stationId < head)) {
                    best = d;
                    head = m.stationId;
                }
            }
            c.headStationId = head;
        }

        if (mHeadStickiness) {
            auto it = mHeads.find(c.id);
            if (it != mHeads.end() && it->second != 0 && now - mLastAssignmentChange < mReassignmentCooldown) {
                c.headStationId = it->second;
            }
            if (mHeads[c.id] != c.headStationId) {
                mHeads[c.id] = c.headStationId;
                mLastAssignmentChange = now;
            }
        } else {
            mHeads[c.id] = c.headStationId;
        }
    }
}

void VehicleClusteringService::emitStats(const std::vector<Cluster>& clusters)
{
    emit(scClusterCount, static_cast<long>(clusters.size()));
    for (const auto& c : clusters) {
        emit(scClusterSize, static_cast<long>(c.members.size()));
        emit(scClusterHead, static_cast<long>(c.headStationId));
    }
}

}  // namespace artery
