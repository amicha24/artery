package artery.application;

import artery.application.ItsG5Service;

simple VehicleClusteringService like ItsG5Service
{
    parameters:
        @signal[ClusterCount](type=long);
        @signal[ClusterHead](type=long);
        @signal[ClusterSize](type=long);

        @statistic[clusterCount](source=ClusterCount; record=vector);
        @statistic[clusterSize](source=ClusterSize; record=histogram,vector);
        @statistic[clusterHead](source=ClusterHead; record=vector);

        string mode = default("spatial");
        double clusteringInterval @unit(s) = default(0.5s);

        double epsilonRadius @unit(m) = default(50m);
        int minPts = default(3);

        double speedTolerance @unit(km/h) = default(5);
        double windowSeconds @unit(s) = default(1s);

        bool includeSelf = default(true);
        bool allowSingletons = default(false);

        bool headStickiness = default(true);
        double reassignmentCooldown @unit(s) = default(2s);
}
