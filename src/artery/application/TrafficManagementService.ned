/*
 * Artery V2X Simulation Framework
 * Copyright 2025 Devin AI
 * Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
 */

package artery.application;

simple TrafficManagementService like ItsG5BaseService
{
    parameters:
        double congestionSpeedThreshold @unit(kmph) = default(30kmph);
        double highDensityThreshold = default(10.0);
        double recommendedSpeed @unit(kmph) = default(50kmph);
        double analysisInterval @unit(s) = default(5s);
        double messageValidityDuration @unit(s) = default(60s);
        
        @class(TrafficManagementService);
        @display("i=block/network2");
}
