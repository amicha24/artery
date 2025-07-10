//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

package artery.application;

simple ClusterService like ItsG5Service
{
    parameters:
        double clusterRange @unit(m) = default(100m);
        double maxSpeedDifference @unit(mps) = default(10mps);
        double maxHeadingDifference @unit(rad) = default(0.52rad);
        double maintenanceInterval @unit(s) = default(2s);
        double memberTimeout @unit(s) = default(5s);
        
        @class(ClusterService);
        @display("i=block/network2");
}
