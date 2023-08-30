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

#include <iostream>
#include "ManhattanPathLoss.h"
#include "inet/common/ModuleAccess.h"
#include "inet/physicallayer/base/packetlevel/TracingObstacleLossBase.h"

namespace inet {

namespace physicallayer {

using namespace inet::physicalenvironment;

Define_Module(ManhattanPathLoss);

    ManhattanPathLoss::ManhattanPathLoss() {}

    void ManhattanPathLoss::initialize(int stage)
    {
        if (stage == INITSTAGE_LOCAL) {
            std::cout << "Initializing Manhattan Path Loss" << endl;
            physicalEnvironment = getModuleFromPar<IPhysicalEnvironment>(par("physicalEnvironmentModule"), this, true);
        }
    }

    std::ostream& ManhattanPathLoss::printToStream(std::ostream& stream, int level) const
    {
        stream << "Manhattan Path Loss";
        return stream;
    }

    double ManhattanPathLoss::computePathLoss(const ITransmission *transmission, const IArrival *arrival) const
    {
//        auto radioMedium = transmission->getMedium();
//        auto narrowbandSignalAnalogModel = check_and_cast<const INarrowbandSignal *>(transmission->getAnalogModel());
        auto transmissionPosition = transmission->getStartPosition();
        auto receptionPosition = arrival->getStartPosition();

        /*
         * Determine if there is an obstacle in the way
         */
        TotalObstacleLossComputation obstacleLossVisitor(this, transmissionPosition, receptionPosition);
        physicalEnvironment->visitObjects(&obstacleLossVisitor, LineSegment(transmissionPosition, receptionPosition));
        bool obstructed = obstacleLossVisitor.isObstacleFound();

        /*
         * Define some basic elements
         */
        double slope = -0.102299107 * ((2.54 * 100) / 12.0);
        double turnLoss = 20;
        double lossDb;
        double d;

        std::cout << slope;

        /*
         * If the path is obstructed, use Manhattan distance with turn loss
         */
        if (obstructed) {
            d = abs(receptionPosition.x - transmissionPosition.x) + abs(receptionPosition.y - transmissionPosition.y);
            lossDb = -turnLoss;
            std::cout << "; obs";
        }
        /*
         * If not obstructed, use Euclidean distance
         */
        else {
            double x = receptionPosition.x - transmissionPosition.x;
            double y = receptionPosition.y - transmissionPosition.y;
            d = sqrt(x * x + y * y);
            lossDb = 0;
            std::cout << "; los";
        }

        std::cout << "; dist = " << d << "m";
        lossDb += slope * d;
        std::cout << "; loss = " << lossDb << " dB"<< endl;
        double loss = pow(10, lossDb / 10);
        return loss;
    }

    bool ManhattanPathLoss::isObstacle(const IPhysicalObject *object, const Coord& transmissionPosition, const Coord& receptionPosition) const
    {
        if (object->getMaterial()->getRelativePermeability() == 2.0) return false;
        const ShapeBase *shape = object->getShape();
        const Coord& position = object->getPosition();
        const Quaternion& orientation = object->getOrientation();
        RotationMatrix rotation(orientation.toEulerAngles());
        const LineSegment lineSegment(rotation.rotateVectorInverse(transmissionPosition - position), rotation.rotateVectorInverse(receptionPosition - position));
        Coord intersection1, intersection2, normal1, normal2;
        bool hasIntersections = shape->computeIntersection(lineSegment, intersection1, intersection2, normal1, normal2);
        bool isObstacle = hasIntersections && intersection1 != intersection2;
        return isObstacle;
    }

    void ManhattanPathLoss::TotalObstacleLossComputation::visit(const cObject *object) const
    {
        if (!isObstacleFound_)
            isObstacleFound_ = obstacleLoss->isObstacle(check_and_cast<const IPhysicalObject *>(object), transmissionPosition, receptionPosition);
    }

    ManhattanPathLoss::TotalObstacleLossComputation::TotalObstacleLossComputation(const ManhattanPathLoss *obstacleLoss, const Coord& transmissionPosition, const Coord& receptionPosition) :
        totalLoss(1),
        obstacleLoss(obstacleLoss),
        transmissionPosition(transmissionPosition),
        receptionPosition(receptionPosition)
    {
    }

} // physicallayer
} // inet

