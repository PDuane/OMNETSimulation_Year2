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

#include "ManhattanObstacleLoss.h"

#include <cmath>
#include <iostream>
#include "inet/common/ModuleAccess.h"
#include "inet/physicallayer/pathloss/FreeSpacePathLoss.h"

namespace inet {

namespace physicallayer {

using namespace inet::physicalenvironment;

Define_Module(ManhattanObstacleLoss);

ManhattanObstacleLoss::ManhattanObstacleLoss() {}

void ManhattanObstacleLoss::initialize(int stage)
{
    if (stage == INITSTAGE_LOCAL) {
        medium = check_and_cast<IRadioMedium *>(getParentModule());
        physicalEnvironment = getModuleFromPar<IPhysicalEnvironment>(par("physicalEnvironmentModule"), this);
    }
}

std::ostream& ManhattanObstacleLoss::printToStream(std::ostream& stream, int level) const
{
    return stream << "ManhattanObstacleLoss";
}

double ManhattanObstacleLoss::computeObstacleLoss(Hz frequency, const Coord& transmissionPosition, const Coord& receptionPosition) const
{
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

bool ManhattanObstacleLoss::isObstacle(const IPhysicalObject *object, const Coord& transmissionPosition, const Coord& receptionPosition) const
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
    if (isObstacle) {
        ObstaclePenetratedEvent event(object, intersection1, intersection2, normal1, normal2, isObstacle ? 1 : 0);
        const_cast<ManhattanObstacleLoss *>(this)->emit(obstaclePenetratedSignal, &event);
    }
    return isObstacle;
}

void ManhattanObstacleLoss::TotalObstacleLossComputation::visit(const cObject *object) const
{
    if (!isObstacleFound_)
        isObstacleFound_ = obstacleLoss->isObstacle(check_and_cast<const IPhysicalObject *>(object), transmissionPosition, receptionPosition);
}

ManhattanObstacleLoss::TotalObstacleLossComputation::TotalObstacleLossComputation(const ManhattanObstacleLoss *obstacleLoss, const Coord& transmissionPosition, const Coord& receptionPosition) :
    totalLoss(1),
    obstacleLoss(obstacleLoss),
    transmissionPosition(transmissionPosition),
    receptionPosition(receptionPosition)
{
}


} // namespace physicallayer
} // namespace inet
