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

#include "PhysicalEnvironmentMine.h"
#include "inet/common/geometry/shape/Cuboid.h"

#define PI 3.14159265358979323846

namespace inet {

namespace physicalenvironment {

Define_Module(PhysicalEnvironmentMine);

PhysicalEnvironmentMine::PhysicalEnvironmentMine() {
}

PhysicalEnvironmentMine::~PhysicalEnvironmentMine() {
    // TODO Auto-generated destructor stub
}

void PhysicalEnvironmentMine::initialize(int stage)
{
    if (stage == INITSTAGE_PHYSICAL_ENVIRONMENT)
        {
            inst_env.tunnel_width = par("tunnel_width");
            inst_env.tunnel_height = par("tunnel_height");
            inst_env.epsilon_ceil = par("epsilon_ceiling");
            inst_env.epsilon_wall = par("epsilon_wall");
            inst_env.sigma_ceil = par("sigma_ceiling");
            inst_env.sigma_wall = par("sigma_wall");
            inst_env.pillar_width = par("pillar_width");
            inst_env.pillar_length = par("pillar_length");
        }
}

bool PhysicalEnvironmentMine::isInObject(const IPhysicalObject *object, const Coord& point) const
{
    const Cuboid *pillar = dynamic_cast<const Cuboid *>(object);
    Coord boundingBox = pillar->computeBoundingBoxSize();
    if (!pillar) return false;
    bool insideX = point.x - object->getPosition().x < boundingBox.x;
    bool insideY = point.y - object->getPosition().y < boundingBox.y;
    bool insideZ = point.z - object->getPosition().z < boundingBox.z;
    return insideX && insideY && insideZ;
}

bool PhysicalEnvironmentMine::isObstacle(const Coord& point) const
{
    for (int i = 0; i < getNumObjects(); i++) {
        if (isInObject(getObject(i), point)) {
            return true;
        }
    }
    return false;
}

const IPhysicalObject* PhysicalEnvironmentMine::getObjectAtCoord(const Coord& point) const
{
    for (int i = 0; i < getNumObjects(); i++) {
        if (isInObject(getObject(i), point)) {
            return getObject(i);
        }
    }
    return NULL;
}

bool PhysicalEnvironmentMine::isLineBlocked(const IPhysicalObject *object, const Coord& transmissionPosition, const Coord& receptionPosition) const
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

PhysicalEnvironmentMine::LOSVisiter::LOSVisiter(const PhysicalEnvironmentMine *env, const Coord& transmissionPosition, const Coord& receptionPosition) :
                env(env),
                transmissionPosition(transmissionPosition),
                receptionPosition(receptionPosition)
            {
            }

void PhysicalEnvironmentMine::LOSVisiter::visit(const cObject *object) const
{
    if (!isObstacleFound_)
        isObstacleFound_ = env->isLineBlocked(check_and_cast<const IPhysicalObject *>(object), transmissionPosition, receptionPosition);
}

bool PhysicalEnvironmentMine::hasLineOfSight(const Coord& point1, const Coord& point2) const
{
    LOSVisiter visitor(this, point1, point2);
    visitObjects(&visitor, LineSegment(point1, point2));
    return !visitor.isObstacleFound();
}

int PhysicalEnvironmentMine::getLocationType(const Coord& point) const
{
    int xblock = 0;
    int yblock = 0;

    if (isObstacle(point)) return TUNNEL_BLOCKED;

    Coord c(point);

    c.x = point.x + (inst_env.tunnel_width / 2) + 1;
    if (isObstacle(c)) {
        xblock = xblock + 1;
    }
    c.x = point.x - (inst_env.tunnel_width / 2) - 1;
    if (isObstacle(c)) {
        xblock = xblock + 1;
    }
    c.x = point.x + inst_env.tunnel_width;
    if (isObstacle(c)) {
        xblock = xblock + 1;
    }
    c.x = point.x - inst_env.tunnel_width;
    if (isObstacle(c)) {
        xblock = xblock + 1;
    }

    c.x = point.x;
    c.y = point.y + (inst_env.tunnel_width / 2) + 1;
    if (isObstacle(c)) {
        yblock = yblock + 1;
    }
    c.y = point.y - (inst_env.tunnel_width / 2) - 1;
    if (isObstacle(c)) {
        yblock = yblock + 1;
    }
    c.y = point.y + inst_env.tunnel_width;
    if (isObstacle(c)) {
        yblock = yblock + 1;
    }
    c.y = point.y - inst_env.tunnel_width;
    if (isObstacle(c)) {
        yblock = yblock + 1;
    }

    if (xblock >= 2 && yblock < 2) {
        return TUNNEL_VERTICAL;
    } else if (xblock <= 2 && yblock <= 2) {
        return TUNNEL_CROSSCUT;
    }
    return TUNNEL_HORIZONTAL;
}

const Coord& PhysicalEnvironmentMine::getTunnelCoordinates(const Coord& point, const Coord& t_pos) const
{
    Coord c(point);
    const IPhysicalObject *o;
    double edge;
    const Coord* pos;
    int loc_type = getLocationType(point);
    if (loc_type == TUNNEL_CROSSCUT) {
        if (point.x == t_pos.x && point.y == t_pos.y) {
            loc_type = TUNNEL_VERTICAL;
        } else {
            double angle = atan2(point.y - t_pos.y, point.x - t_pos.y);
//            if (angle < 0) angle = -angle;
            if ((angle >= (PI/4.0) && angle <= (3*PI/4.0)) || (angle >= (5*PI/4.0) && angle <= (7*PI/4.0))) {
                loc_type = TUNNEL_HORIZONTAL;
            } else {
                loc_type = TUNNEL_VERTICAL;
            }
        }
    }
    switch (loc_type) {
    case TUNNEL_VERTICAL:
        c.x = point.x - inst_env.tunnel_width - 1;
        o = getObjectAtCoord(c);
        edge = o->getPosition().x + o->getShape()->computeBoundingBoxSize().x;
        pos = new Coord(point.x - edge, point.y, point.z);
        return *pos;
    case TUNNEL_HORIZONTAL:
        c.y = point.y - inst_env.tunnel_width - 1;
        o = getObjectAtCoord(c);
        edge = o->getPosition().y + o->getShape()->computeBoundingBoxSize().y;
        pos = new Coord(point.y - edge, point.x, point.z);
        return *pos;
    default:
        return Coord().NIL;
    };
}
}
}

