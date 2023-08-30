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

#ifndef MINE_OBSTACLELOSS_MINEENVIRONMENT_H_
#define MINE_OBSTACLELOSS_MINEENVIRONMENT_H_

#define TUNNEL_T struct tunnel_t *
#define POINT_T struct point_t *

#include "inet/environment/contract/IObjectCache.h"
#include "inet/environment/contract/IPhysicalEnvironment.h"
#include <map>

struct tunnel_t {
    double px1, py1, pz1;
    double px2, py2, pz2;
    double width, height;
    double permittivity_vertical;
    double permittivity_horizontal;
};

struct point_t {
    double x, y, z;
};

namespace inet {

namespace physicalenvironment {

class INET_API MineEnvironment : public cModule, public IPhysicalEnvironment
{
    protected:
        virtual void initialize(int stage) override;
        virtual void parseTunnels(cXMLElement *xml);
        bool pointInTunnel(const TUNNEL_T t, const Coord p);

        IObjectCache *objectCache = nullptr;
        IGround *ground = nullptr;
        std::map<const std::string, const TUNNEL_T> tunnelMap;
    public:
        MineEnvironment();
        virtual ~MineEnvironment();
        bool hasLineOfSight(Coord p1, Coord p2);

        virtual IObjectCache *getObjectCache() const override;
        virtual IGround *getGround() const override;

        virtual const Coord& getSpaceMin() const override;
        virtual const Coord& getSpaceMax() const override;
        virtual const IMaterialRegistry *getMaterialRegistry() const override;

        virtual int getNumObjects() const override;
        virtual const IPhysicalObject *getObject(int index) const override;
        virtual const IPhysicalObject *getObjectById(int id) const override;

        virtual void visitObjects(const IVisitor *visitor, const LineSegment& lineSegment) const override;
    };
}
}

#endif /* MINE_OBSTACLELOSS_MINEENVIRONMENT_H_ */
