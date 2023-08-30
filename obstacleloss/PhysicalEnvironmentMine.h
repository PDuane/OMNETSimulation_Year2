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

#ifndef MINE_OBSTACLELOSS_PHYSICALENVIRONMENTMINE_H_
#define MINE_OBSTACLELOSS_PHYSICALENVIRONMENTMINE_H_

#include "inet/environment/common/PhysicalEnvironment.h"

#define ENV_T struct environment_t *

#define TUNNEL_VERTICAL 0
#define TUNNEL_HORIZONTAL 1
#define TUNNEL_CROSSCUT 2
#define TUNNEL_BLOCKED 3
#define TUNNEL_UNKNOWN -1

namespace inet {

namespace physicalenvironment {

class PhysicalEnvironmentMine : public PhysicalEnvironment {
    protected:
        class LOSVisiter : public IVisitor
        {

        protected:
            const PhysicalEnvironmentMine *env;
            const Coord transmissionPosition;
            const Coord receptionPosition;
            mutable bool isObstacleFound_ = false;

          public:
            LOSVisiter(const PhysicalEnvironmentMine *env, const Coord& transmissionPosition, const Coord& receptionPosition);
            void visit(const cObject *object) const override;
            bool isObstacleFound() const {return isObstacleFound_;}
        };
    public:
        struct environment_t {
            double tunnel_width;
            double tunnel_height;
            double epsilon_ceil;
            double epsilon_wall;
            double sigma_ceil;
            double sigma_wall;
            double pillar_width;
            double pillar_length;
        } inst_env;

        PhysicalEnvironmentMine();
        virtual ~PhysicalEnvironmentMine();

        virtual bool isObstacle(const Coord& point) const;
        virtual const IPhysicalObject* getObjectAtCoord(const Coord& point) const;
        virtual bool isLineBlocked(const IPhysicalObject *object, const Coord& transmissionPosition, const Coord& receptionPosition) const;
        virtual bool hasLineOfSight(const Coord& point1, const Coord& point2) const;
        virtual int getLocationType(const Coord& point) const;
        virtual const Coord& getTunnelCoordinates(const Coord& point, const Coord& t_pos) const;

        const ENV_T getEnvironmentParameters() const {return &inst_env;}

//        virtual void visitObjects(const IVisitor *visitor, const LineSegment& lineSegment) const override;

    protected:
        virtual void initialize(int stage) override;
    private:
    //            struct enviroment_t inst_evn;
        virtual bool isInObject(const IPhysicalObject *object, const Coord& point) const;
};
}
}

#endif /* MINE_OBSTACLELOSS_PHYSICALENVIRONMENTMINE_H_ */
