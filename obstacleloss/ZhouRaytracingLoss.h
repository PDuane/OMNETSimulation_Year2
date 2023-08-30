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

#ifndef MINE_OBSTACLELOSS_ZHOUMODALLOSS_H_
#define MINE_OBSTACLELOSS_ZHOUMODALLOSS_H_

#include "inet/common/IVisitor.h"
#include "inet/common/figures/TrailFigure.h"
#include "inet/environment/contract/IPhysicalEnvironment.h"
#include "inet/physicallayer/pathloss/FreeSpacePathLoss.h"
#include "inet/physicallayer/contract/packetlevel/IRadioMedium.h"
#include "mine/obstacleloss/PhysicalEnvironmentMine.h"

#define NUM_MODES 25
#define CORNER_THRESHOLD 30
#define CORNER_LOSS 30

namespace inet {
namespace physicallayer {

class INET_API ZhouRaytracingLoss : public FreeSpacePathLoss
{

protected:
    class TotalObstacleLossComputation : public IVisitor
    {

    protected:
        mutable double totalLoss;
        const ZhouRaytracingLoss *obstacleLoss = nullptr;
        const Coord transmissionPosition;
        const Coord receptionPosition;
        mutable bool isObstacleFound_ = false;

      public:
        TotalObstacleLossComputation(const ZhouRaytracingLoss *obstacleLoss, const Coord& transmissionPosition, const Coord& receptionPosition);
        void visit(const cObject *object) const override;
        bool isObstacleFound() const { return isObstacleFound_; }
        double getTotalLoss() const { return totalLoss; }
    };

    const physicalenvironment::PhysicalEnvironmentMine *physicalEnvironment;

protected:
    virtual void initialize(int stage) override;
    virtual bool isObstacle(const physicalenvironment::IPhysicalObject *object, const Coord& transmissionPosition, const Coord& receptionPosition) const;

public:
    ZhouRaytracingLoss();
    virtual std::ostream& printToStream(std::ostream& stream, int level) const override;
    virtual double computePathLoss(const ITransmission *transmission, const IArrival *arrival) const override;
    double compute_loss_raytrace(Coord& t_pos, Coord& r_pos, Hz f, const physicalenvironment::PhysicalEnvironmentMine* env) const;
};

}  // physicallayer
}  // inet

#endif /* MINE_OBSTACLELOSS_ZHOUMODALLOSS_H_ */
