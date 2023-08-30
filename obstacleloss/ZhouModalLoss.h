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
#include "mine/obstacleloss/MineEnvironment.h"

namespace inet {
namespace physicallayer {

class INET_API ZhouModalLoss : public FreeSpacePathLoss
{

protected:
    class TotalObstacleLossComputation : public IVisitor
    {

    protected:
        mutable double totalLoss;
        const ZhouModalLoss *obstacleLoss = nullptr;
        const Coord transmissionPosition;
        const Coord receptionPosition;
        mutable bool isObstacleFound_ = false;

      public:
        TotalObstacleLossComputation(const ZhouModalLoss *obstacleLoss, const Coord& transmissionPosition, const Coord& receptionPosition);
        void visit(const cObject *object) const override;
        bool isObstacleFound() const { return isObstacleFound_; }
        double getTotalLoss() const { return totalLoss; }
    };

    physicalenvironment::MineEnvironment *physicalEnvironment = nullptr;
//    std::string *


private:
    omnetpp::simsignal_t tx_x_stat, tx_y_stat, tx_z_stat;
    omnetpp::simsignal_t rx_x_stat, rx_y_stat, rx_z_stat;
    omnetpp::simsignal_t snr;
protected:
    virtual void initialize(int stage) override;
    virtual bool isObstacle(const physicalenvironment::IPhysicalObject *object, const Coord& transmissionPosition, const Coord& receptionPosition) const;

public:
    ZhouModalLoss();
    virtual std::ostream& printToStream(std::ostream& stream, int level) const override;
    virtual double computePathLoss(const ITransmission *transmission, const IArrival *arrival) const override;
    bool compute_loss_modal(double width, double height, double eps_ceil, double eps_wall, int modes, double t_pos[], double r_pos[], Hz f) const;
    virtual double phi(int v) const;
    virtual double f2w(double f, double s) const;
//    cOutVector *getTxXStat() const {return &tx_x_stat;}
//    cOutVector *getTxYStat() const {return &tx_y_stat;}
//    cOutVector *getTxZStat() const {return &tx_z_stat;}
//    cOutVector *getRxXStat() const {return &rx_x_stat;}
//    cOutVector *getRxYStat() const {return &rx_y_stat;}
//    cOutVector *getRxZStat() const {return &rx_z_stat;}
//    double computeObstacleLoss(const Coord& transmissionPosition, const Coord& receptionPosition) const;
};

}  // physicallayer
}  // inet

#endif /* MINE_OBSTACLELOSS_MANHATTANPATHLOSS_H_ */
