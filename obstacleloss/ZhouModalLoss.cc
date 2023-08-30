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
#include <complex>
#include "ZhouModalLoss.h"
#include "inet/common/ModuleAccess.h"
#include "inet/physicallayer/base/packetlevel/TracingObstacleLossBase.h"

#define PI 3.14159265358979323846

namespace inet {

namespace physicallayer {

using namespace inet::physicalenvironment;

Define_Module(ZhouModalLoss);

    ZhouModalLoss::ZhouModalLoss() {}

    void ZhouModalLoss::initialize(int stage)
    {
        tx_x_stat = registerSignal("tx_x");
        tx_y_stat = registerSignal("tx_y");
        tx_z_stat = registerSignal("tx_z");
        rx_x_stat = registerSignal("rx_x");
        rx_y_stat = registerSignal("rx_y");
        rx_z_stat = registerSignal("rx_z");

        if (stage == INITSTAGE_LOCAL) {
            std::cout << "Initializing Manhattan Path Loss" << endl;
            physicalEnvironment = getModuleFromPar<MineEnvironment>(par("physicalEnvironmentModule"), this, true);
        }
    }

    std::ostream& ZhouModalLoss::printToStream(std::ostream& stream, int level) const
    {
        stream << "Manhattan Path Loss";
        return stream;
    }

    double ZhouModalLoss::computePathLoss(const ITransmission *transmission, const IArrival *arrival) const
    {
//        auto radioMedium = transmission->getMedium();
        auto narrowbandSignalAnalogModel = check_and_cast<const INarrowbandSignal *>(transmission->getAnalogModel());
        auto transmissionPosition = transmission->getStartPosition();
        auto receptionPosition = arrival->getStartPosition();
        Hz centerFrequency = narrowbandSignalAnalogModel->getCenterFrequency();

        /*
         * Determine if there is an obstacle in the way
         */
        TotalObstacleLossComputation obstacleLossVisitor(this, transmissionPosition, receptionPosition);
        physicalEnvironment->visitObjects(&obstacleLossVisitor, LineSegment(transmissionPosition, receptionPosition));
        Coord p1 = Coord(transmissionPosition.x, transmissionPosition.y, NaN);
        Coord p2 = Coord(receptionPosition.x, receptionPosition.y, NaN);
        bool obstructed = !physicalEnvironment->hasLineOfSight(p1, p2);
//        if (me = dynamic_cast<MineEnvironment>(physicalEnvironment)) {
//            p1->x = transmissionPosition.x;
//            p1->y = transmissionPosition.y;
//            p2->x = receptionPosition.x;
//            p2->y = receptionPosition.y;
//        } else {
//            obstructed = obstacleLossVisitor.isObstacleFound();
//        }

        /*
         * Define some basic elements
         */

        double turnLoss = 40;
        int modes = 5;

//        double slope = -0.102299107 * ((2.54 * 100) / 12.0);
//        double lossDb;
//        double d;
//
//        std::cout << slope;

        /*
         * If the path is obstructed, use Manhattan distance with turn loss
         */

        double t_pos[] = {transmissionPosition.x, transmissionPosition.z, transmissionPosition.y};
        double r_pos[] = {receptionPosition.x, receptionPosition.z, receptionPosition.y};

        double loss = compute_loss_modal(2.4, 1.8, 8.9, 8.9, 3, t_pos, r_pos, centerFrequency);

        if (obstructed) {
            loss *= pow(10, -turnLoss/10);
        }
        /*
         * If not obstructed, use Euclidean distance
         */
//        else {
//            double x = receptionPosition.x - transmissionPosition.x;
//            double y = receptionPosition.y - transmissionPosition.y;
//            d = sqrt(x * x + y * y);
//            lossDb = 0;
//            std::cout << "; los";
//        }

//        std::cout << "; dist = " << d << "m";
//        lossDb += slope * d;
//        std::cout << "; loss = " << lossDb << " dB"<< endl;
//        double loss = pow(10, lossDb / 10);
//        getTxXStat()->record(transmissionPosition.x);
//        getTxYStat()->record(transmissionPosition.y);
//        getTxZStat()->record(transmissionPosition.z);
//        tx_x_stat.record(transmissionPosition.x);
//        tx_y_stat.record(transmissionPosition.y);
//        tx_z_stat.record(transmissionPosition.z);

//        Coord rx = new Coord(receptionPosition);
//        rx_x_stat.record(rx.x);
//        rx_y_stat.record(rx.y);
//        rx_z_stat.record(rx.z);

//        snr.record(10 * log10(loss));

//        emit(tx_x_stat, transmissionPosition.x);
//        emit(tx_y_stat, transmissionPosition.y);
//        emit(tx_z_stat, transmissionPosition.z);
//        emit(rx_x_stat, receptionPosition.x);
//        emit(rx_y_stat, receptionPosition.y);
//        emit(rx_z_stat, receptionPosition.z);

        return loss;
    }

    bool ZhouModalLoss::compute_loss_modal(double width, double height, double eps_ceil, double eps_wall, int modes, double t_pos[], double r_pos[], Hz f) const
    {
        double c = 299792458;
        double k = 2 * PI * f.get() / c;
        double a = width / 2;
        double b = height / 2;

        std::complex<double> m(0.0, -2 * PI / (a * b));

        std::complex<double> sum(0.0, 0.0);

        for (int p = 1; p <= modes; p++) {
            for (int q = 1; q <= modes; q++) {
                std::complex<double> Lambda_pq(1.0, 1.0);
                Lambda_pq = Lambda_pq *
                        (sin(p * PI * r_pos[0] / (2 * a)) + phi(p)) * (sin(q * PI * r_pos[1] / (2 * b)) + phi(q)) *
                        (sin(p * PI * t_pos[0] / (2 * a)) + phi(p)) * (sin(q * PI * t_pos[1] / (2 * b)) + phi(q));

                std::complex<double> alpha_pq(1.0, 1.0);
                alpha_pq = alpha_pq * (
                        (1 / a) * pow(p * f2w(f.get(),c) / (4 * a), 2) * std::real(1 / sqrt(eps_wall - 1)) + (1 / b) * (q * f2w(f.get(), c) / (4 * b) * std::real(eps_ceil / sqrt(eps_ceil - 1)))
                        );

                std::complex<double> beta_pq(1.0, 1.0);
                beta_pq = beta_pq *
                        (
                                sqrt(k * k + pow(p * PI / (2 * a), 2) + pow(1 * PI / (2 * b), 2))
                        );

                sum = sum + Lambda_pq * exp(-(alpha_pq + std::complex<double>(0.0, 1.0) * beta_pq) * abs(r_pos[2] - t_pos[2])) / beta_pq;
            }
        }

        return real(sum);
    }

    double ZhouModalLoss::phi(int v) const
    {
        return (PI / 2) * (v % 2);
//        if (v % 2) return PI / 2;
//        else return 0;
    }

    double ZhouModalLoss::f2w(double f, double s) const
    {
        return s / f;
    }

    bool ZhouModalLoss::isObstacle(const IPhysicalObject *object, const Coord& transmissionPosition, const Coord& receptionPosition) const
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

    void ZhouModalLoss::TotalObstacleLossComputation::visit(const cObject *object) const
    {
        if (!isObstacleFound_)
            isObstacleFound_ = obstacleLoss->isObstacle(check_and_cast<const IPhysicalObject *>(object), transmissionPosition, receptionPosition);
    }

    ZhouModalLoss::TotalObstacleLossComputation::TotalObstacleLossComputation(const ZhouModalLoss *obstacleLoss, const Coord& transmissionPosition, const Coord& receptionPosition) :
        totalLoss(1),
        obstacleLoss(obstacleLoss),
        transmissionPosition(transmissionPosition),
        receptionPosition(receptionPosition)
    {
    }

} // physicallayer
} // inet

