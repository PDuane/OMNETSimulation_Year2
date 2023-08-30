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
#include "ZhouRaytracingLoss.h"
#include "inet/common/ModuleAccess.h"
#include "inet/physicallayer/base/packetlevel/TracingObstacleLossBase.h"

#define PI 3.14159265358979323846

namespace inet {

namespace physicallayer {

using namespace inet::physicalenvironment;

Define_Module(ZhouRaytracingLoss);

    ZhouRaytracingLoss::ZhouRaytracingLoss() {}

    void ZhouRaytracingLoss::initialize(int stage)
    {

        if (stage == INITSTAGE_LOCAL) {
            std::cout << "Initializing Manhattan Path Loss" << endl;
            physicalEnvironment = getModuleFromPar<PhysicalEnvironmentMine>(par("physicalEnvironmentModule"), this, true);
        }
    }

    std::ostream& ZhouRaytracingLoss::printToStream(std::ostream& stream, int level) const
    {
        stream << "Manhattan Path Loss";
        return stream;
    }

    double ZhouRaytracingLoss::computePathLoss(const ITransmission *transmission, const IArrival *arrival) const
    {
//        auto radioMedium = transmission->getMedium();
        auto narrowbandSignalAnalogModel = check_and_cast<const INarrowbandSignal *>(transmission->getAnalogModel());
        auto transmissionPosition = transmission->getStartPosition();
        auto receptionPosition = arrival->getStartPosition();
        Hz centerFrequency = narrowbandSignalAnalogModel->getCenterFrequency();

        /*
         * Determine if there is an obstacle in the way
         */
//        TotalObstacleLossComputation obstacleLossVisitor(this, transmissionPosition, receptionPosition);
//        physicalEnvironment->visitObjects(&obstacleLossVisitor, LineSegment(transmissionPosition, receptionPosition));
//        Coord p1 = Coord(transmissionPosition.x, transmissionPosition.y, NaN);
//        Coord p2 = Coord(receptionPosition.x, receptionPosition.y, NaN);
//        bool obstructed = obstacleLossVisitor.isObstacleFound();

        /*
         * Define some basic elements
         */

        double turnLoss = 40;
        double turnLossThresh = 50;
        int modes = 25;

        /*
         * If the path is obstructed, use Manhattan distance with turn loss
         */

        /* Note the transposition of the y and z coordinates.
         * This is to fit the notation of Zhou's model
         */
        Coord t_pos(transmissionPosition.x, transmissionPosition.z, transmissionPosition.y);
        Coord r_pos(receptionPosition.x, receptionPosition.z, receptionPosition.y);

        double loss = compute_loss_raytrace(t_pos, r_pos, centerFrequency, physicalEnvironment);

//        if (obstructed) {
//            bool xTurn = (abs(r_pos[0]-t_pos[0]) < abs(r_pos[1]-t_pos[1]));
//            if (xTurn && r_pos[0]-t_pos[0] < turnLossThresh) {
//                double l = turnLoss * ((r_pos[0]-t_pos[0]) / turnLossThresh);
//                loss *= pow(10, -l/10);
//            } else if (!xTurn && r_pos[1]-t_pos[1] < turnLossThresh) {
//                double l = turnLoss * ((r_pos[1]-t_pos[1]) / turnLossThresh);
//                loss *= pow(10, -l/10);
//            } else {
//                loss *= pow(10, -turnLoss/10);
//            }
//        }

        return loss;
    }

    double ZhouRaytracingLoss::compute_loss_raytrace(Coord& t_pos, Coord& r_pos, Hz f, const physicalenvironment::PhysicalEnvironmentMine* env) const
    {
        double c = 299792458;
        double eps_zero = 8.854187817 * pow(10, -12);
        double k = 2 * PI * f.get() / c;
        double a = env->getEnvironmentParameters()->tunnel_width / 2;
        double b = env->getEnvironmentParameters()->tunnel_height / 2;

        std::complex<double> eps_ceil_cplex(env->getEnvironmentParameters()->epsilon_ceil - 1j*env->getEnvironmentParameters()->sigma_ceil/(2*PI*f.get()*eps_zero));
        std::complex<double> eps_wall_cplex(env->getEnvironmentParameters()->epsilon_wall - 1j*env->getEnvironmentParameters()->sigma_wall/(2*PI*f.get()*eps_zero));

        Coord tx_tun_pos = env->getTunnelCoordinates(t_pos, t_pos);
        Coord rx_tun_pos = env->getTunnelCoordinates(r_pos, t_pos);

        double tx_x = tx_tun_pos.x;
        double tx_y = tx_tun_pos.y;
        double tx_z = tx_tun_pos.z;

        double rx_x = rx_tun_pos.x;
        double rx_y = rx_tun_pos.y;
        double rx_z = rx_tun_pos.z;

        std::complex<double> sum(0);

        double y_apnt;
        double corner_loss;

        if (env->hasLineOfSight(t_pos, r_pos)) {
            y_apnt = abs(rx_y - tx_y);
            corner_loss = 1;
        } else {
            y_apnt = rx_y-tx_y;
            if (abs(r_pos.x - t_pos.x) < CORNER_THRESHOLD) {
                corner_loss = abs(r_pos.x - t_pos.x) * CORNER_LOSS;
            } else if (abs(r_pos.z - t_pos.z) < CORNER_THRESHOLD) {
                corner_loss = abs(r_pos.z - t_pos.z) * CORNER_LOSS;
            } else {
                corner_loss = CORNER_LOSS;
            }
            corner_loss = 10 * log10(-corner_loss / 10);
        }

        for (int m = -NUM_MODES; m <= NUM_MODES; m++) {
            for (int n = -NUM_MODES; n <= NUM_MODES; n++) {
                double x_m = (2 * m * a) + (pow(-1, m) * tx_x);
                double y_n = (2 * n * b) + (pow(-1, n) * tx_y);

                // This r_mn calculation does not support a transmitter at z != 0
                // Distance from transmitter to receiver

                double r_mn = sqrt(pow(rx_x - x_m, 2) + pow(y_apnt, 2) + pow(rx_z - tx_z, 2));

                // Incidence angle
                double theta_perp = acos(abs(x_m - rx_x) / r_mn);
                double theta_pll = acos(abs(y_n - rx_y) / r_mn);

                // Impedance factor
                std::complex<double> Delta_perp = sqrt(eps_wall_cplex - pow(sin(theta_perp), 2));
                std::complex<double> Delta_pll = sqrt(eps_ceil_cplex - pow(sin(theta_pll), 2)) / eps_ceil_cplex;

                // Reflection coefficient
                std::complex<double> rho_perp = (cos(theta_perp) - Delta_perp) / (cos(theta_perp) + Delta_perp);
                std::complex<double> rho_pll = (cos(theta_pll) - Delta_pll) / (cos(theta_pll) + Delta_pll);

                // Add the results of the iteration to the total
                sum = sum + ((exp(std::complex<double>(0, k * r_mn)) / r_mn) * pow(rho_perp,abs(m)) * pow(rho_pll,abs(n)));
            }
        }

        return real(pow((c/f.get()) / (4 * PI), 2) + pow(sum * corner_loss, 2));
    }

    bool ZhouRaytracingLoss::isObstacle(const IPhysicalObject *object, const Coord& transmissionPosition, const Coord& receptionPosition) const
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

    void ZhouRaytracingLoss::TotalObstacleLossComputation::visit(const cObject *object) const
    {
        if (!isObstacleFound_)
            isObstacleFound_ = obstacleLoss->isObstacle(check_and_cast<const IPhysicalObject *>(object), transmissionPosition, receptionPosition);
    }

    ZhouRaytracingLoss::TotalObstacleLossComputation::TotalObstacleLossComputation(const ZhouRaytracingLoss *obstacleLoss, const Coord& transmissionPosition, const Coord& receptionPosition) :
        totalLoss(1),
        obstacleLoss(obstacleLoss),
        transmissionPosition(transmissionPosition),
        receptionPosition(receptionPosition)
    {
    }

} // physicallayer
} // inet

