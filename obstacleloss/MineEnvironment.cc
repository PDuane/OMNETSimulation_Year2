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

#include "MineEnvironment.h"
#include <algorithm>
#include <cstdlib>

namespace inet {

namespace physicalenvironment {

Define_Module(MineEnvironment);

MineEnvironment::MineEnvironment() {
    // TODO Auto-generated constructor stub

}

MineEnvironment::~MineEnvironment() {
    // TODO Auto-generated destructor stub
}

void MineEnvironment::initialize(int stage)
{
    if (stage == INITSTAGE_PHYSICAL_ENVIRONMENT)
        {
            cXMLElement *environment = par("config");
            parseTunnels(environment);
        }
}

void MineEnvironment::parseTunnels(cXMLElement *xml)
{
    cXMLElementList children = xml->getChildrenByTagName("tunnel");
    for (cXMLElementList::const_iterator it = children.begin(); it != children.end(); ++it) {
        cXMLElement *element = *it;
        const char *p1 = element->getAttribute("p1");
        const char *p2 = element->getAttribute("p2");

        char result[100];   // array to hold the result.
        strcpy(result,p1); // copy string one into the result.
        strcat(result,p2);
        std::string str = std::string(result);
        std::remove(str.begin(), str.end(), ' ');
        TUNNEL_T tun = new struct tunnel_t;
//        TUNNEL_T tun = malloc(sizeof(struct tunnel_t));

        char cstr[128];
        std::strncpy(cstr, str.c_str(), 128);

        tun->px1 = std::atof(std::strtok(cstr, ","));
        tun->py1 = std::atof(std::strtok(NULL, ","));
        tun->pz1 = std::atof(std::strtok(NULL, ","));
        tun->px2 = std::atof(std::strtok(NULL, ","));
        tun->py2 = std::atof(std::strtok(NULL, ","));
        tun->pz2 = std::atof(std::strtok(NULL, ","));

        const char *width = element->getAttribute("width");
        const char *height = element->getAttribute("height");

        tun->width = std::atof(width);
        tun->height = std::atof(height);

        const char *ptvy_vert = element->getAttribute("permittivity-vertical");
        const char *ptvy_horz = element->getAttribute("permittivity-horizontal");

        tun->permittivity_vertical = std::atof(ptvy_vert);
        tun->permittivity_horizontal = std::atof(ptvy_horz);

        tunnelMap.insert(std::pair<std::string, const TUNNEL_T>(str, tun));
    }
}

// Determine if a point is in a tunnel by comparing the area of
//     the tunnel rectangle to the area of the 4 triangles made
//     by drawing lines from the corners to the point (area will
//     be the same if the point is in the rectangle)
bool MineEnvironment::pointInTunnel(const TUNNEL_T t, const Coord c)
{

    double length = sqrt(pow(t->px1 - t->px2, 2) + pow(t->py1 - t->py2, 2));

    double angle = atan2(t->py1 - t->py2, t->px1 - t->px2);

    POINT_T corners[4];
    corners[0]->x = t->px1 - (0.5 * t->width * sin(angle));
    corners[0]->y = t->py1 + (0.5 * t->width * cos(angle));

    corners[1]->x = t->px2 - (0.5 * t->width * sin(angle));
    corners[1]->y = t->py2 + (0.5 * t->width * cos(angle));

    corners[2]->x = t->px2 + (0.5 * t->width * sin(angle));
    corners[2]->y = t->py2 - (0.5 * t->width * cos(angle));

    corners[3]->x = t->px1 + (0.5 * t->width * sin(angle));
    corners[3]->y = t->py1 - (0.5 * t->width * cos(angle));

    double corn2pnt[4];

    for (int i = 0; i < 4; i++) {
        corn2pnt[i] = sqrt(pow(c.x - corners[i]->x, 2) + pow(c.y - corners[i]->y, 2));
    }

    double areas[4];
    double sum = 0;
    for (int i = 0; i < 4; i++) {
        double side1 = corn2pnt[i];
        double side2 = corn2pnt[(i + 1) % 4];
        double side3 = i % 2 ? length : t->width;
        double semi = (side1 + side2 + side3) / 2;
        areas[i] = sqrt(semi * (semi - side1) * (semi - side2) * (semi - side3));
        sum += areas[i];
    }

    return (length * t->width) - sum < 0.00000001;

//    = {
//            sqrt(pow(p->x - corners[0]->x, 2) + pow(p->y - corners[0]->y, 2)),
//            sqrt(pow(p->x - corners[1]->x, 2) + pow(p->y - corners[1]->y, 2)),
//            sqrt(pow(p->x - corners[2]->x, 2) + pow(p->y - corners[2]->y, 2)),
//            sqrt(pow(p->x - corners[3]->x, 2) + pow(p->y - corners[3]->y, 2))
//    };
}

// Use the simple assumption that only points in the same tunnel have line of sight
bool MineEnvironment::hasLineOfSight(Coord p1, Coord p2)
{
    for (std::map<const std::string, const TUNNEL_T>::iterator it=tunnelMap.begin(); it!=tunnelMap.end(); ++it) {
        const TUNNEL_T t = it->second;
        if (pointInTunnel(t, p1)) {
            if (pointInTunnel(t, p2)) {
                return true;
            }
        }
    }
    return false;
}

IObjectCache *MineEnvironment::getObjectCache() const {return NULL;}
IGround *MineEnvironment::getGround() const {return NULL;}

const Coord& MineEnvironment::getSpaceMin() const {return Coord::NIL;}
const Coord& MineEnvironment::getSpaceMax() const {return Coord::NIL;}
const IMaterialRegistry *MineEnvironment::getMaterialRegistry() const {return NULL;}

int MineEnvironment::getNumObjects() const {return tunnelMap.size();}
const IPhysicalObject *MineEnvironment::getObject(int index) const {return NULL;}
const IPhysicalObject *MineEnvironment::getObjectById(int id) const {return NULL;}

void MineEnvironment::visitObjects(const IVisitor *visitor, const LineSegment& lineSegment) const {}

}
}
