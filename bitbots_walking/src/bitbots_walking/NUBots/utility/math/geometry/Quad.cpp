/*
 * This file is part of the NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "Quad.h"
#include "../../../utility/math/geometry/ParametricLine.h"

namespace utility {
namespace math {
namespace geometry {

    Quad::Quad()
        : bl(arma::zeros<arma::vec>(2)), br(arma::zeros<arma::vec>(2)), tr(arma::zeros<arma::vec>(2)), tl(arma::zeros<arma::vec>(2)) {
        // Empty constructor.
    }

    Quad::Quad(const Quad& other)
        : bl(other.bl), br(other.br), tr(other.tr), tl(other.tl) {
    }

    Quad::Quad(arma::vec2 bottomLeft, arma::vec2 topLeft, arma::vec2 topRight, arma::vec2 bottomRight)
        : bl(bottomLeft), br(bottomRight), tr(topRight), tl(topLeft) {
    }

    Quad::Quad(arma::ivec2 bottomLeft, arma::ivec2 topLeft, arma::ivec2 topRight, arma::ivec2 bottomRight)
        : bl(arma::conv_to<arma::vec>::from(bottomLeft))
        , br(arma::conv_to<arma::vec>::from(bottomRight))
        , tr(arma::conv_to<arma::vec>::from(topRight))
        , tl(arma::conv_to<arma::vec>::from(topLeft)) {
    }

    Quad::Quad(double left, double top, double right, double bottom)
        : bl({left, bottom}), br({right, bottom}), tr({right, top}), tl({left, top}) {
    }

    void Quad::set(double left, double top, double right, double bottom) {
        bl[0] = left;           bl[1] = bottom;
        br[0] = right;          br[1] = bottom;
        tl[0] = left;           tl[1] = top;
        tr[0] = right;          tr[1] = top;
    }

    void Quad::set(arma::vec2 bottomLeft, arma::vec2 topLeft, arma::vec2 topRight, arma::vec2 bottomRight) {
        bl = bottomLeft;
        tl = topLeft;
        tr = topRight;
        br = bottomRight;
    }

    arma::vec2 Quad::getCentre() const {
        return ((bl + tl + tr + br) * 0.25);
    }

    double Quad::getAverageWidth() const {
        return ((0.5 * (arma::norm(br - bl, 2) + arma::norm(tr - tl, 2))));
    }

    double Quad::getAverageHeight() const {
        return ((0.5 * (arma::norm(br - tr, 2) + arma::norm(bl - tl, 2))));
    }

    double Quad::area() const {
        // Area of a quadrilateral: A = 0.5 * |diag1 X diag2|
        // In two dimensions, this equates to: A = 0.5 * |(diag1.x)(diag2.y) - (diag2.x)(diag2.y)|

        arma::vec2 diag1 = bl - tr;
        arma::vec2 diag2 = tl - br;
        return abs(0.5 * ((diag1[0] * diag2[1]) - (diag1[1] * diag2[0])));
    }

    double Quad::aspectRatio() const {
        return ((arma::norm(br - tr, 2) + arma::norm(bl - tl, 2) + 2) / (arma::norm(br - bl, 2) + arma::norm(tr - tl, 2) + 2));
    }

    bool Quad::overlapsHorizontally(const Quad &other) const {
        // Rough for now.
        double farRight = std::max(tr[0], br[0]);
        double farLeft = std::min(tl[0], bl[0]);
        double o_farRight = std::max(other.tr[0], other.br[0]);
        double o_farLeft = std::min(other.tl[0], other.bl[0]);

        return !((farRight < o_farLeft) || (o_farRight < farLeft));
    }

    arma::vec2 Quad::getTopCentre() const {
        return ((tl + tr) * 0.5);
    }

    arma::vec2 Quad::getBottomCentre() const {
        return ((bl + br) * 0.5);
    }

    arma::vec2 Quad::getRightCentre() const {
        return ((br + tr) * 0.5);
    }

    arma::vec2 Quad::getLeftCentre() const {
        return ((bl + tl) * 0.5);
    }

    arma::vec2 Quad::getBottomLeft() const {
        return bl;
    }

    arma::vec2 Quad::getBottomRight() const {
        return br;
    }

    arma::vec2 Quad::getTopLeft() const {
        return tl;
    }

    arma::vec2 Quad::getTopRight() const {
        return tr;
    }

    arma::vec2 Quad::getSize() const{
        Quad boundingBox = getBoundingBox(getVertices());
        return {boundingBox.getAverageWidth(),boundingBox.getAverageHeight()};
    }


    double Quad::getLeft() const {
        return (0.5 * (bl[0] + tl[0]));
    }

    double Quad::getRight() const {
        return (0.5 * (br[0] + tr[0]));
    }

    double Quad::getTop() const {
        return (0.5 * (tl[1] + tr[1]));
    }

    double Quad::getBottom() const {
        return (0.5 * (bl[1] + br[1]));
    }

    /// @brief Stream insertion operator for a single Quad.
    /// @relates Quad
    std::ostream& operator<< (std::ostream& output, const Quad& quad) {
        output << "(" << quad.getBottomLeft()[0] << ", " << quad.getBottomLeft()[1] << ") ("
                << quad.getTopLeft()[0] << ", " << quad.getTopLeft()[1] << ") ("
                << quad.getTopRight()[0] << ", " << quad.getTopRight()[1] << ") ("
                << quad.getBottomRight()[0] << ", " << quad.getBottomRight()[1] << ")";

        return output;
    }

    /// @brief Stream insertion operator for a std::vector of Quads.
    /// @relates Quad
    std::ostream& operator<< (std::ostream& output, const std::vector<Quad>& quads) {
        output << "[";

        for (const auto& quad : quads)
            output << quad << ", ";

        output << "]";

        return output;
    }

    std::vector<arma::vec2> Quad::getVertices() const {
        std::vector<arma::vec2> vert(4);

        vert[0] = tr;
        vert[1] = br;
        vert[2] = bl;
        vert[3] = tl;

        return vert;
    }

    bool Quad::checkCornersValid() const {
        return br.size()==2  && bl.size()==2 && tr.size()==2 && tl.size()==2;
    }

    arma::vec2 Quad::getEdgePoints(uint y) const {
        auto edgePoints = getEdgePoints(double(y));
        return {
            std::round(edgePoints[0]),
            std::round(edgePoints[1])
        };
    }

    arma::vec2 Quad::getEdgePoints(double y) const {
        // create the horizontal intersection line
        ParametricLine<> scanLine;
        scanLine.setFromDirection({1, 0}, {0, y});

        // create a line-segment for each side of the quad
        std::vector<ParametricLine<>> lines = {
            ParametricLine<>(tl, tr, true),
            ParametricLine<>(tr, br, true),
            ParametricLine<>(bl, br, true),
            ParametricLine<>(bl, tl, true)
        };

        // loop through lines and intersect it with the horizontal scan line
        std::vector<double> values;
        for (auto& line : lines) {
            try {
                values.push_back(scanLine.intersect(line)[0]);
            } catch (std::domain_error&) {
                // did not intersect, ignore
            }
        }

        // only two should intersect if there is a solution
        if (values.size() != 2) {
            throw std::domain_error("Could not find the edges points");
        }

        // return the minX and maxX
        return {
            std::min(values[0], values[1]),
            std::max(values[0], values[1])
        };

    }

    std::pair<arma::vec2, arma::vec2> Quad::getIntersectionPoints(Line line) const {

        std::pair<arma::vec2, arma::vec2> points;

        std::vector<ParametricLine<>> quadLines = {
            ParametricLine<>(tl, tr, true),
            ParametricLine<>(tr, br, true),
            ParametricLine<>(bl, br, true),
            ParametricLine<>(bl, tl, true)
        };

        int counter = 0;
        ParametricLine<> pLine(line.pointFromTangentialDistance(0), line.pointFromTangentialDistance(1));
        for (auto& quadLine : quadLines) {
            try {
                if (counter == 0) {
                    points.first = pLine.intersect(quadLine);
                    counter++;
                } else {
                    points.second = pLine.intersect(quadLine);
                    counter++;
                    break;
                }
            } catch (std::domain_error&) {
                // did not intersect, ignore
            }
        }
        if (counter < 2) {
            throw std::domain_error("Quad::intersect - Line does not not intersect quad");
        }

        return points;

    }

    Quad Quad::getBoundingBox(const std::vector<arma::vec2>& points){
        //Check for
        if(points.size() <= 0){
            throw std::domain_error("Request made for bounding box for empty list of points!");
        }

        double min_x = points[0][0];
        double max_x = points[0][0];
        double min_y = points[0][1];
        double max_y = points[0][1];
        for(uint i = 1; i < points.size(); i++){
            auto& p = points[i];
            max_x = std::max(max_x,p[0]);
            min_x = std::min(min_x,p[0]);
            max_y = std::max(max_y,p[1]);
            min_y = std::min(min_y,p[1]);
        }
        return Quad(
                    arma::vec2({min_x,min_y}),
                    arma::vec2({min_x,max_y}),
                    arma::vec2({max_x,max_y}),
                    arma::vec2({max_x,min_y})
                    );
    }



}
}
}
