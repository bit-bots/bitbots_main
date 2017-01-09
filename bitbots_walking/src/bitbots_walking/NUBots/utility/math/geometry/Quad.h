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

#ifndef UTILITY_MATH_GEOMETRY_QUAD_H
#define UTILITY_MATH_GEOMETRY_QUAD_H

#include <armadillo>
#include <ostream>
#include <vector>

#include "Line.h"

namespace utility {
namespace math {
namespace geometry {

    class Quad {
    public:
        Quad();
        Quad(const Quad& other);
        Quad(arma::vec2 bottomLeft, arma::vec2 topLeft, arma::vec2 topRight, arma::vec2 bottomRight);
        Quad(arma::ivec2 bottomLeft, arma::ivec2 topLeft, arma::ivec2 topRight, arma::ivec2 bottomRight);
        Quad(double left, double top, double right, double bottom);

        /**
         * Sets the Quad as a screen aligned rectangle given the specified positions.
         * @param left     The left x pixel value.
         * @param top      The top y pixel value.
         * @param right    The right x pixel value.
         * @param bottom   The bottom y pixel value.
         */
        void set(double left, double top, double right, double bottom);

        /**
         * Sets the Quad given the specified corners.
         * @param bottomLeft  The bottom left corner.
         * @param topLeft     The top left corner.
         * @param topRight    The top right corner.
         * @param bottomRight The bottom right corner.
         */
        void set(arma::vec2 bottomLeft, arma::vec2 topLeft, arma::vec2 topRight, arma::vec2 bottomRight);

        arma::vec2 getTopCentre() const;                                //! Returns the bottom centre pixel location of the Quad.
        arma::vec2 getBottomCentre() const;                             //! Returns the bottom centre pixel location of the Quad.
        arma::vec2 getRightCentre() const;
        arma::vec2 getLeftCentre() const;

        arma::vec2 getCentre() const;                                   //! Returns the centre pixel location  of the Quad.

        arma::vec2 getBottomLeft() const;                               //! Returns the bottom left pixel location  of the Quad.
        arma::vec2 getBottomRight() const;                              //! Returns the bottom right pixel location  of the Quad.
        arma::vec2 getTopLeft() const;                                  //! Returns the top left pixel location  of the Quad.
        arma::vec2 getTopRight() const;                                 //! Returns the top right pixel location  of the Quad.

        arma::vec2 getSize() const;                                     //Returns the bounding box width and height

        double getLeft() const;
        double getRight() const;
        double getTop() const;
        double getBottom() const;

        int getBaseWidth() const;                                       //! Returns the base width of the Quad in pixels.
        int getTopWidth() const;                                        //! Returns the top width of the Quad in pixels.

        int getLeftHeight() const;                                      //! Returns the left height of the Quad in pixels.
        int getRightHeight() const;                                     //! Returns the right height of the Quad in pixels.

        double getAverageWidth() const;                                 //! Returns the average width of the Quad in pixels.
        double getAverageHeight() const;                                //! Returns the average height of the Quad in pixels.

        double area() const;
        double aspectRatio() const;

        std::vector<arma::vec2> getVertices() const;

        bool overlapsHorizontally(const Quad& other) const;

        bool checkCornersValid() const;

        /**
         * Finds and returns the two rounded intersections points on x given a y
         * @param y The horizonal line to solve the 2 x-axis intersections with
         * @return The minX and maxX rounded that intersect given y
         */
        arma::vec2 getEdgePoints(uint y) const;

        std::pair<arma::vec2, arma::vec2> getIntersectionPoints(Line line) const;

        /**
         * Finds and returns the two intersections points on x given a y
         * @param y The horizonal line to solve the 2 x-axis intersections with
         * @return The minX and maxX that intersect given y
         */
        arma::vec2 getEdgePoints(double y) const;

        static Quad getBoundingBox(const std::vector<arma::vec2>& points);

    private:
        arma::vec2 bl;                                                  //! @variable The bottom-left of the Quad.
        arma::vec2 br;                                                  //! @variable The bottom-right of the Quad.
        arma::vec2 tr;                                                  //! @variable The top-right of the Quad.
        arma::vec2 tl;                                                  //! @variable The top-left of the Quad.

        //! @brief output stream operator.
    friend std::ostream& operator<< (std::ostream& output, const Quad& quad);

    //! @brief output stream operator for a vector of goals.
    friend std::ostream& operator<< (std::ostream& output, const std::vector<Quad>& quads);
    };
}
}
}

#endif
