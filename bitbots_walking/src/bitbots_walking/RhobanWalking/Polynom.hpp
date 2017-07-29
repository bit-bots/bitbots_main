#ifndef LEPH_POLYNOM_HPP
#define LEPH_POLYNOM_HPP

#include <cstdlib>
#include <vector>
#include <iostream>

namespace Leph {

/**
 * Polynom
 *
 * Simple one dimentional 
 * polynom class for spline 
 * generation
 */
class Polynom
{
    public:

        /**
         * Default and inital degree initialization
         */
        Polynom();
        Polynom(unsigned int degree);

        /**
         * Access to coefficient
         * indexed from constant to
         * higher degree
         */
        const std::vector<double>& getCoefs() const;
        std::vector<double>& getCoefs();

        /**
         * Access to coefficient
         */
        const double& operator()(size_t index) const;
        double& operator()(size_t index);

        /**
         * Return polynom degree
         * -1 mean empty polynom
         */
        size_t degree() const;

        /**
         * Polynom evaluation, its first and
         * second derivative at given x
         */
        double pos(double x) const;
        double vel(double x) const;
        double acc(double x) const;

        /**
         * Some useful operators
         */
        void operator*=(double coef);
        void operator+=(const Polynom& p);

    private:

        /**
         * Polynom coeficients
         */
        std::vector<double> _coefs;
};

/**
 * Print operator
 */
std::ostream& operator<<(std::ostream& os, const Polynom& p);

}

#endif

