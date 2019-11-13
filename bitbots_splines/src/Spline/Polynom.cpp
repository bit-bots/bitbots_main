/*
This code is largely based on the original code by Quentin "Leph" Rouxel and Team Rhoban.
The original files can be found at:
https://github.com/Rhoban/model/
*/
#include "bitbots_splines/Polynom.hpp"
#include "bitbots_splines/NewtonBinomial.hpp"

namespace bitbots_splines {
        
Polynom::Polynom() :
    _coefs()
{
}
Polynom::Polynom(unsigned int degree) :
    _coefs()
{
    for (size_t i=0;i<degree+1;i++) {
        _coefs.push_back(0.0);
    }
}

const std::vector<double>& Polynom::getCoefs() const
{
    return _coefs;
}
std::vector<double>& Polynom::getCoefs()
{
    return _coefs;
}
        
const double& Polynom::operator()(size_t index) const
{
    return _coefs.at(index);
}
double& Polynom::operator()(size_t index)
{
    return _coefs.at(index);
}
        
size_t Polynom::degree() const
{
    return _coefs.size()-1;
}

double Polynom::pos(double x) const
{
    double xx = 1.0;
    double val = 0.0;
    for (double _coef : _coefs) {
        val += xx*_coef;
        xx *= x;
    }
    return val;
}
double Polynom::vel(double x) const
{
    double xx = 1.0;
    double val = 0.0;
    for (size_t i=1;i<_coefs.size();i++) {
        val += i*xx*_coefs[i];
        xx *= x;
    }
    return val;
}
double Polynom::acc(double x) const
{
    double xx = 1.0;
    double val = 0.0;
    for (size_t i=2;i<_coefs.size();i++) {
        val += (i-1)*i*xx*_coefs[i];
        xx *= x;
    }
    return val;
}
double Polynom::jerk(double x) const
{
    double xx = 1.0;
    double val = 0.0;
    for (size_t i=3;i<_coefs.size();i++) {
        val += (i-2)*(i-1)*i*xx*_coefs[i];
        xx *= x;
    }
    return val;
}

void Polynom::operator*=(double coef)
{
    for (double & _coef : _coefs) {
        _coef *= coef;
    }
}
void Polynom::operator+=(const Polynom& p)
{
    while (p._coefs.size() > _coefs.size()) {
        _coefs.push_back(0.0);
    }

    for (size_t i=0;i<p._coefs.size();i++) {
        _coefs[i] += p._coefs[i];
    }
}
        
void Polynom::shift(double delta)
{
    Polynom n(_coefs.size()-1);
    n._coefs[0] = _coefs[0];

    for (size_t k=1;k<_coefs.size();k++) {
        Polynom tmp = NewtonBinomial::expandPolynom(delta, k);
        for (size_t l=0;l<=k;l++) {
            n._coefs[l] += _coefs[k]*tmp._coefs[l];
        }
    }

    *this = n;
}

std::ostream& operator<<(std::ostream& os, const Polynom& p)
{
    os << "degree=" << p.degree() << " ";
    for (size_t i=0;i<p.degree()+1;i++) {
        os << p(i) << " ";
    }

    return os;
}

}

