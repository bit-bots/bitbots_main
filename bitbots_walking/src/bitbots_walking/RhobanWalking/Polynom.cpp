#include "Polynom.hpp"

namespace Leph {
        
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
    for (size_t i=0;i<_coefs.size();i++) {
        val += xx*_coefs[i];
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

void Polynom::operator*=(double coef)
{
    for (size_t i=0;i<_coefs.size();i++) {
        _coefs[i] *= coef;
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
        
std::ostream& operator<<(std::ostream& os, const Polynom& p)
{
    os << "degree=" << p.degree() << " ";
    for (size_t i=0;i<p.degree()+1;i++) {
        os << p(i) << " ";
    }

    return os;
}

}

