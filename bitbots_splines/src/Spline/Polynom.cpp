/*
This code is largely based on the original code by Quentin "Leph" Rouxel and Team Rhoban.
The original files can be found at:
https://github.com/Rhoban/model/
*/
#include "bitbots_splines/Polynom.hpp"
#include "bitbots_splines/NewtonBinomial.hpp"

namespace bitbots_splines {
        
Polynom::Polynom() :
    coefs_()
{
}
Polynom::Polynom(unsigned int degree) :
    coefs_()
{
    for (size_t i=0;i<degree+1;i++) {
        coefs_.push_back(0.0);
    }
}

const std::vector<double>& Polynom::getCoefs() const
{
    return coefs_;
}
std::vector<double>& Polynom::getCoefs()
{
    return coefs_;
}
        
const double& Polynom::operator()(size_t index) const
{
    return coefs_.at(index);
}
double& Polynom::operator()(size_t index)
{
    return coefs_.at(index);
}
        
size_t Polynom::degree() const
{
    return coefs_.size()-1;
}

double Polynom::pos(double x) const
{
    double xx = 1.0;
    double val = 0.0;
    for (double coef : coefs_) {
        val += xx*coef;
        xx *= x;
    }
    return val;
}
double Polynom::vel(double x) const
{
    double xx = 1.0;
    double val = 0.0;
    for (size_t i=1; i<coefs_.size(); i++) {
        val += i*xx*coefs_[i];
        xx *= x;
    }
    return val;
}
double Polynom::acc(double x) const
{
    double xx = 1.0;
    double val = 0.0;
    for (size_t i=2; i<coefs_.size(); i++) {
        val += (i-1)*i*xx*coefs_[i];
        xx *= x;
    }
    return val;
}
double Polynom::jerk(double x) const
{
    double xx = 1.0;
    double val = 0.0;
    for (size_t i=3; i<coefs_.size(); i++) {
        val += (i-2)*(i-1)*i*xx*coefs_[i];
        xx *= x;
    }
    return val;
}

void Polynom::operator*=(double coef)
{
    for (double & _coef : coefs_) {
        _coef *= coef;
    }
}
void Polynom::operator+=(const Polynom& p)
{
    while (p.coefs_.size() > coefs_.size()) {
        coefs_.push_back(0.0);
    }

    for (size_t i=0; i<p.coefs_.size(); i++) {
      coefs_[i] += p.coefs_[i];
    }
}
        
void Polynom::shift(double delta)
{
    Polynom n(coefs_.size()-1);
    n.coefs_[0] = coefs_[0];

    for (size_t k=1; k<coefs_.size(); k++) {
        Polynom tmp = NewtonBinomial::expandPolynom(delta, k);
        for (size_t l=0;l<=k;l++) {
            n.coefs_[l] += coefs_[k]*tmp.coefs_[l];
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

