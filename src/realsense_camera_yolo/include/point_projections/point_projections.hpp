#ifndef POINT_PROJECTIONS_HPP
#define POINT_PROJECTIONS_HPP

#include <iostream>

class pointProjections
{
    private:
    double A_, B_, C_, D_;
    double a_, b_, c_;
    double x0_, y0_, z0_;

    public:
    pointProjections(double A, double B, double C, double D, double a, double b, double c, double x0, double y0, double z0);


}

pointProjections::pointProjections(double A, double B, double C, double D, double a, double b, double c, double x0, double y0, double z0)
{
    A_ = A;
    B_ = B;
    C_ = C;
    D_ = D;
    a_ = a;
    b_ = b;
    c_ = c;
    x0_ = x0;
    y0_ = y0;
    z0_ = z0;
}


#endif // point_projections_HPP
