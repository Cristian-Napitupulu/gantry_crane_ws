#ifndef POINT_PROJECTIONS_HPP
#define POINT_PROJECTIONS_HPP

// #include <cmath>

class pointProjections
{
    private:
    double A_, B_, C_, D_; // Plane parameters from equation: Ax + By + Cz + D = 0
    double a_, b_, c_; // 3D Line parameters from equation: <x, y, z> = <x_1, y_1, z_1> + t<a, b, c>
    double x0_, y0_, z0_; // New origin of the coordinate system (troley frame)

    double normalPlaneUnitVectorX, normalPlaneUnitVectorY, normalPlaneUnitVectorZ;

    public:
    pointProjections(double A, double B, double C, double D, double a, double b, double c, double x0, double y0, double z0);
    double projectPointAndGetDistance(double x, double y, double z);
    double projectPointAndGetAngle(double x, double y, double z);


};

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

    normalPlaneUnitVectorX = A_ / sqrt(A_*A_ + B_*B_ + C_*C_);
    normalPlaneUnitVectorY = B_ / sqrt(A_*A_ + B_*B_ + C_*C_);
    normalPlaneUnitVectorZ = C_ / sqrt(A_*A_ + B_*B_ + C_*C_);
}

double pointProjections::projectPointAndGetDistance(double x, double y, double z)
{
    double dummyPointX = 0;
    double dummyPointY = 0;
    double dummyPointZ = -(A_*dummyPointX + B_*dummyPointY + D_) / C_;

    double dummyVectorX = dummyPointX - x;
    double dummyVectorY = dummyPointY - y;
    double dummyVectorZ = dummyPointZ - z;

    double dotProduct = dummyVectorX*normalPlaneUnitVectorX + dummyVectorY*normalPlaneUnitVectorY + dummyVectorZ*normalPlaneUnitVectorZ;

    double projectionVectorX = dummyVectorX - dotProduct*normalPlaneUnitVectorX;
    double projectionVectorY = dummyVectorY - dotProduct*normalPlaneUnitVectorY;
    double projectionVectorZ = dummyVectorZ - dotProduct*normalPlaneUnitVectorZ;

    x = x + projectionVectorX;
    y = y + projectionVectorY;
    z = z + projectionVectorZ;

    double deltaX = x - x0_;
    double deltaY = y - y0_;
    double deltaZ = z - z0_;

    double distance = sqrt(deltaX*deltaX + deltaY*deltaY + deltaZ*deltaZ);

    return distance;
}

double pointProjections::projectPointAndGetAngle(double x, double y, double z)
{
    double dummyPointX = 0;
    double dummyPointY = 0;
    double dummyPointZ = -(A_*dummyPointX + B_*dummyPointY + D_) / C_;

    double dummyVectorX = dummyPointX - x;
    double dummyVectorY = dummyPointY - y;
    double dummyVectorZ = dummyPointZ - z;

    double dotProduct = dummyVectorX*normalPlaneUnitVectorX + dummyVectorY*normalPlaneUnitVectorY + dummyVectorZ*normalPlaneUnitVectorZ;

    double projectionVectorX = dummyVectorX - dotProduct*normalPlaneUnitVectorX;
    double projectionVectorY = dummyVectorY - dotProduct*normalPlaneUnitVectorY;
    double projectionVectorZ = dummyVectorZ - dotProduct*normalPlaneUnitVectorZ;

    x = x + projectionVectorX;
    y = y + projectionVectorY;
    z = z + projectionVectorZ;

    double vectorX = x - x0_;
    double vectorY = y - y0_;
    double vectorZ = z - z0_;

    double angle = acos((vectorX*a_ + vectorY*b_ + vectorZ*c_) / (sqrt(vectorX*vectorX + vectorY*vectorY + vectorZ*vectorZ)*sqrt(a_*a_ + b_*b_ + c_*c_) ));

    if (x < 0)
    {
        angle = -angle;
    }

    return angle;
}

#endif // point_projections_HPP
