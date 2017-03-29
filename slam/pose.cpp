#include "pose.h"
#include <cmath>
pose::pose():ponto()
{
    angulo = 0.0;
}
pose::pose(double x, double y, double angulo) : ponto(x,y)
{
    this->angulo = angulo;
}

pose::pose(ponto & p, double angulo):ponto(p)
{
    this->angulo = angulo;
}

pose::pose(Eigen::VectorXd &v):ponto(v(0),v(1))
{
    this->angulo = v(2);
}

pose & pose::operator=(const pose & rhs)
{
    x = rhs.x;
    y = rhs.y;
    angulo = rhs.angulo;
    return *this;
}

pose pose::operator+(const pose & rhs)
{
    pose resultado(0, 0, 0);
    resultado.x = x + rhs.x*std::cos(angulo) - rhs.y*std::sin(angulo);
    resultado.y = y + rhs.x*std::sin(angulo) + rhs.y*std::cos(angulo);
    resultado.angulo = angulo + rhs.angulo;
    return resultado;
}

pose& pose::operator+=(const pose & rhs)
{
    x = x + rhs.x*std::cos(angulo) - rhs.y*std::sin(angulo);
    y = y + rhs.x*std::sin(angulo) + rhs.y*std::cos(angulo);
    angulo = angulo + rhs.angulo;
    return (*this);
}

pose pose::operator-(const pose & rhs)
{
    pose resultado(0, 0, 0);
    resultado.x = (x - rhs.x)*std::cos(rhs.angulo) + (y - rhs.y)*std::sin(rhs.angulo);
    //y=-(xa-xb)*sin(thb)+(ya-yb)*cos(thb);
    resultado.y = (rhs.x - x)*std::sin(rhs.angulo) + (y - rhs.y)*std::cos(rhs.angulo);
    resultado.angulo = angulo - rhs.angulo;
    return resultado;
}

pose & pose::operator-=(const pose & rhs)
{
    x = (x - rhs.x)*std::cos(rhs.angulo) + (y - rhs.y)*std::sin(rhs.angulo);
    //y=-(xa-xb)*sin(thb)+(ya-yb)*cos(thb);
    y = (rhs.x - x)*std::sin(rhs.angulo) + (y - rhs.y)*std::cos(rhs.angulo);
    angulo = angulo - rhs.angulo;
    return (*this);
}

pose pose::operator!(void)
{
    pose resultado(0,0,0);
    return resultado - (*this);
}

ponto pose::operator+(const ponto & rhs)
{
    ponto resultado(0, 0);
    resultado.x = x + rhs.x*std::cos(angulo) - rhs.y*std::sin(angulo);
    resultado.y = y + rhs.x*std::sin(angulo) + rhs.y*std::cos(angulo);
    return resultado;
}