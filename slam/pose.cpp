#include "pose.h"
#include "constantes.h"
#include <cmath>

inline double limitaAngulo(double x) {
    x = std::fmod(x + M_PI, M_2PI);
    if (x < 0)
        x += M_2PI;
    return x - M_PI;
}

pose::pose():ponto()
{
    angulo = 0.0;
}
pose::pose(double x, double y, double angulo) : ponto(x,y)
{
    this->angulo = limitaAngulo(angulo);
}

pose::pose(ponto & p, double angulo):ponto(p)
{
    this->angulo = limitaAngulo(angulo);
}

pose::pose(Eigen::VectorXd &v)
{
    x = v(0);
    y = v(1);
    angulo = limitaAngulo(v(2));
}

pose & pose::operator=(const pose & rhs)
{
    x = rhs.x;
    y = rhs.y;
    angulo = limitaAngulo(rhs.angulo);
    return *this;
}

pose& pose::operator+=(const pose & rhs)
{
    x = x + rhs.x*std::cos(angulo) - rhs.y*std::sin(angulo);
    y = y + rhs.x*std::sin(angulo) + rhs.y*std::cos(angulo);
    angulo = limitaAngulo(angulo + rhs.angulo);
    return (*this);
}

pose pose::operator+(pose rhs)
{    
    return rhs+=(*this);
}

pose & pose::operator-=(const pose & rhs)
{
    x = (x - rhs.x)*std::cos(rhs.angulo) + (y - rhs.y)*std::sin(rhs.angulo);
    //y=-(xa-xb)*sin(thb)+(ya-yb)*cos(thb);
    y = (rhs.x - x)*std::sin(rhs.angulo) + (y - rhs.y)*std::cos(rhs.angulo);
    angulo = limitaAngulo(angulo - rhs.angulo);
    return (*this);
}

pose pose::operator-(pose rhs)
{
    return rhs-=(*this);
}

pose pose::operator!(void)
{
    pose resultado(0,0,0);
    return resultado - (*this);
}

ponto & operator+=( ponto & rhs,const pose & lhs )
{
    rhs.x=lhs.x + rhs.x*std::cos(lhs.angulo) - rhs.y*std::sin(lhs.angulo);
    rhs.y=lhs.y + rhs.x*std::sin(lhs.angulo) + rhs.y*std::cos(lhs.angulo);
    return rhs;
}

ponto operator+(ponto rhs, const pose & p)
{
    return rhs += p;
}

/*
inline ponto & operator+=(const pose & lhs, ponto & rhs)
{
    return rhs += lhs;
}

inline ponto operator+(const pose & p, ponto & rhs)
{
    return rhs+p;
}
*/
