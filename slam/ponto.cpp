#include "ponto.h"
#include <cstdio>
#include <cmath>

ponto::ponto()
{
    //construtor padrao
    x = y = 0.0;
}

ponto::ponto(double x, double y)
{
    this->x = x;
    this->y = y;
}
ponto::ponto(const ponto &p)//construtor de copia
{
    x = p.x;
    y = p.y;
}

ponto ponto::operator=(const ponto & rhs)
{
    return ponto(rhs);
}

ponto ponto::operator+(const ponto & a)
{
    return ponto(x+a.x,y+a.y);
}

ponto &ponto::operator+=(const ponto & rhs)
{
    x += rhs.x;
    y += rhs.y;
    return *this;
}

ponto ponto::operator-(const ponto & a)
{
    return ponto(x-a.x,y-a.y);
}

ponto & ponto::operator-=(const ponto & rhs)
{
    x -= rhs.x;
    y -= rhs.y;
    return *this;
}

ponto ponto::operator/(const double & rhs)
{
    return ponto(x/rhs,y/rhs);
}

ponto & ponto::operator/=(const double & rhs)
{
    x /= rhs;
    y /= rhs;
    return *this;
}

bool ponto::operator==(ponto & rhs)
{
    if (x == rhs.x && y == rhs.y)
        return true;
    else
        return false;
}

bool ponto::operator!=(ponto & rhs)
{
    if (x != rhs.x || y != rhs.y)
        return false;
    else
        return true;
}
//encarando o ponto como um vetor centrado na origem...definamos sua norma
double ponto::norma(void)
{
    //norma L^2
    return std::sqrt(x*x + y*y);
}

double ponto::norma(int p)
{
    //norma L^p
    return std::pow(std::pow(x, p) + std::pow(y, p), 1.0 / ((double)p));//comparar performance..com
}

Eigen::Vector2d ponto::paraVetor(void)
{
    return Eigen::Vector2d(x,y);
}

ponto ponto::polarParaCartesiano(double r, double angulo)
{
    return ponto(r*std::cos(angulo),r*std::sin(angulo));
}
