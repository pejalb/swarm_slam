#include "fn_benchmark.h"
#include <cmath>
double ackley(Eigen::VectorXd  &x, double a, double b, double c)
{
    double fval = 0.0;
    int dim = x.size();        
    fval -= a*std::exp(-b*std::sqrt(x.squaredNorm() / (double)dim));
    double total = 0.0;
    for (int i = 0; i < dim; i++)//ineficiente mas as reducoes do Eigen estavam apresentando incompatibilidades
        total += std::cos(c*x(i));
    total /= (double) dim;
    fval += -std::exp(total) + a + M_E;
    return fval;
}

double schwefel(Eigen::VectorXd & x)
{
    int dim = x.size();
    double fval = 418.9829*dim;
    for (int i = 0; i < dim; i++)
        fval -= x(i)*std::sin(std::sqrt(std::abs(x(i))));
    return fval;
}
