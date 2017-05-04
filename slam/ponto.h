#ifndef _PONTO
#define _PONTO
#include <Eigen/Dense>

class ponto {//define um ponto em 2d, num sistema cartesiano
    public:    
       double x, y;
       ponto();
       ponto(double x, double y);
       ponto(const ponto &);
       ponto operator=(const ponto& rhs);
       ponto operator+(const ponto& rhs);//válido?
       ponto &operator+=(const ponto& rhs);
       ponto operator-(const ponto& rhs);//talvez retornar um vetor, overloading?
       ponto &operator-=(const ponto& rhs);
       ponto operator/ (const double& rhs);
       ponto &operator/=(const double& rhs);
       bool operator== (ponto& rhs);
       bool operator!=(ponto& rhs);
       //norma L^2
       double norma(void);
       //norma L^p
       double norma(int p);
       //converte para Eigen::Vector2d
       Eigen::Vector2d paraVetor(void);
       //conveniencia
       static ponto polarParaCartesiano(double r, double angulo);
};

#endif