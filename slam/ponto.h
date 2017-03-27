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
	   //converte um ponto especificado por sua distancia a origem e o angulo que a reta que o intercepta juntamente com a origem forma com o eixo das abcissas, conforme medido no sentido anti-horario (em radianos)
	   static ponto polarParaCartesiano(double raio, double angulo);
};

#endif