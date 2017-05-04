#ifndef POSE
#define POSE
#include "ponto.h"
#include <Eigen/Dense>

class pose : public ponto {    
    public :
        double angulo;//a componente angular indica a orientacao do versor
        pose();
        pose(double x, double y, double angulo);
        pose(ponto &p, double angulo);
        pose(Eigen::VectorXd &v);
        pose &operator=(const pose &rhs);
        pose operator+(pose rhs);//retorna a composicao da presente pose com uma pose relativa
        pose& operator+=(const pose& rhs);//retorna a composicao da presente pose com uma pose relativa
        pose operator-( pose rhs);//retorna  a diferenca relativa entre a pose atual e outra pose
         pose& operator-=(const pose& rhs);//retorna a composicao da presente pose com uma pose relativa
        pose operator! (void);//retorna a "pose relativa" inversa e.g:!Dab=Dba
        //composicao de poses e pontos
        //ponto operator+(const ponto &rhs);
        friend ponto& operator+=(ponto &rhs,const pose & lhs);
        friend ponto operator+(ponto rhs, const pose &p);
        //friend ponto& operator+=(const pose & lhs,ponto &rhs );
        //friend ponto operator+(const pose &p, ponto &rhs);
};


#endif