#ifndef POSE
#define POSE
#include "ponto.h"
#include <Eigen/Dense>
#include <vector>

class pose : public ponto {    
    public :
        double angulo;//a componente angular indica a orientacao do versor
        pose();
		pose(const pose &);
        pose(double x, double y, double angulo);
        pose(ponto &p, double angulo);
        pose(Eigen::VectorXd &v);//bugged
        pose &operator=(const pose &rhs);
        pose operator+(const pose& rhs);//retorna a composicao da presente pose com uma pose relativa
        pose& operator+=(const pose& rhs);//retorna a composicao da presente pose com uma pose relativa
        pose operator-(const pose& rhs);//retorna  a diferenca relativa entre a pose atual e outra pose
         pose& operator-=(const pose& rhs);//retorna a composicao da presente pose com uma pose relativa
        pose operator! (void);//retorna a "pose relativa" inversa e.g:!Dab=Dba
        //composicao de poses e pontos
        friend ponto operator+(const pose &lhs,ponto rhs);
		friend ponto operator+(ponto lhs,const pose &rhs);
};

inline void transforma_vetor_pontos(std::vector<ponto>& scan, pose &p);

#endif