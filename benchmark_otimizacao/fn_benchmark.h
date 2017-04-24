#ifndef FN_BENCHMARK_
#define FN_BENCHMARK_
/*Nesse cabecalho sao definidas algumas funcoes de teste normalmente usadas para avaliar metodos de otimizacao.*/
#include <Eigen/Dense>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif // !M_PI

#ifndef M_E
#define M_E 2.71828182845904523536
#endif

/*Funcao de Ackley:
Possui diversos minimos locais e pode levar a estagnaçao de metodos do tipo hill-climbing.
Dimensao: d
Dominio: hipercubo xi em  [-32.768, 32.768].
Valores recomendados para os parametros: a = 20, b = 0.2 and c = 2*pi. 
Minimo em x ={0,0,...,0}*/
double ackley(Eigen::VectorXd &x, double a=20, double b=0.2, double c=2*M_PI);

/*Funcao de Schwefel:
Possui muitos minimos locais.
Dimensao : d
Dominio: hipercubo xi em  [-500, 500].*/
double schwefel(Eigen::VectorXd &x);
#endif
