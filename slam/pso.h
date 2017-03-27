#ifndef PSO
#define PSO
#include <Eigen/Dense>
#include "ponto.h"
#include <functional>
/*A implementacao feita abaixo, aceita receber tipos genericos,
mas eles devem possuir operadores que lhes permitam se comportar como um numero em ponto flutuante.

O ponteiro para a funcao de restricao pode, se fornecido, deve retornar valores tao mais altos quanto mais serias forem as violacoes. A penalidade funciona de modo aditivo

O ponteiro para funcao fConversao, deve receber um vetor de doubles e retornar um objeto do tipo T a ser avaliado. se T for um tipo padrao, essa funcao não precisa ser fornecida
Os vetores limiteInferior e limiteSuperior.*/

/* reflexo, a particula e lancada de volta ao espaco de buscas com a dimensao que violou a restricao refletida,
bloqueio ela permanece no bordo que violou*/
enum confinamento { REFLEXO_DIM = 0, BLOQUEIO = 1 };

struct opcoesPSO {
    int numParticulas,maxIter,numDimensoes;
    double coefInercia, coefCognitivo, coefSocial,tolerancia,velMax;
    int modoDeConfinamento;//como lidar com particulas que escape do espaco de busca
    bool limitaVelocidade;//as velocidades devem ser restritas ou nao?
    bool normalizaValores;//os valores de aptidao devem ser normalizados no enxame ou nao
};

double pso_gbest(Eigen::VectorXd &x, std::function<double(Eigen::VectorXd)> fobj,opcoesPSO &opcoes,
    double *limiteInferior,double *limiteSuperior, std::function<double(Eigen::VectorXd)> fRestricao);
#endif