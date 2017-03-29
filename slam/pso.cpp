#include "pso.h"
#include <cstdlib>
#include <vector>
#include <iostream>//

#define DT 1
using namespace Eigen;


inline double rand_uniforme(double min, double max) 
{
    return (std::rand()*(max - min) / RAND_MAX)+min;
}


double pso_gbest(VectorXd &x, std::function<double(Eigen::VectorXd)> fobj, opcoesPSO &opcoes, double *limiteInferior,
    double *limiteSuperior, std::function<double(Eigen::VectorXd)> fRestricao)
{
    //cria enxame
    MatrixXd posicoes = 0.5*(MatrixXd::Random(opcoes.numParticulas, opcoes.numDimensoes) + MatrixXd::Constant(opcoes.numParticulas, opcoes.numDimensoes, 0.5));//[0,1]
    MatrixXd pbest; pbest = posicoes;
    VectorXd gbest = posicoes.row(0);
    //double *melhoresAptidoes = new double[opcoes.numParticulas];
    std::vector <double> melhoresAptidoes; melhoresAptidoes.reserve(opcoes.numParticulas);

    MatrixXd velocidades = (MatrixXd::Random(opcoes.numParticulas, opcoes.numDimensoes))*opcoes.velMax;//[-velMax,velMax]

    //cria matrizes de update
    //MatrixXd inercia = MatrixXd::Identity(opcoes.numDimensoes, opcoes.numDimensoes)*op;
    //inicializa todas as particulas    
    int i, iter, idxGbest;//gbest e um indice para a melhor particula do enxame
    register int j;

    //completa a inicializacao
    //double *faixasDeValores = new double[opcoes.numDimensoes];

    std::vector <double> faixasDeValores; faixasDeValores.reserve(opcoes.numDimensoes);
    for (j = 0; j < opcoes.numDimensoes; j++) {
        faixasDeValores.push_back(limiteSuperior[j] - limiteInferior[j]);
    }
    //inicializa aptidoes
    for (j = 0; j < opcoes.numParticulas; j++) {
        melhoresAptidoes.push_back(INFINITY);
    }
    //vetorizar essa parte se possivel
    for (i = 0; i < opcoes.numParticulas; i++) {
        for (j = 0; j < opcoes.numDimensoes; j++) {
            posicoes(i, j) *= faixasDeValores[j];
            posicoes(i, j) += limiteInferior[j];
        }
    }
    idxGbest = 0;//na primeira iteracao
    double apt, violacaoLim, normaVel,gbestAnterior=INFINITY;

    int estagnacao = opcoes.maxIter >> 1;
    //o algoritmo tradicional, possibilita que haja modificao do melhor historico...
    //inicia processo
    for (iter = 0; (iter < opcoes.maxIter) && (estagnacao>0); iter++) {
        //avalia a a qualidade das solucoes de cada particula
        for (j = 0; j < opcoes.numParticulas; j++) {
            apt = fobj(posicoes.row(j)) + fRestricao(posicoes.row(j));
            //apt = fRestricao == NULL ? fobj(posicoes.row(j)) : fobj(posicoes.row(j)) + fRestricao(posicoes.row(j));            
            if ((iter == 0 || apt < melhoresAptidoes[j] && j != idxGbest) && std::isnormal(apt)) {
                melhoresAptidoes[j] = apt;
                pbest.row(j) = posicoes.row(j);
            }
            if (iter == 0 && j == 0) {
                idxGbest = 0;
                gbest = posicoes.row(0);
            }
            if ((iter == 0 || apt < melhoresAptidoes[idxGbest] && apt < gbestAnterior) &&std::isnormal(apt)) {
                gbestAnterior = melhoresAptidoes[idxGbest];
                melhoresAptidoes[j] = apt;
                gbest = posicoes.row(j);
                idxGbest = j;
                pbest.row(j) = posicoes.row(j);
            }
        }
            //atualiza velocidades
            velocidades += opcoes.coefInercia*velocidades +
                opcoes.coefCognitivo*rand_uniforme(0, 1)*(pbest - posicoes) +
                opcoes.coefSocial*rand_uniforme(0, 1)*(gbest.transpose().replicate(opcoes.numParticulas, 1) - posicoes);
            //limita as velocidades        
            if (opcoes.limitaVelocidade) {
                for (i = 0; i < opcoes.numParticulas; i++) {
                    normaVel = velocidades.row(i).norm();
                    if (normaVel>0 && normaVel > opcoes.velMax && !std::isnan(normaVel) && !std::isinf(normaVel)) {
                        velocidades.row(i).normalize();
                        velocidades.row(i) *= opcoes.velMax;
                    }
                    else {
                        velocidades.row(i).Random(1,opcoes.numDimensoes)*opcoes.velMax;//se for invalido sorteia um valor
                    }
                }
            }
            //atualiza posicoes
            posicoes = posicoes + velocidades;
            //garante confinamento
            switch (opcoes.modoDeConfinamento){
            case BLOQUEIO:
                for (i = 0; i < opcoes.numParticulas; i++) {
                    for (j = 0; j < opcoes.numDimensoes; j++) {
                        if (posicoes(i, j) < limiteInferior[j]) {
                            posicoes(i, j) = limiteInferior[j];
                        }
                        if (posicoes(i, j) > limiteSuperior[j]) {
                            posicoes(i, j) = limiteSuperior[j];
                        }
                    }
                }
                break;
            case REFLEXO_DIM:
                //a dimensao que violar a restricao sera refletida
                for (i = 0; i < opcoes.numParticulas; i++) {
                    for (j = 0; j < opcoes.numDimensoes; j++) {
                        if (posicoes(i, j) < limiteInferior[j]) {
                            violacaoLim = limiteInferior[j] - posicoes(i, j);
                            posicoes(i, j) = limiteInferior[j] + violacaoLim;
                        }
                        if (posicoes(i, j) > limiteSuperior[j]) {
                            violacaoLim = posicoes(i, j) - limiteSuperior[j];
                            posicoes(i, j) = limiteSuperior[j] - violacaoLim;
                        }
                    }
                }
                break;
            }           
            if (std::abs(melhoresAptidoes[idxGbest] - gbestAnterior) <= opcoes.tolerancia)
				estagnacao--;                
           // std::cout << "\niter" << iter << " gbest = " << apt ;//debugging
        }        
    x = posicoes.row(idxGbest);    
    apt = melhoresAptidoes[idxGbest];
//    delete[] melhoresAptidoes;
  //  delete[] faixasDeValores;
    if (std::isnormal(apt))//cuidado com zero (seria ideal...)!!!
        return apt;
    else
        return -1.0;//pela forma como foi definida...fobj (no problema) e nao negativa!
}