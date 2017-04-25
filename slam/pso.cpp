#include "pso.h"
#include <cstdlib>
#include <vector>
#include <iostream>//

using namespace Eigen;


inline double rand_uniforme(double min, double max) 
{
    return (std::rand()*(max - min) / RAND_MAX)+min;
}

/**
* Gaussian random with a mean and standard deviation. Uses the
* Polar method of Marsaglia.(INSPIRADO POR G2O)
*/
static double rand_gauss(double mu, double sigma)
{
    double x, y, r2;
    do {
        x = -1.0 + 2.0 * rand_uniforme(0.0, 1.0);
        y = -1.0 + 2.0 * rand_uniforme(0.0, 1.0);
        r2 = x * x + y * y;
    } while (r2 > 1.0 || r2 == 0.0);
    return mu + sigma * y * std::sqrt(-2.0 * log(r2) / r2);
}


double pso_gbest(VectorXd &x, std::function<double(Eigen::VectorXd)> fobj, opcoesPSO &opcoes, double *limiteInferior,
    double *limiteSuperior, std::function<double(Eigen::VectorXd)> fRestricao)
{
    //cria enxame
	MatrixXd posicoes(opcoes.numParticulas, opcoes.numDimensoes); 
	//posicoes *= 0.5;//limita os valores a [0,1]

    MatrixXd pbest(opcoes.numParticulas, opcoes.numDimensoes); 
    VectorXd gbest;// = posicoes.row(0);
    //double *melhoresAptidoes = new double[opcoes.numParticulas];
    std::vector <double> melhoresAptidoes; melhoresAptidoes.reserve(opcoes.numParticulas);


	MatrixXd velocidades(opcoes.numParticulas, opcoes.numDimensoes); //(MatrixXd::Random(opcoes.numParticulas, opcoes.numDimensoes))*opcoes.velMax;//[-velMax,velMax]
	//velocidades.setRandom();
	//velocidades.setRandom();//tentativa de descobrir onde comecam os -nan(ind)
	//velocidades *= opcoes.velMax;
    //cria matrizes de update
    //MatrixXd inercia = MatrixXd::Identity(opcoes.numDimensoes, opcoes.numDimensoes)*op;
    //inicializa todas as particulas    
    int i,iter, idxGbest;//gbest e um indice para a melhor particula do enxame
    register int j;
	double apt,violacaoLim, normaVel, gbestApt,gbestAnterior;
    //completa a inicializacao
    //double *faixasDeValores = new double[opcoes.numDimensoes];

    std::vector <double> faixasDeValores; faixasDeValores.reserve(opcoes.numDimensoes);
    for (j = 0; j < opcoes.numDimensoes; j++) {
        faixasDeValores.push_back(limiteSuperior[j] - limiteInferior[j]);
    }

    //vetorizar essa parte se possivel
    for (i = 0; i < opcoes.numParticulas; i++) {
        for (j = 0; j < opcoes.numDimensoes; j++) {
            //posicoes(i, j) *= faixasDeValores[j];
            //posicoes(i, j) += limiteInferior[j];
            //inicializa a posicao de (i,j)
            //posicoes iniciais uniformemente distribuidas no intervalo
            posicoes(i, j) = rand_uniforme(limiteInferior[j], limiteSuperior[j]);
            //inicializa a velocidade de (i,j)
            //velocidades iniciais normalmente distribuidas com media 0 e variancia 1
            velocidades(i, j) = rand_gauss(0.0, 1.0);
        }
        //inicializa aptidao
        apt = fobj(posicoes.row(i));
        if (i == 0 || apt < gbestApt) {
            gbestAnterior = gbestApt = apt;
            idxGbest = i;
            gbest = posicoes.row(i);
        }
        melhoresAptidoes.push_back(apt);
        pbest.row(i) = posicoes.row(i);//a primeira posicao e necessariamente a melhor historica na primeira iteracao...
    }
    //idxGbest = 0;//na primeira iteracao
    pbest = posicoes;
	const int maxIterEstagnado = opcoes.maxIter >> 1;
	int estagnacao = maxIterEstagnado;
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
            if ((iter == 0 || apt < gbestApt) &&(std::isnormal(apt) || apt==0.0)) {
				gbestAnterior = gbestApt;//para ser possivel conferir se ha estagnacao do processo de busca
                gbestApt = apt;
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
			if (std::abs(gbestApt - gbestAnterior) <= opcoes.tolerancia)
				estagnacao--;//diminui o numero de iteracoes permitidas com o mesmo valor
			else
				estagnacao = maxIterEstagnado;//quebrou-se a estagnacao...reinicie a contagem
           // std::cout << "\niter" << iter << " gbest = " << apt ;//debugging
        }        
    x = gbest;
    //apt = gbestApt;
//    delete[] melhoresAptidoes;
  //  delete[] faixasDeValores;
    //if (std::isnormal(apt))//cuidado com zero (seria ideal...)!!!
        return gbestApt;
    //else
    //    return -1.0;//pela forma como foi definida...fobj (no problema) e nao negativa!
}