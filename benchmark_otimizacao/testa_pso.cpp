#include "pso.h"
#include "fn_benchmark.h"
#include <Eigen/Dense>
#include <iostream>
#include <functional>
#include <fstream>
#include <chrono>

using namespace Eigen;
double parabola(VectorXd x,...)//teste simples
{
    return x.squaredNorm();
}
double restricao(VectorXd x)
{
    return 0;
}

int main(int argc, char **argv)
{
    using namespace std::placeholders;
    const int NDIM = 3;
    const int NMAX = 100;
    const double coefMin = 0.1;
    const double coefMax = 2.0;
    const double incCoef = 0.25;
    double erroMedio = 0.0;
    opcoesPSO o;    
    o.limitaVelocidade = true;
    o.maxIter = 100;
    o.numDimensoes = NDIM;
    o.modoDeConfinamento =REFLEXO_DIM;
    o.normalizaValores = false;
    o.numParticulas = 50;
    o.velMax = 1;
    double fval=0.0;
    double limiteInferior[NDIM];
    double limiteSuperior[NDIM];
    VectorXd x(NDIM, 1);
    //ackley
    //cria um log dos resultados
    x.setRandom();
    fval = 0.0;    
    for (int i = 0; i < NDIM; i++) {
        limiteInferior[i] = -32.768;
        limiteSuperior[i] = 32.768;
    }
    std::cout << "\nEnsaio com a funcao de Ackley" << std::endl;
    std::ofstream arqAckley;
    arqAckley.open("teste_pso_ackley.csv", std::ios::out | std::ios::trunc);
    arqAckley << "\ncognitivo,inercia,social,erroMedio,tempoMedio\n";//cabecalho
    std::function<double(Eigen::VectorXd&)> objetivo = std::bind(ackley, _1, 20, 0.2, 2 * M_PI);
    for (o.coefCognitivo = coefMin; o.coefCognitivo< coefMax; o.coefCognitivo+=incCoef){
        for (o.coefInercia = coefMin; o.coefInercia < coefMax  ; o.coefInercia +=incCoef){
            for (o.coefSocial = coefMin; o.coefSocial < coefMax; o.coefSocial += incCoef) {                     
                erroMedio = 0.0;
                for (size_t nRodada = 0; nRodada < NMAX; nRodada++) {
                    x.setRandom();
                    fval = pso_gbest(x, objetivo, o, limiteInferior, limiteSuperior, restricao);
                    //std::cout << "\nfval rodada " << nRodada << "= " << fval;
                    erroMedio += fval;
                }
                erroMedio /= NMAX;                
                auto start = std::chrono::steady_clock::now();
                for (size_t nRodada = 0; nRodada < NMAX; nRodada++) {
                    fval = pso_gbest(x, schwefel, o, limiteInferior, limiteSuperior, restricao);
                }
                auto end = std::chrono::steady_clock::now();
                auto diff = end - start;
                arqAckley << "\n" << o.coefCognitivo << "," << o.coefInercia << "," << o.coefSocial << "," << erroMedio << "," << std::chrono::duration <double, std::milli>(diff).count() / NMAX<<","<<std::endl;
            }
        }

    }
    arqAckley.close();
    //schwefel
    x.setRandom();
    fval = 0.0;
    std::cout << "\nEnsaio com a funcao de Schwefel" << std::endl;
    erroMedio = 0.0;
    VectorXd xMedio(NDIM, 1); xMedio.setZero();
    for (int i = 0; i < NDIM; i++) {
        limiteInferior[i] = -500.0;
        limiteSuperior[i] = 500.0;
    }
    //cria um log dos resultados   
    std::ofstream arqSchwefel;
    arqSchwefel.open("teste_pso_schwefel.csv", std::ios::out | std::ios::trunc);
    arqSchwefel << "\ncognitivo,inercia,social,erroMedio,tempoMedio\n";//cabecalho
    objetivo = schwefel;
    for (o.coefCognitivo = coefMin; o.coefCognitivo< coefMax; o.coefCognitivo += incCoef) {        
        for (o.coefInercia = coefMin; o.coefInercia < coefMax; o.coefInercia += incCoef) {            
            for (o.coefSocial = coefMin; o.coefSocial < coefMax; o.coefSocial += incCoef) {
                erroMedio = 0.0;
                for (size_t nRodada = 0; nRodada < NMAX; nRodada++) {
                    fval = pso_gbest(x, objetivo, o, limiteInferior, limiteSuperior, restricao);
                    //std::cout << "\nfval rodada " << nRodada << "= " << fval;
                    erroMedio += fval;
                }
                erroMedio /= NMAX;                
                auto start = std::chrono::steady_clock::now();
                for (size_t nRodada = 0; nRodada < NMAX; nRodada++) {
                    fval = pso_gbest(x, schwefel, o, limiteInferior, limiteSuperior, restricao);
                }
                auto end = std::chrono::steady_clock::now();
                auto diff = end - start;                
                arqSchwefel << "\n" << o.coefCognitivo << "," << o.coefInercia << "," << o.coefSocial << "," << erroMedio << "," << std::chrono::duration <double, std::milli>(diff).count() / NMAX << "," << std::endl;
            }
        }
    }
    arqSchwefel.close();
    return 0;
}