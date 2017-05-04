#include "scanMatch.h"
#include <Eigen/Dense>
#include <cstdarg>
#include <functional>
#include <iostream>//debugging
#include "pointcloud_kd_tree.h"

#ifndef M_PI
#define 	M_PI   3.14159265358979323846
#endif

#define DX 1.0
#define DY 1.0
#define D_ANG M_PI
inline double rand_uniforme(double min, double max)
{
    return (std::rand()*(max - min) / RAND_MAX) + min;
}

Eigen::Transform<double, 2, Eigen::Affine> cria_transformacao(double angulo, double transX, double transY)
{
    /*cria uma transformacao com rotacao seguida de translacao*/
    Eigen::Rotation2D<double> Rot(angulo);
    Eigen::Translation<double, 2> translacao(Eigen::Vector2d(transX, transY));
    Eigen::Transform<double, 2, Eigen::Affine> T = Rot*translacao;
    return T;
}

double fobj(Eigen::VectorXd v, std::vector<ponto> & scanOrigem, std::vector<ponto> & scanDestino, std::vector<std::vector<size_t> > &idx)
{
    //std::cout << "\nv =" << v(0) << "," << v(1) << "," << v(2)<<std::endl;//debugging -nan(ind)
    pose p(v);
    double erro = 0.0;
    //PointCloud<double> cloud; cloud.pts = scanDestino;
    //double query[2] = { scanOrigem[0].x,scanOrigem[0].y };
    //std::vector<std::vector<size_t> > idx=constroiKDtree<double>(cloud, scanOrigem,1);
    int numPontos = idx.size();
    for (size_t i = 0; i < numPontos; i += 1) {
        for (size_t j = 0; j < idx[i].size(); j++) {
            erro += (scanDestino[idx[i][j]] - (scanOrigem[i] + p)).norma();
        }
    }
    //std::cout << "\nerro = " << erro << std::endl;
    return std::sqrt(erro / ((double)numPontos));
}


inline double fRestricaoPadrao(Eigen::VectorXd v)
{
    return 0.0;//se a restricao nao-linear nao foi fornecida...usa um "placeholder"
}

pose psoScanMatch(std::vector<ponto> & scanOrigem, std::vector<ponto> & scanDestino,double estimativaInicial[3])
{
    using namespace std::placeholders;
    //assert(scanOrigem.size() == scanDestino.size());
    ////normaliza os scans
    //int numPontos = scanOrigem.size();
    //ponto centro_scanOrigem(0, 0), centro_scanDestino(0, 0);
    //for (int i = 0; i<numPontos; ++i)
    //{
    //    centro_scanOrigem += scanOrigem[i];
    //    centro_scanDestino += scanDestino[i];
    //}
    //centro_scanOrigem /= (double)numPontos;
    //centro_scanDestino /= (double)numPontos;
    ///*double pso_gbest(VectorXd &x, double(*fobj)(VectorXd,...), opcoesPSO &opcoes, double *limiteInferior,
    //double *limiteSuperior, double (*fConversao)(double *), double(*fRestricao)(VectorXd))*/,
    Eigen::VectorXd x = Eigen::VectorXd::Zero(3, 1);
    opcoesPSO opcoes;
    opcoes.coefCognitivo = 0.7;
    opcoes.coefInercia = 0.9;
    opcoes.coefSocial = 0.7;
    opcoes.limitaVelocidade = true;
    opcoes.maxIter = 100;
    opcoes.numDimensoes = 3;
    opcoes.modoDeConfinamento = BLOQUEIO;
    opcoes.normalizaValores = false;
    opcoes.numParticulas = 150;
    opcoes.velMax = 0.05;
    opcoes.tolerancia = 0.1;
    double limiteInferior[3] = { estimativaInicial[0]-DX,estimativaInicial[1]-DY,estimativaInicial[2]-D_ANG };
    double limiteSuperior[3] = { estimativaInicial[0] + DY,estimativaInicial[1] + DY,estimativaInicial[2] + D_ANG };
    //ponto center_src(0, 0);
    //ponto center_dst(0, 0);
    int tamanho = scanOrigem.size();

    PointCloud<double> cloud; cloud.pts = scanDestino;
    std::vector<std::vector<size_t> > idx = constroiKDtree<double>(cloud, scanOrigem, 2);
    //normaliza a nuvem de pontos
    //dados os centros a determinacao da translacao entre as nuvens e trivial.
    /*for (int i = 0; i<tamanho; ++i)
    {
        center_src += scanOrigem[i];
        center_dst += scanDestino[i];
    }
    center_src /= (double)tamanho;
    center_dst /= (double)tamanho;*/

    std::function<double(Eigen::VectorXd)> objetivo= 
        std::bind(fobj,_1,scanOrigem, scanDestino,idx);//wrapper para a fobj, requer apenas a transformacao
    std::function<double(Eigen::VectorXd)> restricao = fRestricaoPadrao;
        double erro = pso_gbest(x, objetivo, opcoes, limiteInferior, limiteSuperior, restricao);
    return pose(x);
}

pose psoScanMatch(std::vector<ponto>& scanOrigem, std::vector<ponto>& scanDestino, double estimativaInicial[3],
    std::vector<pose>& outrasPoses, double(*fRestricao)(Eigen::VectorXd, std::vector<pose>&))
{
    using namespace std::placeholders;
    Eigen::VectorXd x = Eigen::VectorXd::Zero(3, 1);
    opcoesPSO opcoes;
    opcoes.coefCognitivo = 0.7;
    opcoes.coefInercia = 0.9;
    opcoes.coefSocial = 0.7;
    opcoes.limitaVelocidade = true;
    opcoes.maxIter = 100;
    opcoes.numDimensoes = 3;
    opcoes.modoDeConfinamento = BLOQUEIO;
    opcoes.normalizaValores = false;
    opcoes.numParticulas = 25;
    opcoes.velMax = 0.05;
    opcoes.tolerancia = 0.1;
    double limiteInferior[3] = { estimativaInicial[0] - DX,estimativaInicial[1] - DY,estimativaInicial[2] - D_ANG };
    double limiteSuperior[3] = { estimativaInicial[0] + DY,estimativaInicial[1] + DY,estimativaInicial[2] + D_ANG };
    //ponto center_src(0, 0);
    //ponto center_dst(0, 0);
    int tamanho = scanOrigem.size();
    PointCloud<double> cloud; cloud.pts = scanDestino;
    std::vector<std::vector<size_t> > idx = constroiKDtree<double>(cloud, scanOrigem, 2);
    //normaliza a nuvem de pontos
    //dados os centros a determinacao da translacao entre as nuvens e trivial.
    /*for (int i = 0; i<tamanho; ++i)
    {
    center_src += scanOrigem[i];
    center_dst += scanDestino[i];
    }
    center_src /= (double)tamanho;
    center_dst /= (double)tamanho;*/

    std::function<double(Eigen::VectorXd)> objetivo =
        std::bind(fobj, _1, scanOrigem, scanDestino, idx);//wrapper para a fobj, requer apenas a transformacao
    std::function<double(Eigen::VectorXd)> restricao =
            std::bind(fRestricao, _1,outrasPoses );
    double erro = pso_gbest(x, objetivo, opcoes, limiteInferior, limiteSuperior, restricao);
    return pose(x);
}
