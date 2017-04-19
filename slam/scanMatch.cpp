#include "scanMatch.h"
#include "constantes.h"
#include <Eigen/Dense>
#include <cstdarg>
#include <exception>
#include <functional>
#include "pointcloud_kd_tree.h"
#include <iostream>//debugging
using namespace nanoflann;

inline void transforma_vetor_pontos(std::vector<ponto>& scan, pose &p)
{
    for (int i = 0; i < scan.size(); i++) {
        scan[i] = p + scan[i];
    }
}

inline double rand_uniforme(double min, double max)
{
    return (std::rand()*(max - min) / RAND_MAX) + min;
}

using namespace Eigen;

typedef std::pair<Eigen::Matrix2d, Eigen::Vector2d> TransformType;
typedef std::vector<Eigen::Vector2d>   PointsType;

TransformType icpSVD(const PointsType& src, const PointsType& dst) 
{
	assert(src.size() == dst.size());
	int pairSize = src.size();
	Eigen::Vector2d center_src(0, 0), center_dst(0, 0);
	for (int i = 0; i<pairSize; ++i)
	{
		center_src += src[i];
		center_dst += dst[i];
	}
	center_src /= (double)pairSize;
	center_dst /= (double)pairSize;

	Eigen::MatrixXd S(pairSize, 2), D(pairSize, 2);
	for (int i = 0; i<pairSize; ++i)
	{
		for (int j = 0; j<2; ++j)
			S(i, j) = src[i][j] - center_src[j];
		for (int j = 0; j<2; ++j)
			D(i, j) = dst[i][j] - center_dst[j];
	}
	Eigen::MatrixXd Dt = D.transpose();
	Eigen::Matrix2d H = Dt*S;
	Eigen::Matrix2d W, U, V;
	Eigen::BDCSVD<Eigen::MatrixXd> svd;
	Eigen::MatrixXd H_(2, 2);
	for (int i = 0; i<2; ++i) for (int j = 0; j<2; ++j) H_(i, j) = H(i, j);
	svd.compute(H_, Eigen::ComputeThinU | Eigen::ComputeThinV);
	if (!svd.computeU() || !svd.computeV()) {
		std::cerr << "erro de decomposicao" << std::endl;
		return std::make_pair(Eigen::Matrix2d::Identity(), Eigen::Vector2d::Zero());
	}
	Eigen::Matrix2d Vt = svd.matrixV().transpose();
	Eigen::Matrix2d R = svd.matrixU()*Vt;
	Eigen::Vector2d t = center_dst - R*center_src;

	return std::make_pair(R, t);
}

PointsType converte_ponto_vector(std::vector <ponto> &pontos)
{
	const int tamanho = pontos.size();
	PointsType p; p.reserve(tamanho);
	for (int i = 0; i < tamanho; i++) {
		p.push_back(pontos[i].paraVetor());
	}
	return p;
}

void transformTypeToDouble(TransformType &RT, double*x, int tamanho = 3)
{
	x[0] = (RT.second)[0];
	x[1] = (RT.second)[1];
	x[2] = std::acos(RT.first(0, 0));
}

Eigen::Transform<double, 2, Eigen::Affine> cria_transformacao(double angulo, double transX, double transY)
{
    /*cria uma transformacao com rotacao seguida de translacao*/
    Eigen::Rotation2D<double> Rot(angulo);
    Eigen::Translation<double, 2> translacao(Eigen::Vector2d(transX, transY));
    Eigen::Transform<double, 2, Eigen::Affine> T = Rot*translacao;
    return T;
}

double fobj(Eigen::VectorXd v,std::vector<ponto> & scanOrigem, std::vector<ponto> & scanDestino, std::vector<std::vector<size_t> > &idx)
{
	//std::cout << "\nv =" << v(0) << "," << v(1) << "," << v(2)<<std::endl;//debugging -nan(ind)
    pose p(v);
    double erro=0.0;    
    //PointCloud<double> cloud; cloud.pts = scanDestino;
    //double query[2] = { scanOrigem[0].x,scanOrigem[0].y };
    //std::vector<std::vector<size_t> > idx=constroiKDtree<double>(cloud, scanOrigem,1);
    int numPontos = idx.size();
    for (size_t i = 0; i < numPontos; i+=1){
        for (size_t j = 0; j < idx[i].size(); j++){
            erro += (scanDestino[idx[i][j]] - (p + scanOrigem[i])).quadradoNorma();
        }        
    }
    //std::cout << "\nerro = " << erro << std::endl;
    return std::sqrt(erro/((double)numPontos));
}

double fobjMelhorada(Eigen::VectorXd v, std::vector<ponto> & scanOrigem, std::vector<ponto> & scanDestino, std::vector<std::vector<size_t> > &idx, gridMap *m)
{
    pose p(v);
    int numPontos = scanOrigem.size();
    int maxLin = m->maxLinhas /10;
    int maxCol = m->maxColunas /10;
    transforma_vetor_pontos(scanOrigem, p);
    double erro =fobj(v,scanOrigem,scanDestino,idx);
    double diffMapa = 0.0;
    for (size_t i = 0; i < scanDestino.size(); i++){
        diffMapa -= m->leMapa(std::round(scanOrigem[i].x), std::round(scanOrigem[i].y));
    }
    //std::cout << "\nerro = " << erro << std::endl;
    return std::sqrt(erro / ((double)numPontos))*diffMapa;
}

inline double fRestricaoPadrao(Eigen::VectorXd v)
{
    return 0.0;// v.squaredNorm();//se a restricao nao-linear nao foi fornecida...usa um "placeholder"
}

pose psoScanMatch(std::vector<ponto> & scanOrigem, std::vector<ponto> & scanDestino,double estimativaInicial[3],gridMap *m) throw(std::domain_error)
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
    Eigen::VectorXd x;// = Eigen::VectorXd::Zero(3, 1);
    opcoesPSO opcoes;
    opcoes.coefCognitivo = 0.7;
    opcoes.coefInercia = 0.9;
    opcoes.coefSocial = 0.7;
    opcoes.limitaVelocidade = true;
    opcoes.maxIter = 100;
    opcoes.numDimensoes = 3;
    opcoes.modoDeConfinamento = BLOQUEIO;
    opcoes.normalizaValores = false;
    double estimativaSVD[3];
    opcoes.numParticulas = 50;
    opcoes.velMax = 0.05;
    opcoes.tolerancia = 0.1;
	//se desejado pode criar uma estimativa
	TransformType RT = icpSVD(converte_ponto_vector(scanOrigem), converte_ponto_vector(scanDestino));
	transformTypeToDouble(RT, estimativaSVD);
	//pose poseRelativa = psoScanMatch(scanOrigem, scanDestino, estimativaInicial);
//	pose poseRelativa(0.0, 0.0, 0.0);
	
    if (std::max(std::max(std::abs(estimativaSVD[0]), std::abs(estimativaSVD[1])), 
        std::abs(estimativaSVD[2])) <= opcoes.tolerancia && (std::isnormal(estimativaSVD[2]) || estimativaSVD[2]==0))//nao calcule se ja estiver na tolerancia
        return pose(estimativaSVD[0], estimativaSVD[1], estimativaSVD[2]);
    //se a estimativa estiver muito distante...
    //use pso para aproximar mais
    double limiteInferior[3] = { estimativaInicial[1] + estimativaSVD[0] - DX,estimativaInicial[1] + estimativaSVD[1] - DY,estimativaInicial[2] - D_ANG };
    double limiteSuperior[3] = { estimativaInicial[1] + estimativaSVD[0] + DY,estimativaInicial[1] + estimativaSVD[1] + DY,estimativaInicial[2] + D_ANG };
    //ponto center_src(0, 0);
    //ponto center_dst(0, 0);
   // int tamanho = scanOrigem.size();
    //normaliza a nuvem de pontos
    //dados os centros a determinacao da translacao entre as nuvens e trivial.
    /*for (int i = 0; i<tamanho; ++i)
    {
        center_src += scanOrigem[i];
        center_dst += scanDestino[i];
    }
    center_src /= (double)tamanho;
    center_dst /= (double)tamanho;*/

    //std::function<double(Eigen::VectorXd)> objetivo= 
    //    std::bind(fobj,_1,scanOrigem, scanDestino);//wrapper para a fobj, requer apenas a transformacao
	PointCloud<double> cloud; cloud.pts = scanDestino;
	std::vector<std::vector<size_t> > idx = constroiKDtree<double>(cloud, scanOrigem, 1);
    std::function<double(Eigen::VectorXd)> objetivo =
            std::bind(fobjMelhorada,_1,scanOrigem, scanDestino,idx,m);//wrapper para a fobj, requer apenas a transformacao
    std::function<double(Eigen::VectorXd)> restricao = fRestricaoPadrao;
    double erro = pso_gbest(x, objetivo, opcoes, limiteInferior, limiteSuperior, restricao);
    std::cout << "\t erro = " << erro;
    if (erro < 0)
        throw (std::domain_error("\nO processo de otimizacao obteve um valor invalido!"));
    return pose(x);
}

pose psoScanMatch(std::vector<ponto>& scanOrigem, std::vector<ponto>& scanDestino, double estimativaInicial[3], gridMap *m,
    std::vector<pose>& outrasPoses, double(*fRestricao)(Eigen::VectorXd, std::vector<pose>&)) throw(std::domain_error)
{
    using namespace std::placeholders;
    Eigen::VectorXd x;// = Eigen::VectorXd::Zero(3, 1);
    opcoesPSO opcoes;
    opcoes.coefCognitivo = 0.9;
    opcoes.coefInercia = 0.9;
    opcoes.coefSocial = 0.7;
    opcoes.limitaVelocidade = true;
    opcoes.maxIter = 100;
    opcoes.numDimensoes = 3;
    opcoes.modoDeConfinamento = BLOQUEIO;
    opcoes.normalizaValores = false;
    opcoes.numParticulas = 50;
    opcoes.velMax = 0.5;
    opcoes.tolerancia = 1;
	//cria estimativa inicial...
	//TransformType RT = icpSVD(converte_ponto_vector(scanOrigem), converte_ponto_vector(scanDestino));
	//transformTypeToDouble(RT, estimativaInicial);
    double limiteInferior[3] = { estimativaInicial[0] - DX,estimativaInicial[1] - DY,estimativaInicial[2] - D_ANG };
    double limiteSuperior[3] = { estimativaInicial[0] + DY,estimativaInicial[1] + DY,estimativaInicial[2] + D_ANG };
    //ponto center_src(0, 0);
    //ponto center_dst(0, 0);
    int tamanho = scanOrigem.size();
    //normaliza a nuvem de pontos
    //dados os centros a determinacao da translacao entre as nuvens e trivial.
    /*for (int i = 0; i<tamanho; ++i)
    {
    center_src += scanOrigem[i];
    center_dst += scanDestino[i];
    }
    center_src /= (double)tamanho;
    center_dst /= (double)tamanho;*/

    //std::function<double(Eigen::VectorXd)> objetivo =
    //    std::bind(fobj, _1, scanOrigem, scanDestino);//wrapper para a fobj, requer apenas a transformacao
	PointCloud<double> cloud; cloud.pts = scanDestino;
	std::vector<std::vector<size_t> > idx = constroiKDtree<double>(cloud, scanOrigem, 1);
    std::function<double(Eigen::VectorXd)> objetivo =
        std::bind(fobjMelhorada, _1, scanOrigem, scanDestino,idx,m);//wrapper para a fobj, requer apenas a transformacao
    std::function<double(Eigen::VectorXd)> restricao =
            std::bind(fRestricao, _1,outrasPoses );
    double erro = pso_gbest(x, objetivo, opcoes, limiteInferior, limiteSuperior, restricao);
    std::cout << "\t erro = " << erro;
    if (erro < 0.0)
        throw(std::domain_error("\nO processo de otimizacao obteve um valor invalido!"));
    return pose(x);
}
