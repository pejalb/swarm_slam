#include "modelo_movimento.h"
#include <cmath>
#define ABS(x) ((x)>0)?(x):(-x)
#define TOL 1e-3
#define E_ZERO(x) ABS(x)<TOL

modeloMovimento::modeloMovimento()
{
	poseRobo.setZero();
	padraoCov.setIdentity();
	covariancias.setZero();
}

inline modeloMovimento::modeloMovimento(Eigen::Vector3d poseInicial)
{
	poseRobo = poseInicial;
	padraoCov.setIdentity();
	covariancias.setZero();
}

Eigen::Vector3d modeloMovimento::posicao()
{
	return poseRobo;
}

void modeloMovimento::mudaPasso(double novoDt)
{
	dt = novoDt;
}

void modeloMovimento::definePose(Eigen::Vector3d novaPose)
{
	poseRobo = novaPose;
	covariancias.setIdentity();
}

Eigen::Vector3d modeloMovimento::atualizaPosicao(Eigen::Vector2d velocidade)
{
	//pose [0] = x,pose[1]=y,pose[3] = angulo
	double razao = -velocidade[0] / velocidade[1];// v / w
	if (!E_ZERO(velocidade[1])) {
		poseRobo[0] = poseRobo[0] + razao*(-std::sin(poseRobo[2]) + std::sin(poseRobo[2] + velocidade[1] * dt));
		poseRobo[1] = poseRobo[1] + razao*(std::cos(poseRobo[2]) - std::cos(poseRobo[2] + velocidade[1] * dt));
		poseRobo[2] = poseRobo[2] + velocidade[1] * dt;
	}
	else {
		poseRobo[0] = poseRobo[0] + velocidade[0] *(-std::sin(poseRobo[2]));
		poseRobo[1] = poseRobo[1] + velocidade[0] *(std::cos(poseRobo[2]));
		poseRobo[2] = poseRobo[2];
	}

	//se nao for passada matriz de variancias da velocidade, assume uma covariancia padrao.
	covariancias += padraoCov;
	return poseRobo;
}

const Eigen::Vector3d modeloMovimento::retornaIncerteza(void)
{
	//Eigen::JacobiSVD<Eigen::Matrix3d> svd(covariancias);
	//const Eigen::Vector3d autoval=svd.singularValues();
	return Eigen::Vector3d((std::sqrt(covariancias(0,0))), 
		std::abs(std::sqrt(covariancias(1, 1))),std::abs(std::sqrt(covariancias(2, 2))));
}
