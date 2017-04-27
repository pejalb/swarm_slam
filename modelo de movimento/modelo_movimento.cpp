#include "modelo_movimento.h"
#include <cmath>


modeloMovimento::modeloMovimento()
{
	poseRobo.setZero();
	padraoCov.setIdentity();
}

inline modeloMovimento::modeloMovimento(Eigen::Vector3d poseInicial)
{
	poseRobo = poseInicial;
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
	poseRobo[0] = poseRobo[0] + razao*(-std::sin(poseRobo[3])+std::sin(poseRobo[3]+velocidade[1]*dt));
	poseRobo[1] = poseRobo[1] + razao*(std::cos(poseRobo[3]) - std::cos(poseRobo[3] + velocidade[1] * dt));
	poseRobo[2] = poseRobo[2] + velocidade[1] * dt;

	//se nao for passada matriz de variancias da velocidade, assume uma covariancia padrao.
	covariancias += padraoCov;
	return poseRobo;
}
