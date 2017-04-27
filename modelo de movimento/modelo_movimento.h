#ifndef MODELO_MOVIMENTO
#define MODELO_MOVIMENTO
#include "modelo_movimento_base.h"
#include <Eigen\Dense>

/*O tipo tipoVetor deve possuir operadores que permitam o acesso as suas componentes individualmente, de modo similar ao que se da com 
vetores padrao em C++. Alem disso, assume-se que as operacoes de soma, diferenca, comparacao e atribuicao estejam definidas pelos operadores usuais
(sobrecarga). Os elementos constituintes do vetor poseRobo, e seus congeneres,
devem ser capazes de aceitar argumentos numericos como inicializadores. Elementos tipoMatriz devem aceitar indexacao do tipo A(lin,col) (sobrecarga de operator())*/

class modeloMovimento: public modeloMovimentoBase<Eigen::Vector3d, Eigen::Vector2d, Eigen::Matrix3d>{
	private:
		Eigen::Matrix3d padraoCov;
		double dt;//passo temporal;
	public:
		modeloMovimento();
		modeloMovimento(Eigen::Vector3d poseInicial);
		Eigen::Vector3d posicao();
		void mudaPasso(double novoDt);
		void definePose(Eigen::Vector3d novaPose);
		Eigen::Vector3d atualizaPosicao(Eigen::Vector2d velocidade);//vt,w
		
};
#endif

