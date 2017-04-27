#ifndef MODELO_MOVIMENTO
#define MODELO_MOVIMENTO
#include "modelo_movimento_base.h"

/*O tipo tipoVetor deve possuir operadores que permitam o acesso as suas componentes individualmente, de modo similar ao que se da com 
vetores padrao em C++. Alem disso, assume-se que as operacoes de soma, diferenca, comparacao e atribuicao estejam definidas pelos operadores usuais
(sobrecarga). Os elementos constituintes do vetor poseRobo, e seus congeneres,
devem ser capazes de aceitar argumentos numericos como inicializadores. Elementos tipoMatriz devem aceitar indexacao do tipo A(lin,col) (sobrecarga de operator())*/
template< class tipoVetor,class tipoMatriz,unsigned int dim>
class modeloMovimento: public modeloMovimentoBase<tipoVetor,tipoMatriz>{
	const tipoMatriz padraoCov;
public:
	modeloMovimento();
	modeloMovimento(tipoVetor poseInicial);
	tipoVetor posicao();
	tipoVetor atualizaPosicao(tipoVetor velocidade);
};
#endif

