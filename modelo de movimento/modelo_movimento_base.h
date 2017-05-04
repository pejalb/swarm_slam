#ifndef MODELO_MOVIMENTO_BASE
#define MODELO_MOVIMENTO_BASE
/*NOTA PESSOAL : Define um modelo para a estimacao do movimento do robo baseado na ultima velocidade e posicao conhecidas.
Isso torna possivel que se reduza o espaco de buscas nos casos em que o processo de scan matching for bem sucedido e torna
possivel sustentar o processo de localizacao caso o processo de scan matching falhe. Tem sido observado que em casos de haver pequena
sobreposicao entre os scans um processo de correspondencia fornece resultados pouco satisfatorios, ademais a multimodalidade da funcao objetivo
tem apresentado um desafio por levar a estimativas inconsistentes.*/
template <class tipoVetor,class tipoVetor2,class tipoMatriz>
class modeloMovimentoBase {
	protected:
		tipoVetor poseRobo;//pose
		tipoMatriz covariancias;//matriz de covariancias da pose
	public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		virtual tipoVetor posicao() = 0;
		virtual tipoVetor atualizaPosicao(tipoVetor2 velocidade) = 0;
};

#endif