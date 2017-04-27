#include "modelo_movimento.h"
template< class tipoVetor, class tipoMatriz, unsigned int dim>
modeloMovimento<tipoVetor,tipoMatriz,dim>::modeloMovimento()
{
	//inicializa a posicao
	for (size_t i = 0; i < dim; i++){
		poseRobo[i] = 0.0;//cada elemento da posica
		for (size_t j = 0; j < dim; j++){
			covariancias(i,j) = 0.0;
		}
	}
	//inicializa a matriz padrao de covariancias
	
}
template< class tipoVetor, class tipoMatriz, unsigned int dim>
inline modeloMovimento<tipoVetor,tipoMatriz,dim>::modeloMovimento(tipoVetor poseInicial)
{
	poseRobo = poseInicial;
}

template< class tipoVetor, class tipoMatriz, unsigned int dim>
tipoVetor modeloMovimento<tipoVetor,tipoMatriz,dim>::posicao()
{
	return poseRobo;
}

template< class tipoVetor, class tipoMatriz, unsigned int dim>
tipoVetor modeloMovimento<tipoVetor, tipoMatriz, dim>::atualizaPosicao(tipoVetor velocidade)
{
	poseRobo += velocidade;
	//se nao for passada matriz de variancias da velocidade, assume uma covariancia padrao.

	return poseRobo;
}
