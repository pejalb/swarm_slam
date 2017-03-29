#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <cmath>
#include <fstream>
#include "gridMap.h"
#include "constantes.h"

//necessario no vc++
#define _CRT_SECURE_NO_WARNINGS
//diferenca absoluta
#define ABS(a) (a)>=0?a:(-a)
#define DIFF_ABS(x,y) (x>0&&y>0)?(x>y?(x-y):(y-x)):0
//verifica faixa
inline bool verificaIndice(int idx, int min, int max) 
{
	//retorna true se idx pertence a [min,max[
	if (idx >= min && idx < max)
		return true;
	else
		return false;
}



gridMap::gridMap():probOcupacao(0.9), probPriori(0.5), probLivre(0.1)
{
	//ctr padrao
	grid = NULL;
	numLinhas = numColunas = 0;
	incremento = std::log(probOcupacao / probLivre);
	incremento = std::log(probLivre /  probOcupacao);
	numFrames = 0;
}

gridMap::gridMap(int n, bool usaLogOdds, double probPrior, 
	double probOcc, double probFree):probOcupacao(probOcc), probPriori(probPrior), probLivre(probFree)
{
	if (n >= 0){
        try {
            grid = new double[n*n];
        }
        catch (std::bad_alloc){
            std::cerr << "\n Nao foi possivel alocar memoria para o mapa!" << std::endl;
            std::exit(1);
        }
		
		numLinhas = n;
		numColunas = n;	
		logOdds = usaLogOdds;
		if (usaLogOdds) {
			incremento = std::log(probOcupacao / probLivre);
			decremento= std::log(probLivre / probOcupacao);
		}
		numFrames = 0;
	}
	double logPrior = std::log(probPriori / (1 - probPriori));
	for (int i = 0; i < numLinhas; i++) {
		for (int j = 0; j < numColunas; j++) {
			alteraMapa(i, j, logPrior);
		}

	}
	/*acrescente tratamento em caso de n ser invalido
	ou de ser impossivel alocar espaco suficiente*/
}

gridMap::gridMap(int linhas,int colunas, bool usaLogOdds,
	double probPrior , double probOcc , double probFree ) :probOcupacao(probOcc), probPriori(probPrior), probLivre(probFree)
{
	if (linhas >= 0 && colunas >= 0){
        try {
            grid = new double[linhas*colunas];
        }
        catch (std::bad_alloc) {
            std::cerr << "\n Nao foi possivel alocar memoria para o mapa!" << std::endl;
            std::exit(1);
        }
        
		numLinhas = linhas;
		numColunas = colunas;
		logOdds = usaLogOdds;
		if (usaLogOdds) {
			incremento = std::log(probOcupacao / probLivre);
			decremento = std::log(probLivre / probOcupacao);
		}
		numFrames = 0;
	}
	double logPrior = std::log(probPriori / (1 - probPriori));
	for (int i = 0; i < numLinhas; i++) {
		for (int j = 0; j < numColunas; j++) {
			alteraMapa(i, j, logPrior);
		}

	}
	/*acrescente tratamento em caso de n ser invalido
	ou de ser impossivel alocar espaco suficiente*/
}


gridMap::~gridMap()
{
	//destrutor padrao
	if (grid != NULL){
		delete[] grid;
	}
	grid = NULL;
}

double gridMap::leMapa(int linha,int coluna)
{
	//retorna o valor presente na posicao linha,coluna
    if (linha < numLinhas && coluna < numColunas) {
        return grid[coluna*numLinhas + linha];
        //debugging
        //if (grid[coluna*numLinhas + linha] != 0.0)
            //std::cout << "\n (" << linha << "," << coluna << ") = " << grid[coluna*numLinhas + linha];
        //system("pause");
    }
}

double gridMap::alteraMapa(int linha,int coluna,double novoValor)
{
	//if (linha < numLinhas && coluna < numColunas ){
		//if (grid[coluna*numLinhas+linha])
                //    printf("\ngrid[...] aponta para null");
                //if(grid==NULL)
                //    printf("\ngrid e null");
		//printf("\ngrid[%d][%d]=%lf",linha,coluna,grid[coluna*numLinhas+linha]);
                grid[coluna*numLinhas + linha] = novoValor;
		//printf("\ngrid[%d][%d]=%lf",linha,coluna,grid[coluna*numLinhas+linha]);
		return grid[coluna*numLinhas+linha];//retorna o valor posto na posicao
	//}
	//else {
	//	return NULL;//MELHORE O TRATAMENTO DE POSSIVEIS ERROS DE ALOCACAO!!!
	//}
}

inline double gridMap::incrementa(int linha,int coluna)
{
    double *tmp = &grid[coluna*numLinhas + linha];
    /*teste somente o valor das linhas aqui!
    use a criacao para testar a existencia do espaco alocado*/
    //if (grid != NULL && linha <= numLinhas && coluna <= numColunas
	//if (grid != NULL && linha <= numLinhas && coluna <= numColunas
	//if (linha < numLinhas && coluna < numColunas) {
        (*tmp) = (*tmp) + incremento;
        return (*tmp);
    //}
}

inline double gridMap::decrementa(int linha,int coluna)
{
    double *tmp = &grid[coluna*numLinhas + linha];
    //if (grid != NULL && linha < numLinhas && coluna < numColunas
	//if ( linha < numLinhas && coluna < numColunas) {
        (*tmp) = (*tmp) - decremento;
        return (*tmp);
    //}
}

inline double gridMap::tamanhoHorizontal(void)
{
    return numLinhas;
}

inline double gridMap::tamanhoVertical(void)
{
    return numColunas;
}

Eigen::MatrixXd gridMap::retornaMatriz(void)
{
	/*Converte o mapa inteiro para uma matriz do Eigen e a retorna*/
	Eigen::MatrixXd matrizMapa = Eigen::MatrixXd::Map(grid, numLinhas, numColunas);
	return matrizMapa;
}

bool gridMap::operator==(gridMap &outroMapa)
{
	if (numLinhas != outroMapa.numLinhas || numColunas != outroMapa.numColunas){
		return false;
	}
	else{
		for (int i = 0; i < numLinhas; i++){
			for (int j = 0; j < numColunas; j++){
				if (leMapa(i, j) != outroMapa.leMapa(i, j))
					return false;
			}
		}
		return true;
	}	
}

bool gridMap::operator!=(gridMap &outroMapa)
{
	if (numLinhas != outroMapa.numLinhas || numColunas != outroMapa.numColunas){
		return true;
	}
	else{
		for (int i = 0; i < numLinhas; i++){
			for (int j = 0; j < numColunas; j++){
				if (leMapa(i, j) != outroMapa.leMapa(i, j))
					return true;
			}
		}
		return false;
	}
}

bool gridMap::gravaMapa(char * nomeArq)
{
	std::ofstream myfile;
	if (nomeArq != NULL){
		myfile.open(nomeArq);
		if (myfile.is_open()) {
			for (int i = 0; i < this->tamanhoVertical(); i++) {
				for (int j = 0; j < this->tamanhoHorizontal(); j++) {
					myfile << this->leMapa(i, j) << ",";
				}
				myfile << std::endl;
			}
			myfile.close();
			return true;
		}
		else
			return false;
	}
	else 
		return false;		
}

bool gridMap::gravaMapaSeq(char * nomeArq)
{
	std::ofstream myfile;
	if (nomeArq != NULL) {
		char nomeArqFrame[TAM_MAX_NOME_ARQ];
		char strNumFrames[TAM_MAX_NUM];
		strcpy(nomeArqFrame, nomeArq);
		std::sprintf(strNumFrames, "%d", numFrames++);
		strcat(nomeArqFrame, strNumFrames);
		myfile.open(nomeArqFrame);
		int j = 0;
		if (myfile.is_open()) {
			for (int i = 0; i < this->tamanhoVertical(); i++) {
				for (j = 0; j < this->tamanhoHorizontal(); j++) {
					myfile << this->leMapa(i, j) << ",";
				}
				myfile << std::endl;
			}
			myfile.close();
			return true;
		}
		else
			return false;
	}
	else
		return false;
}

double & gridMap::operator()(int linha, int coluna)//nao faz quaisquer verificacoes de limites
{
	return grid[coluna*numLinhas + linha];
}

//metodo de breesenham para desenhar linhas

void gridMap::marcaLinha(int xInicial, int yInicial, int xFinal, int yFinal, bool decrementa)
{
	bool teste_xInicial = verificaIndice(xInicial, 0, numColunas);
	bool teste_xFinal = verificaIndice(xFinal, 0, numColunas);
	bool teste_yInicial = verificaIndice(yInicial, 0, numLinhas);
	bool teste_yFinal = verificaIndice(yFinal, 0, numLinhas);

    if ( teste_xInicial && teste_xFinal  && teste_yInicial && teste_yFinal ) {
        /*Utiliza o algoritmo de Bresenham para marcar a regiao entre o robo e o primeiro obstaculo.
        O argumento decrementa==true faz a probabilidade dos itens considerados obstáculos no mapa o serem de fato crescer.
        Embora o modo padrao decremente o espaço vazio e incremente a probabilidade das regioes consideradas intransponíveis,
        é possível inverter esse comportamento fazendo decrementa==false.
        */
        int dx = std::abs(xFinal - xInicial);
        int sx = xInicial < xFinal ? 1 : -1;
        int dy = std::abs(yFinal - yInicial);
        int sy = yInicial < yFinal ? 1 : -1;
        int err = (dx > dy ? dx : -dy) / 2, e2;
        for (;;) {
            if (decrementa)
                this->decrementa(xInicial, yInicial);
            else
                this->incrementa(xInicial, yInicial);
                //std::cout << " xInicial = " << xInicial << " yInicial = " << yInicial << " xFinal = " << xFinal << " yFinal = " << yFinal << std::endl;
            if (xInicial == xFinal && yInicial == yFinal) break;
            e2 = err;
            if (e2 > -dx) { err -= dy; xInicial += sx; }
            if (e2 < dy) { err += dx; yInicial += sy; }
            if (xInicial >= numColunas || yInicial >= numLinhas) break;
        }
        if (decrementa)
            this->incrementa(xFinal, yFinal);
        else
            this->decrementa(xFinal, yFinal);
    }
	else { //tenta corrigir os indices para valores aceitaveis
		if (!teste_xInicial) {
			double derivada = (yFinal - yInicial) / (xFinal - xInicial);//y'(x)
			if (xInicial < 0) {
				xInicial = 0;
				yInicial = std::ceil(yFinal - xFinal*derivada);
			}
				//marcaLinha(0, std::ceil(yFinal - xFinal*derivada), xFinal, yFinal);
			else {
				xInicial = numColunas;//violou o limite superior
				yInicial = std::floor(yFinal - derivada*(xFinal - numColunas));
			}
				//marcaLinha(numColunas, std::floor(yFinal-derivada*(xFinal-numColunas)), xFinal, yFinal);
		}
		if (!teste_xFinal) {
			double derivada = (yFinal - yInicial) / (xFinal - xInicial);//y'(x)
			if (xFinal < 0) {
				xFinal = 0;
				yFinal = std::ceil(yInicial - (xInicial)*derivada);
			}
				//marcaLinha(xInicial, yInicial, 0, std::ceil(yInicial + (-xInicial)*derivada));
			else {
				//violou o limite superior
				xFinal = numLinhas;
				yFinal = std::floor(yInicial + derivada*(numLinhas - xInicial));
				//marcaLinha(xInicial, yInicial, numLinhas, std::floor(yInicial+derivada*(numLinhas-xInicial)) );
			}
		}
		if (!teste_yInicial) {
			double derivada = (yFinal - yInicial) / (xFinal - xInicial);//y'(x)
			if (yInicial < 0) {
				xInicial = std::ceil(xFinal - (yFinal / derivada));
				yInicial = 0;
					//marcaLinha(std::ceil(xFinal-(yFinal/derivada)), 0, xFinal, yFinal);
			}
			else {//violou o limite superior
				xInicial = std::floor(xFinal - (yFinal - numColunas) / derivada);
				yInicial = numColunas;
			}
				//marcaLinha(std::floor(xFinal-(yFinal-numColunas)/derivada), numColunas, xFinal, yFinal);
		}
		if (!teste_yFinal) {
			double derivada = (yFinal - yInicial) / (xFinal - xInicial);//y'(x)
			if (yFinal < 0) {
				xFinal = std::ceil(xInicial - yInicial / derivada);
				yFinal = 0;
			}
				//marcaLinha(xInicial, yInicial, std::ceil(xInicial-yInicial/derivada), 0);
			else {
				xFinal = std::floor(yInicial + (numColunas - yInicial) / derivada);//violou o limite superior
				yFinal = numColunas;
				//marcaLinha(xInicial, yInicial, std::floor(yInicial+(numColunas-yInicial)/derivada), numColunas);
			}
		}
		marcaLinha(xInicial, yInicial, xFinal, yFinal);
	}
}
