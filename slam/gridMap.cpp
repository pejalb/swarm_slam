#include <cstdlib>
#include <cstdio>
#include <cmath>
#include <fstream>
#include "gridMap.h"

#define TAM_MAX_NOME_ARQ 80
#define TAM_MAX_NUM 10

//necessario no vc++
#define _CRT_SECURE_NO_WARNINGS

gridMap::gridMap():probOcupacao(0.9), probPriori(0.5), probLivre(0.1)
{
	//ctr padrao
	numLinhas = numColunas = 0;
	incremento = 0.1*std::log(probOcupacao / probLivre);
	//incremento = std::log(probLivre /  probOcupacao);
	numFrames = 0;
}

gridMap::gridMap(int linhas, int colunas, bool usaLogOdds,
	double probPrior , double probOcc , double probFree ) :probOcupacao(probOcc), probPriori(probPrior), probLivre(probFree)
{
	if (linhas >= 0 && colunas >= 0){
		//grid = new double[linhas*colunas];
        grid.reserve(linhas);
        logOddsPrior = std::log(probPrior / (1 - probPrior));
        std::vector<long double> tmp; tmp.reserve(colunas);
        for (int j = 0; j < colunas; j++) {
            tmp.push_back(logOddsPrior);
        }
        for (int i = 0; i < linhas; i++){            
            grid.push_back(tmp);
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
	/*acrescente tratamento em caso de n ser invalido
	ou de ser impossivel alocar espaco suficiente*/
}


gridMap::~gridMap()
{
}

long double gridMap::leMapa(int linha, int coluna)
{
	//retorna o valor presente na posicao linha,coluna
    if (linha < numLinhas && coluna < numColunas
        && linha >= 0 && coluna >= 0) {
        return grid[linha][coluna];
    }
    else {
        return 0.0;//MELHORE O TRATAMENTO DE POSSIVEIS ERROS DE ALOCACAO!!!
    }
}

long double gridMap::alteraMapa(int linha, int coluna, long double novoValor)
{
	if (linha < numLinhas && coluna < numColunas
		&& linha >= 0 && coluna >= 0){
		//if (grid[coluna*numLinhas+linha])
                //    printf("\ngrid[...] aponta para null");
                //if(grid==NULL)
                //    printf("\ngrid e null");
		//printf("\ngrid[%d][%d]=%lf",linha,coluna,grid[coluna*numLinhas+linha]);
                grid[linha][coluna] = novoValor;
		//printf("\ngrid[%d][%d]=%lf",linha,coluna,grid[coluna*numLinhas+linha]);
		return grid[linha][coluna];//retorna o valor posto na posicao
	}
	else {
		return 0.0;//MELHORE O TRATAMENTO DE POSSIVEIS ERROS DE ALOCACAO!!!
	}
}

long double gridMap::incrementa(int linha, int coluna)
{
    if (linha < numLinhas && coluna < numColunas
        && linha >= 0 && coluna >= 0) {
        grid[linha][coluna] = grid[linha][coluna] + 2*incremento - logOddsPrior;
    }
    else {
        return 0.0;
    }
}

long double gridMap::decrementa(int linha, int coluna)
{
    if (linha < numLinhas && coluna < numColunas
        && linha >= 0 && coluna >= 0) {
        grid[linha][coluna] = grid[linha][coluna] + 0.5*decremento - logOddsPrior;
    }
    else {
        return 0.0;
    }
}

int gridMap::tamanhoHorizontal(void)
{
    return numLinhas;
}

int gridMap::tamanhoVertical(void)
{
    return numColunas;
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
        long double max = 0.0;
		if (myfile.is_open()) {
			for (int i = 0; i < this->tamanhoVertical(); i++) {
				for (int j = 0; j < this->tamanhoHorizontal(); j++) {
                    long double val = leMapa(i, j);
                    if (val > max)
                        max = val;
					myfile << val << ",";
				}
				myfile << std::endl;
			}
            //max += 1.0;
			myfile.close();
            for (int i = 0; i < this->tamanhoVertical(); i++) {
                for (int j = 0; j < this->tamanhoHorizontal(); j++) {
                    grid[i][j] = grid[i][j] / max;
                }
            }
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

//metodo de breesenham para desenhar linhas

void gridMap::marcaLinha(int xInicial,int yInicial,int xFinal, int yFinal,bool decrementa)
{
	/*Utiliza o algoritmo de Bresenham para marcar a regiao entre o robo e o primeiro obstaculo. 
	O argumento decrementa==true faz a probabilidade dos itens considerados obstáculos no mapa o serem de fato crescer.
	Embora o modo padrao decremente o espaço vazio e incremente a probabilidade das regioes consideradas intransponíveis,
	é possível inverter esse comportamento fazendo decrementa==false.
	*/	
	int dx, dy, incr1, incr2, d, x, y, xend, yend, xdirflag, ydirflag;
    int cnt = 0;

    dx = abs(xFinal - xInicial); dy = abs(yFinal - yInicial);

    if (dy <= dx) {
        d = 2 * dy - dx; incr1 = 2 * dy; incr2 = 2 * (dy - dx);
        if (xInicial > xFinal) {
            x = xFinal; y = yFinal;
            ydirflag = (-1);
            xend = xInicial;
        }
        else {
            x = xInicial; y = yInicial;
            ydirflag = 1;
            xend = xFinal;
        }
        this->decrementa(x, y);
        if (((yFinal - yInicial) * ydirflag) > 0) {
            while (x < xend) {
                x++;
                if (d <0) {
                    d += incr1;
                }
                else {
                    y++; d += incr2;
                }
                this->decrementa(x, y);
            }
        }
        else {
            while (x < xend) {
                x++;
                if (d <0) {
                    d += incr1;
                }
                else {
                    y--; d += incr2;
                }
                this->decrementa(x, y);
            }
        }
    }
    else {
        d = 2 * dx - dy;
        incr1 = 2 * dx; incr2 = 2 * (dx - dy);
        if (yInicial > yFinal) {
            y = yFinal; x = xFinal;
            yend = yInicial;
            xdirflag = (-1);
        }
        else {
            y = yInicial; x = xInicial;
            yend = yFinal;
            xdirflag = 1;
        }
        this->decrementa(x, y);
        if (((xFinal - xInicial) * xdirflag) > 0) {
            while (y < yend) {
                y++;
                if (d <0) {
                    d += incr1;
                }
                else {
                    x++; d += incr2;
                }
                this->decrementa(x, y);
            }
        }
        else {
            while (y < yend) {
                y++;
                if (d <0) {
                    d += incr1;
                }
                else {
                    x--; d += incr2;
                }
                this->decrementa(x, y);
            }
        }
    }
    this->incrementa(xFinal, yFinal);
    this->incrementa(xFinal, yFinal);
    this->incrementa(xFinal, yFinal);
}
