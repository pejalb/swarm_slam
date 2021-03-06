#include "gridMap.h"
#include <cmath>
#include <fstream>
#include <boost/foreach.hpp>


bool gridMap::maiorLinhaPertencente(int &x1, int &y1, int &x2, int &y2)
{
    //encontra o maior segmento pertencente ao espaco do mapa (se houver retorna true)
    static int numRec = 1;
    if ((x1 < 0 && x2 < 0) || (x1 >= maxLinhas && x2 >= maxLinhas) || (y1 < 0 && y2 < 0) || (y1 >= maxColunas && y2 >= maxColunas))
        return false;//nao ha segmentos pertencentes    
    else if (pertence(x1, y1) && pertence(x2, y2)) {
        numRec = 1;
        return true;
    }
    else if (!pertence(x1, y1) && !pertence(x2, y2)) {
        numRec = 0;
        return false;
    }
    else {
        if (x1 < 0 && x2 >= 0 ) {
            x1 = 0;
        }
        if (x1 >= 0 && x2 < 0) {
            x2 = 0;
        }
        if (x1 >= maxLinhas && x2 < maxLinhas) {
            x1 = maxLinhas-1;
        }
        if (x2 >= maxLinhas && x1 < maxLinhas) {
            x2 = maxLinhas-1;
        }
        if (y1 < 0 && y2 >= 0) {
            y1 = 0;
        }
        if (y1 >= 0 && y2 < 0) {
            y2 = 0;
        }
        if (y1 >= maxColunas && y2 < maxColunas) {
            y1 = maxColunas-1;           
        }
        if (x2 >= maxColunas && x1 < maxColunas) {
            y2 = maxColunas-1;            
        }
        if (numRec-- > 0)
            maiorLinhaPertencente(x1, y1, x2, y2);
    }
    return false;//nao foi possivel...
}

gridMap::gridMap() :maxLinhas(0),maxColunas(0)
{

}
/*
void bresenham(int x1, int y1, int x2, int y2)
{
    int dx, dy, p, p2, xy2, x, y, xf;
    dx = x2 - x1;
    dy = y2 - y1;
    p = 2 * dy - dx;
    p2 = 2 * dy;
    xy2 = 2 * (dy - dx);
    if (x1>x2)
    {
        x = x2; y = y2; xf = x1;
    }
    else
    {
        x = x1; y = y1; xf = x2;
    }
    putpixel(x, y, 9);
    while (x<xf) {
        x++;
        if (p<0)
            p += p2;
        else {
            y++;
            p += xy2;
        }
        putpixel(x, y, 9);
    }
}
*/

gridMap::gridMap(int linhas, int colunas, double probPrior, double probOcc ) : maxLinhas (linhas),maxColunas(colunas)
{
    //se nada foi definido...inicialize todos os valores com zero
    mapa.resize(maxLinhas, maxColunas);
	mapa.setConstant(M);
    //incrementoFundamental = std::log(probOcc/(1-probOcc));
	origem.x = linhas >> 1;
	origem.y = colunas >> 1;
    //decrementoFundamental = -incrementoFundamental;
}

gridMap::gridMap(int linhas, int colunas, std::vector<ponto>&scan, double probPrior , double probOcc ) : maxLinhas(linhas), maxColunas(colunas)
{
    //se nada foi definido...inicialize todos os valores com zero
    mapa.resize(maxLinhas, maxColunas);
	mapa.setConstant(M);
    //preenche o mapa criado
	origem.x = linhas >> 1;
	origem.y = colunas >> 1;
    BOOST_FOREACH(ponto pto,scan){
        marcaLinha(origem.x, origem.y, pto.x, pto.y);
    }
}

double & gridMap::operator()(int linha, int coluna)
{
    return mapa(origem.x+linha,origem.y+coluna);
}

double gridMap::operator-(const gridMap &rhs)
{
    return (this->mapa - rhs.mapa).norm();
}

double gridMap::operator-(std::vector<ponto>& scans)
{
    return ((*this) - gridMap(maxLinhas, maxColunas, scans));
}

void gridMap::limpa(void)
{
	mapa.setConstant(M);
}

inline double gridMap::incrementa(int linha, int coluna)
{
	linha += origem.x;
	coluna += origem.y;
    if (pertence(linha, coluna)) {
		if (mapa(linha, coluna) == VVL) {
			mapa(linha, coluna) = VL;
		}
		else if (mapa(linha, coluna) == VL) {
			mapa(linha, coluna) = L;
		}
		else if (mapa(linha, coluna) == L) {
			mapa(linha, coluna) = M;
		}
		else if (mapa(linha, coluna) == M) {
			mapa(linha, coluna) = H;
		}
		else if (mapa(linha, coluna) == H) {
			mapa(linha, coluna) = VH;
		}
		else if(mapa(linha, coluna) == VH) {
			mapa(linha, coluna) = VVH;
		}
		else
			mapa(linha, coluna) = VVH;
        return mapa(linha, coluna);
    }
}

inline double gridMap::decrementa(int linha, int coluna)
{
	linha += origem.x;
	coluna += origem.y;
	if (pertence(linha, coluna)) {
		if (mapa(linha, coluna) == VVH) {
			mapa(linha, coluna) =VH;
		}
		else if (mapa(linha, coluna) == VH) {
			mapa(linha, coluna) = H;
		}
		else if (mapa(linha, coluna) == H) {
			mapa(linha, coluna) = M;
		}
		else if (mapa(linha, coluna) == M) {
			mapa(linha, coluna) = L;
		}
		else if (mapa(linha, coluna) == L) {
			mapa(linha, coluna) = VL;
		}
		else if (mapa(linha, coluna) == VL) {
			mapa(linha, coluna) = VVL;
		}
		else {//(mapa(linha, coluna) == VVL) 
			mapa(linha, coluna) = VVL;
		}
		return mapa(linha, coluna);
	}
}

//marca celulas de um segmento de reta no mapa

inline bool gridMap::pertence(int x, int y)
{
    //determina se o ponto (x,y) pertence ao espaco do mapa
    if (0 <= x && x < maxLinhas && 0 <= y && y < maxColunas)
        return true;
    else
        return false;
}

void gridMap::marcaLinha(int x1, int y1, int x2, int y2)
{
    //if (maiorLinhaPertencente(x1, y1, x2, y2)) {
        int dx, dy, p, p2, xy2, x, y, xf, yf;
        dx = x2 - x1;
        dy = y2 - y1;
        p = 2 * dy - dx;
        p2 = 2 * dy;
        xy2 = 2 * (dy - dx);
        if (x1 > x2) {
            x = x2; y = y2; xf = x1; yf = y1;
        }
        else {
            x = x1; y = y1; xf = x2; yf = y2;
        }
        decrementa(x, y); decrementa(x, y);//ponto inicial...vazio por definicao ...ja que e onde se encontra o robo...
        while (x < xf && x<maxLinhas && y<maxColunas) {
            x++;
            if (p < 0)
                p += p2;
            else {
                y++;
                p += xy2;
            }if (y!=yf)
                decrementa(x, y);//espaco livre
        }
        incrementa(x, y); incrementa(x, y);
    //}
}



void gridMap::salva(char * nomeArq)
{
    std::ofstream arquivo;
    arquivo.open(nomeArq,std::ios::out | std::ios::trunc);//apaga quaisquer outros arquivos com o mesmo nome!
    if (arquivo.is_open()) {
        arquivo << mapa;
        arquivo.close();
    }
}

double gridMap::leMapa(int linha, int coluna)
{
	linha += origem.x;
	coluna += origem.y;
    if (pertence(linha, coluna))
        return mapa(linha, coluna);
    else //adicione tratamento de excecao!!!!!!!!!!
        return 0.0;//nao contribui para o erro na fobj
}

double gridMap::alteraMapa(int linha, int coluna, double valor)
{
    return mapa(linha+origem.x,coluna+origem.y)=valor;
}

gridMap::~gridMap()
{
}
