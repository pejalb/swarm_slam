#include "detector_cantos.h"
#define ABS(x) x>=0?(x):(-x)
#define IGUAL(x,y,tol) ABS(x-y)<=(tol)?true:false
#define DIFERENTE(x,y,tol) !IGUAL(x,y,tol)

inline bool horizontal(ponto pInicial, ponto pFinal, double tolerancia = 1e-3)
{
    /*Determina se o segmento com extremindades em p1 e p2 e horizontal,
    com a tolerancia especificada para os criterios utilizados.*/
    ponto diff = pInicial - pFinal;
    if (IGUAL(diff.y, 0.0, tolerancia) && DIFERENTE(diff.x, 0.0, tolerancia))
        return true;
    else
        return false;
}

inline bool vertical(ponto pInicial, ponto pFinal, double tolerancia = 1e-3)
{
    /*Determina se o segmento com extremindades em p1 e p2 e horizontal,
    com a tolerancia especificada para os criterios utilizados.*/
    ponto diff = pInicial - pFinal;
    if (IGUAL(diff.x, 0.0, tolerancia) && DIFERENTE(diff.y, 0.0, tolerancia))
        return true;
    else
        return false;
}

bool detectaCanto(const std::vector<ponto> &scan, int inicio, int fim, int &idx, double tolerancia)
{
    //calcula o ponto medio
    int meio = (inicio + fim) / 2;
    if (inicio == fim || vertical(scan[inicio], scan[fim], tolerancia) || horizontal(scan[inicio], scan[fim], tolerancia)) {
        return false;
    }
    else {
        if (horizontal(scan[inicio], scan[meio], tolerancia) && vertical(scan[meio], scan[fim], tolerancia)) {
            idx = meio;
            return true;
        }
        else if (vertical(scan[inicio], scan[meio], tolerancia) && horizontal(scan[meio], scan[inicio], tolerancia)) {
            idx = meio;
            return true;
        }
        else {
            if (detectaCanto(scan, inicio, meio, idx, tolerancia))
                return true;
            else if (detectaCanto(scan, meio, fim, idx, tolerancia))
                return true;
            else
                return false;
        }
    }
}