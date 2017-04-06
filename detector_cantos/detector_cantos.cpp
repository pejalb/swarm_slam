#include "detector_cantos.h"
#define ABS(x) x>=0?x:-x
#define IGUAL(x,y,tol) ABS(x-y)<=tol?true:false
#define DIFERENTE(x,y,tol) ABS(x-y)>tol?true:false

bool detectaCanto(std::vector<ponto> scan,int inicio, int fim,ponto *canto,double tolerancia)
{
	//calcula a diferenca
	ponto diff = scan[inicio] - scan[fim];
	if (IGUAL(diff.x, 0.0, tolerancia) && DIFERENTE(diff.y, 0.0, tolerancia))//e vertical
		return false;
	else if (DIFERENTE(diff.x, 0.0, tolerancia) && IGUAL(diff.y, 0.0, tolerancia))//horizontal
		return false;
	else if (IGUAL(diff.x, 0.0, tolerancia) && IGUAL(diff.y, 0.0, tolerancia))//sao o mesmo ponto
		return false;
	else {//ha uma inclinacao, donde o intervalo pode conter um canto
		int mid = (inicio + fim) / 2;
		bool subInt1 = detectaCanto(scan, inicio, mid, canto,tolerancia);
		bool subInt2 = detectaCanto(scan, mid, fim, canto, tolerancia);
		if (!(subInt1 && subInt2)) {//ha uma mudanca abrupta em mid
			canto = &scan[mid];
			return true;
		}
	}
}