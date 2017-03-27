#include "slam.h"
#include <cstdlib>
#include <vector>
#include <iostream>
#include <fstream>
#include <Eigen/Sparse>
#include <string>
#define SCANS_DE_TESTE 1000
#define NUM_MAX_SCANS 700

inline double rand_uniforme(double min, double max)
{
	return (std::rand()*(max - min) / RAND_MAX) + min;
}


//cria pose falsa para teste

std::vector<ponto> cria_scan_de_teste(void)
{
	std::vector<ponto> scan; scan.reserve(LEITURAS_POR_SCAN);
	for (int i=0; i<LEITURAS_POR_SCAN; i++){
		scan.push_back(ponto(rand_uniforme(0, 20000), rand_uniforme(0, 20000)));
	}
	return scan;
}

std::vector < std::vector<double> > leScansCSV(char *nomeArq,const int tamLinha=361)
{
	std::vector < std::vector<double> > scans; scans.reserve(100);
	std::vector<double> tmp;
	std::ifstream dados; dados.open(nomeArq);
	std::vector<std::vector<double>>::iterator it = scans.begin();
	std::string buffer;

	int contador = 0;
	if (dados.is_open()) {
		std::getline(dados, buffer, ',');
		std::cout << "\nvalor lido: ";
		;
		if (contador >= tamLinha) {
			it++;
			contador = 0;
		}
		else {
			(*it).push_back(std::stod(buffer));
		}
	}
	dados.close();
	return scans;
}





int main()
{
	slam s(50, 50, 5000); 
	//teste de funcionamento...
	int i = 0;
	for (i = 0; i < SCANS_DE_TESTE ; i++)
	{
		std::cout << "\nIteracao " << i;
		s.atualiza(cria_scan_de_teste());
	}
	
	system("pause");
    return 0;
}