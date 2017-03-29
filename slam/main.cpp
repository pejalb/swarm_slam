#define TESTE_SLAM_BASE_
#include "slam.h"
#include <cstdlib>
#include <vector>
#include <iostream>
#include <fstream>
#include <Eigen/Sparse>
#include <string>
#include <sstream>
#include <iterator>
#include "constantes.h"
#include <algorithm>

#define TESTE_CSV_ 1


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

void Tokenize(const std::string& str,
	std::vector<double>& tokens,
	const std::string& delimiters = ",")
{
	// Skip delimiters at beginning.
	std::string::size_type lastPos = str.find_first_not_of(delimiters, 0);
	// Find first "non-delimiter".
	std::string::size_type pos = str.find_first_of(delimiters, lastPos);

	while (std::string::npos != pos || std::string::npos != lastPos)
	{
		// Found a token, add it to the vector.
		tokens.push_back(std::stod(str.substr(lastPos, pos - lastPos)));
		// Skip delimiters.  Note the "not_of"
		lastPos = str.find_first_not_of(delimiters, pos);
		// Find next "non-delimiter"
		pos = str.find_first_of(delimiters, lastPos);
	}
}

std::vector < std::vector<double> > leScansCSV(char *nomeArq,const int tamLinha=361)
{
	std::vector < std::vector<double> > scans; scans.reserve(100);
	std::vector<double> tmp;
	std::ifstream dados; dados.open(nomeArq);
	std::string buffer;
	bool teste = dados.is_open();
	int contador = 0;
	if (dados.is_open()) {
		while (std::getline(dados, buffer)){
			Tokenize(buffer, tmp);
			scans.push_back(tmp);
            tmp.clear();
		}
		dados.close();
	}	
	return scans;
}

void paraCoordenadasCartesianas(std::vector < std::vector<double> > polarScans, std::vector <std::vector<ponto> > &pontos,const double espacoAngular=A_270_GRAUS,const double offsetAngular = A_135_GRAUS)
{
	std::vector< std::vector<ponto> >::iterator itDest = pontos.begin();
	std::vector<ponto> buffer; buffer.reserve(LEITURAS_POR_SCAN);	
	double incrementoAngular = espacoAngular / (double)LEITURAS_POR_SCAN;
	int i;

	for (std::vector < std::vector<double> >::iterator it = polarScans.begin(); it != polarScans.end();it++) {
		for (i = 0; i < it->size(); i++) {
			//buffer.push_back(ponto::polarParaCartesiano(it->at(i), ((double)i)*incrementoAngular - offsetAngular));
			buffer.push_back(ponto::polarParaCartesiano(it->at(i), ((double)i)*incrementoAngular - offsetAngular));
		}
		pontos.push_back(buffer);
        buffer.clear();
	}
}



int main()
{
	slam s(200, 200, 1000); 
	//teste de funcionamento...
	auto tmp = leScansCSV("mit-csail.csv");
	if(TESTE_CSV_) {
		std::vector <std::vector<ponto> > leituras; leituras.reserve(tmp.size());
		paraCoordenadasCartesianas(tmp,leituras, A_180_GRAUS, A_90_GRAUS);
		int i = 0;
		for (i = 0; i < leituras.size(); i++) {
			std::cout << "\nIteracao " << i;
			s.atualiza(leituras.at(i));
		}
	}
	else {
		int i = 0;
		for (i = 0; i < SCANS_DE_TESTE; i++){
			std::cout << "\nIteracao " << i;
			s.atualiza(cria_scan_de_teste());
		}
	}	
	system("pause");
    return 0;
}