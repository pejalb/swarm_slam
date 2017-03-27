#include "slam.h"
#include <fstream>
#include <cmath>

inline ponto slam::paraCoordenadasMapa(double x, double y)
{
	return ponto(x/tamanhoCelula,y/tamanhoCelula);
}
inline ponto slam::paraCoordenadasMapa(ponto &p)
{
	p.x = p.x / tamanhoCelula;
	p.y = p.y / tamanhoCelula;
	return ponto(p.x,p.y);
}
inline void slam::paraCoordenadasMapaVector(std::vector<ponto>& scan)
{
	for (std::vector<ponto>::iterator i = scan.begin(); i != scan.end(); i++){
		paraCoordenadasMapa(*i);
	}
}
slam::slam()
{
    guardaScans = false;
    mapa = new gridMap();
}

slam::slam(int linhas, int colunas, double tamanhoCelula,bool guardaScans, bool usaLogOdds, double probPrior, double probOcc, double probFree)
{
    mapa = new gridMap(linhas, colunas, usaLogOdds, probPrior, probOcc, probFree);//inclua captura de excecao!!!!
    this->tamanhoCelula = tamanhoCelula;
    this->guardaScans = guardaScans;
    poses.reserve(NUM_MINIMO_POSES);
    poses.push_back(pose(linhas/2, colunas/2, 0));
    scans.reserve(NUM_MINIMO_POSES);
}

slam::~slam()
{
    delete mapa;
}

void slam::atualiza(std::vector<ponto>& scan)
{
	paraCoordenadasMapaVector(scan);
	if (guardaScans)
		scans.push_back(scan);//sem correcao, possibilitando refinamentos sucessivos, se desejado
	pose atual = (poses.rbegin())[0];
	double estimativa[3] = { atual.x,atual.y,atual.angulo };
	pose p = psoScanMatch(scan, scans.rbegin()[0], estimativa);
	p = p + atual;
	poses.push_back(p);
	int i;
	for (i = 0; i < LEITURAS_POR_SCAN; i++) {
		scan[i] = p + scan[i];
		mapa->marcaLinha(std::round(p.x),std::round(p.y), std::round(scan[i].x), std::round(scan[i].y));
	}
	if (poses.size() == 0) {
		std::ofstream arqPoses; arqPoses.open("poses", std::ios::out | std::ios::trunc);
		arqPoses << "\nx,y,angulo,\n";
		arqPoses << poses[i].x << "," << poses[i].y << "," << poses[i].angulo << std::endl;
		arqPoses.close();
	}
	else {
		std::ofstream arqPoses; arqPoses.open("poses", std::ios::out | std::ios::app);
		arqPoses << poses.back().x << "," << poses.back().y << "," << poses.back().angulo << std::endl;
		arqPoses.close();
	}
	//scans x (sistema local)
	/*std::ofstream arqScansX; arqScansX.open("arqScansX.csv", std::ios::out | std::ios::trunc);
	for (i = 0; i < scans.size(); i++){
		for (size_t j = 0; j < scans[i].size(); j++){

		}
	}*/
	/*arqScansX.close();*/
	//arqScansY.close();
    
    mapa->gravaMapaSeq("mapa");
}

