#include "slam.h"
#include <fstream>
#include <iostream>
#include <cmath>
#include <Eigen/SparseCholesky>

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
inline void slam::paraCoordenadasMapaVector(std::vector<ponto> &scan)
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
    try {
        mapa = new gridMap(linhas, colunas, usaLogOdds, probPrior, probOcc, probFree);//inclua captura de excecao!!!!
    }
    catch (std::bad_alloc)
    {
        std::cerr << "\n Nao foi possivel alocar memoria para o mapa!" << std::endl;
        std::exit(1);
    }
    
    this->tamanhoCelula = tamanhoCelula;
    this->guardaScans = guardaScans;
    poses.reserve(NUM_MINIMO_POSES);
    poses.push_back(pose(linhas/2, colunas/2, 0));
    scans.reserve(NUM_MINIMO_POSES);
    //cria arquivo para armazenamento de poses
    std::ofstream arqPoses;
    arqPoses.open("poses", std::ios::out | std::ios::trunc);
}

slam::~slam()
{
    this->corrige();
    this->mapa->gravaMapa("mapaFinal");
    delete mapa;
}

inline double fRestricaoPoses(Eigen::VectorXd v, std::vector<pose>& outrasPoses)
{
    double erro = 0.0;
    int passos = 3;
    for (std::vector<pose>::reverse_iterator it = outrasPoses.rbegin(); passos > 0 && it!=outrasPoses.rend();passos--,it++) {
        erro += (pose(v) - *it).quadradoNorma();
    }
    return erro;//se a restricao nao-linear nao foi fornecida...usa um "placeholder"
}

void slam::corrige()
{
    // monta o sistema linear
    Eigen::SparseMatrix<double> A(poses.size(),poses.size());
    Eigen::VectorXd dX(poses.size(),1), dY(poses.size(), 1), dAng(poses.size(), 1), x(poses.size(), 1),y(poses.size(), 1),ang(poses.size(), 1);
    // fill A
    A.setIdentity();
    pose tmp;
    int i;

    for ( i = 1; i < poses.size()-1; i++) {
        tmp = poses[i + 1] - poses[i];
        A.insert(i, i + 1)=-1;
        dX(i) = tmp.x;
        dY(i) = tmp.y;
        dAng(i) = tmp.angulo;
    }
    
    // fill b
    // solve Ax = b
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
    solver.compute(A);
    if (solver.info() != Eigen::Success) {
        // decomposition failed
        return;
    }
    x = solver.solve(dX);
    y = solver.solve(dY);
    ang = solver.solve(dAng);
    // corrige as poses
    poses.clear();
    std::ofstream arqPoses; arqPoses.open("poses", std::ios::out | std::ios::trunc);
    arqPoses << "\nx,y,angulo,\n";
    poses.push_back(pose(mapa->tamanhoHorizontal() / 2, mapa->tamanhoVertical() / 2, 0));
    for (i = 1; i < scans.size(); i++) {
        poses.push_back(poses[i-1]+pose(x(i), y(i), ang(i)));
        arqPoses << poses[i].x << "," << poses[i].y << "," << poses[i].angulo << std::endl;
    }
    arqPoses.close();
    this->mapa->reset();
   // int i;
    i=0;
    for (std::vector<std::vector<ponto> >::iterator sc = scans.begin(); sc != scans.end(); sc++) {
        for (std::vector<ponto>::iterator it = sc->begin(); it != sc->end(); it++) {
            ponto corrigido(((poses[i]) + (*it)));
            mapa->marcaLinha(std::round(poses[i].x), std::round(poses[i].y), std::round(corrigido.x), std::round(corrigido.y));
        }
    }
}

void slam::atualiza(std::vector<ponto> scan)
{
    if (guardaScans)
        scans.push_back(scan);//sem correcao, possibilitando refinamentos sucessivos, se desejado
    paraCoordenadasMapaVector(scan);
    pose atual = (poses.rbegin())[0];
    double estimativa[3] = { atual.x,atual.y,atual.angulo };
    pose p;
    try {
        p = psoScanMatch(scan, scans.rbegin()[0], estimativa);
        //p = psoScanMatch(scan, scans.rbegin()[0], estimativa,
            //poses, fRestricaoPoses);
    }
    catch (const std::exception&) {
        scans.pop_back();
        return;
    }
    //se chegou aqui nao houve excecao
    //p += atual;
    //monta o sistema linear aproximado

    poses.push_back(p);
    // int i;
    std::vector<ponto>::iterator it = scan.begin();
    for (it = scan.begin(); it != scan.end(); it++) {
        ponto corrigido = (p + (*it));
        mapa->marcaLinha(std::round(p.x), std::round(p.y), std::round(corrigido.x), std::round(corrigido.y));
    }
    std::ofstream arqPoses; arqPoses.open("poses", std::ios::out | std::ios::app);
    if (poses.size() == 1) {
        arqPoses << "\nx,y,angulo,\n";
    }
    arqPoses << poses.back().x << "," << poses.back().y << "," << poses.back().angulo << std::endl;
    //std::cout << poses.back().x << "," << poses.back().y << "," << poses.back().angulo << std::endl;
    arqPoses.close();
    //scans x (sistema local)
    /*std::ofstream arqScansX; arqScansX.open("arqScansX.csv", std::ios::out | std::ios::trunc);
    for (i = 0; i < scans.size(); i++){
    for (size_t j = 0; j < scans[i].size(); j++){

    }
    }*/
    /*arqScansX.close();*/
    //arqScansY.close();

    //mapa->gravaMapaSeq("mapa");
}

