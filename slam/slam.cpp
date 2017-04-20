#include "slam.h"
#define TESTE_CSV_ 1
#include "constantes.h"
#include <fstream>
#include <iostream>
#include <cmath>
#include <Eigen/SparseCholesky>
#include <functional>

inline Eigen::Matrix2d matrizDeRotacao(double angulo)
{
	Eigen::Matrix2d R;
	double ct = std::cos(angulo);
	double st = std::sin(angulo);
	R(0, 0) = ct;
	R(0, 1) = -st;
	R(1, 0) = st;
	R(1, 1) = ct;
	return R;
}

inline Eigen::Matrix3d formaMatrizA(std::vector<pose> &poses, int i, int j)
{
	Eigen::Matrix3d A;
	Eigen::Matrix2d R_i, dR_i, R_ij;
	double ct, st;
	R_ij = matrizDeRotacao((poses[j] - poses[i]).angulo);
	R_i = matrizDeRotacao(poses[i].angulo);
	ct = std::cos(poses[i].angulo);
	st = std::sin(poses[i].angulo);
	dR_i(0, 0) = -st; dR_i(0, 1) = -ct;
	dR_i(1, 0) = ct; dR_i(1, 1) = -st;
	Eigen::Vector2d t;
	t(0) = poses[j].x - poses[i].x;
	t(1) = poses[j].y - poses[i].y;
	//confira o código abaixo!!!
	A.block(0, 0, 2, 2) << -R_ij.transpose()*R_i.transpose();
	A.block(0, 2, 2, 1) << R_ij.transpose()*dR_i.transpose()*t;
	A.block(2, 0, 1, 2) << Eigen::Vector2d::Zero(1, 2); A(2, 2) = -1;
	return A;
}

inline Eigen::Matrix3d formaMatrizB(std::vector<pose> & poses, int i, int j)
{
	Eigen::Matrix3d B;
	Eigen::Matrix2d R_ij;
	R_ij = matrizDeRotacao((poses[j] - poses[i]).angulo);
	B.block(0, 0, 2, 2) << R_ij;
	B.block(0, 2, 2, 1) << Eigen::Vector2d::Zero(2, 1);
	B.block(2, 0, 1, 2) << Eigen::Vector2d::Zero(1, 2);
	B(2, 2) = 1;
	return B;
}

inline void transforma_vetor_pontos(std::vector<ponto>& scan, pose &p)
{
    for (int i = 0; i < scan.size(); i++) {
        scan[i] = p + scan[i];
    }
}

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
void slam::estimaVelocidade(int numPoses)
{
    //estima a velocidade do sistema baseado em suas ultimas numPose poses
    double vx=0.0, vy = 0.0 , w = 0.0;
    pose dp;
    for (int i = numPoses; poses.size()>numPoses && i >0; i--) {
        dp = poses.rbegin()[i] - poses.rbegin()[i-1];
        vx +=  dp.x;
        vy += dp.y;
        w += dp.angulo;
    }
    vx /= (double)numPoses ;
    vy /= (double)numPoses;
    w /= (double)numPoses;
    vel = pose(vx,vy,w);
}
pose slam::estimaProximaPose(void)
{
    estimaVelocidade();
    if (!std::isinf(vel.x) && !std::isinf(vel.y), !std::isinf(vel.angulo))
        return poses.rbegin()[0] + vel;
    else
        return poses.rbegin()[0];
}
slam::slam()
{
    guardaScans = false;
    origem = pose();
    mapa = new gridMap();
}

slam::slam(int linhas, int colunas, double tamanhoCelula,bool guardaScans, bool usaLogOdds, double probPrior, double probOcc, double probFree)
{
    try {
        mapa = new gridMap(linhas, colunas, probPrior, probOcc);//inclua captura de excecao!!!!
    }
    catch (std::bad_alloc)
    {
        std::cerr << "\n Nao foi possivel alocar memoria para o mapa!" << std::endl;
        std::exit(1);
    }
    
    this->tamanhoCelula = tamanhoCelula;
    this->guardaScans = guardaScans;
    poses.reserve(NUM_MINIMO_POSES);
    poses.push_back(pose(0.0, 0.0, 0.0));
    origem = pose(linhas / 2, linhas / 2, M_PI*0.5);
    scans.reserve(NUM_MINIMO_POSES);
    //cria arquivo para armazenamento de poses
    std::ofstream arqPoses;
    arqPoses.open("poses", std::ios::out | std::ios::trunc);
    arqPoses << "x,y,angulo,\n";
    arqPoses << poses.back().x<< "," << poses.back().y << "," << poses.back().angulo << std::endl;
    arqPoses.close();
}

slam::~slam()
{
    //this->corrige();
    this->mapa->salva("mapaFinal");
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
    Eigen::SparseMatrix<double> H(poses.size(),poses.size());
    //]Eigen::VectorXd dX(poses.size(),1), dY(poses.size(), 1), dAng(poses.size(), 1), x(poses.size(), 1),y(poses.size(), 1),ang(poses.size(), 1);
	Eigen::VectorXd b(poses.size(), 1)
	// fill A
	H.setZero();
    //pose tmp;
    int i;
	Eigen::Matrix3d A_ij, B_ij, covMat;
	covMat.setIdentity();
    for ( i = 1; i < poses.size()-1; i++) {
		for (int j = 0; j < poses.size(); j++){
			A_ij = formaMatrizA(poses, i, j);
			B_ij = formaMatrizB(poses, i, j);
			//matriz H
			H.block(i, i, 3, 3) += (A_ij.transpose())*covMat*A_ij;
			H.block(j, i, 3, 3) += (B_ij.transpose())*covMat*A_ij;
			H.block(i, j, 3, 3) += (A_ij.transpose())*covMat*B_ij;
			H.block(j, j, 3, 3) += (B_ij.transpose())*covMat*B_ij;
		}
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
    
}
void salvaWrapper (slam *s,char nome[80])
{
    s->mapa->salva(nome);
}
void slam::atualiza(std::vector<ponto> scan,bool usaOdometria, double odoX,double odoY,double odoAng,std::ofstream &linhas)
{
    paraCoordenadasMapaVector(scan);
    pose p,estimada,anterior;
    if (guardaScans)
        scans.push_back(scan);//sem correcao, possibilitando refinamentos sucessivos, se desejado
    if (scans.size() > 1) {
        pose atual = (poses.rbegin())[0];
        double estimativaInicial[3] = { 0.0,0.0,0.0 };
        //estimada = estimaProximaPose();
        double estimativa[3];
        if (usaOdometria) {
            estimativa[0] = odoX/ESCALA;
            estimativa[1] = odoY/ESCALA;
            estimativa[2] = odoAng;
        }
        else {
            estimativa[0] = estimada.x;
            estimativa[1] = estimada.y;
            estimativa[2] = estimada.angulo;
        }
        try {
            p = psoScanMatch(scans.rbegin()[1], scan, estimativaInicial,mapa);
            //p = psoScanMatch(scans.rbegin()[1], scan, estimativa, mapa);
            poses.push_back(p);
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

        // poses.push_back(pose(0.5*p.x + 0.5*estimada.x, 0.5*p.y + 0.5*estimada.y, 0.5*p.angulo + 0.5*estimada.angulo));
        
        //p = pose(estimativa[0], estimativa[1], estimativa[2]);
        anterior = poses.back();
        p += anterior;
        poses.push_back(p);
	
        //p = (poses.rbegin())[0];
    }
    else
        p = (poses.rbegin())[0];// p.x /= tamanhoCelula; p.y /= tamanhoCelula;
    
    // int i;
    std::vector<ponto>::iterator it = scan.begin();
    transforma_vetor_pontos(scan, p);
	//transforma_vetor_pontos(scan, p + origem);
    scans.pop_back();
    scans.push_back(scan);
    for (it = scan.begin(); it != scan.end(); it++) {        
        //std::cout << "\nALCANCE = " << MAX_ALCANCE;
        if (it->norma() <= MAX_ALCANCE) {
			if (usaOdometria) {	
				p+=pose(odoX,odoY,odoAng);
				linhas<<p.x<<","<<p.y<<","<<it->x<<","<<it->y<<std::endl;
			}
			mapa->marcaLinha(origem.x+p.x,origem.y+ p.y,origem.x+ it->x,origem.y+ it->y);
		}
	}
    static int numScans = 0;   
    if (numScans % 10==0) {
        char s[80];
        std::sprintf(s, "mapa%d", numScans++);
        mapa->salva(s);
    }    
    std::ofstream arqPoses; arqPoses.open("poses", std::ios::out | std::ios::app);
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
    //if (scans.size()%100==0)//imprime um em cada 100 mapas
    
    
}

