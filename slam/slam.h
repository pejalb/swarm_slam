#ifndef SLAM_
#define SLAM_
#include "gridMap.h"
#include "scanMatch.h"
//constantes inerentes ao problema
#define LEITURAS_POR_SCAN 541
//numero minimo de poses cujo armazenamento e garantido
#define NUM_MINIMO_POSES 3

class slam {
    private:
        //define um vetor de poses vistas, permite descrever a trajetoria
        std::vector<pose> poses;
        std::vector<std::vector<ponto>> scans;
        //mapa sendo construido
        gridMap *mapa;
        //se guarda scans==true todos os scans serao armazenados, caso contrario, apenas dois serao armazenados, o atual e o predecessor imediato
        bool guardaScans;
        //escala usada
        double tamanhoCelula;
		//define restricoes lineares entre as poses
		inline ponto paraCoordenadasMapa(double x, double y);
		inline ponto paraCoordenadasMapa(ponto &);
		inline void paraCoordenadasMapaVector(std::vector<ponto> &);
    public:
        slam();//ctor padrao
        slam(int linhas, int colunas,double tamanhoCelula,bool guardaScans=true,bool usaLogOdds = true, double probPrior = 0.5, double probOcc = 0.9, double probFree = 0.1);
        ~slam();//dtor padrao
        //atualizacao
        void atualiza(std::vector<ponto> &scan);//atualiza a pose com um novo scan
};
#endif