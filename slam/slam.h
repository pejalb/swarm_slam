#ifndef SLAM_
#define SLAM_
#include "gridMap.h"
#include "scanMatch.h"

class slam {
    private:
        //define um vetor de poses vistas, permite descrever a trajetoria
        std::vector<pose> poses;
        std::vector<std::vector<ponto> > scans;
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
        pose origem;
        //funcoes de uso interno para a reducao do espaco de buscas
        void estimaVelocidade(int numPoses=5);
        //passo estimado entre poses
        pose vel;
        pose estimaProximaPose(void);
    public:
        slam();//ctor padrao
        slam(int linhas, int colunas,double tamanhoCelula,bool guardaScans=true,bool usaLogOdds = true, double probPrior = 0.5, double probOcc = 0.9, double probFree = 0.1);
        ~slam();//dtor padrao
        //atualizacao
        void atualiza(std::vector<ponto> scan, bool usaOdometria = false,
            double odoX=0.0, double odoY=0.0, double odoAng=0.0);//atualiza a pose com um novo scan
        void corrige(void);//calcula poses etc, mas nao atualiza o mapa
};
#endif