#ifndef GRIDMAP_
#define GRIDMAP_
#include <Eigen/Dense>
#include <vector>
#include "ponto.h"

class gridMap {
    private:
        Eigen::MatrixXd mapa;
        /*As constante de dimensao maxLinhas e maxColunas sao assim mapeadas
        maxLinhas = max valor ao longo das abcissas ("x")
        maxColunas = max valor ao longo das ordenadas("y")
        */
        double incrementoFundamental;//, decrementoFundamental;
        bool maiorLinhaPertencente(int &x1, int &y1, int &x2, int &y2);
        //EXPERIMENTAL!!
        void aumentaMapa(void);        
    public:
        int maxLinhas, maxColunas;//perigoso!!!
        //construtores
        gridMap();
        gridMap(int linhas, int colunas,double probPrior=0.5,double probOcc=0.9);
        //constroi mapa local partindo de um vetor de pontos (em coordenadas de mapa)
        gridMap(int linhas,int colunas,std::vector<ponto> & scan, double probPrior = 0.5, double probOcc = 0.9);
        //destrutor
        ~gridMap();
        //operadores
        double &operator()(int linha, int coluna);
        double operator- (const gridMap&  rhs);
        double operator-(std::vector<ponto> &scans);
        //limpa mapa
        void limpa(void);
        //incrementos
        inline double incrementa(int linha, int coluna);
        //decrementos
        inline double decrementa(int linha, int coluna);
        //determina pontos a marcar
        inline bool pertence(int x, int y);
        void marcaLinha(int x1, int y1, int x2, int y2);        
        //salva em arquivo
        void salva(char *nomeArq);
        //para retro compatibilidade
        double leMapa(int linha, int coluna);
        double alteraMapa(int linha, int coluna, double valor);
};

#endif