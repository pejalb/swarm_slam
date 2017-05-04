#ifndef GRIDMAP_
#define GRIDMAP_

#ifndef EIGEN_DENSE_H
#define EIGEN_DENSE_H
#endif
#include <Eigen/Dense>
#include <vector>

typedef std::vector<std::vector<long double> > array_type;
/*Versão intermediária da transição para o uso do Eigen*/
class gridMap {
	private:
		array_type grid;//mapa armazenado
		int numLinhas, numColunas;
        double decremento;//theta
        double incremento;//beta
		bool logOdds;
		const double probOcupacao, probPriori,probLivre;
		long int numFrames;
        long double logOddsPrior;
	public:
		gridMap();//ctr padrao
		//gridMap(int n, bool usaLogOdds = true, double probPrior = 0.5, double probOcc = 0.9, double probFree = 0.1);//cria mapa n x n
		gridMap(int linhas, int colunas,bool usaLogOdds = true,double probPrior = 0.5, double probOcc = 0.9, double probFree = 0.1);//cria mapa de tamanho linhas x colunas
		~gridMap();//destrutor padrao
		//acesso
		long double leMapa(int linha, int coluna);
		long double alteraMapa(int linha, int coluna, long double novoValor);
        long double incrementa(int linha, int coluna);
        long double decrementa(int linha, int coluna);
        int tamanhoHorizontal(void);
        int tamanhoVertical(void);
		void marcaLinha(int xInicial, int yInicial, int xFinal, int yFinal, bool decrementa = true);
		//retorna mapa como matriz eigen
		/*Pode ser ineficiente para mapas grandes!!!
		Vale considerar o uso de memoria compartilhada e um ponteiro para acesso direto (se necessario).*/
		Eigen::MatrixXd retornaMatriz(void);
		//operacoes de comparacao
		bool operator==(gridMap &outroMapa);
		bool operator!=(gridMap &outroMapa);
		//exporta como arquivo csv
		bool gravaMapa(char *nomeArq);//um unico uso
		bool gravaMapaSeq(char *nomeArq);
};

#endif