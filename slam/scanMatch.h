#ifndef SCAN_MATCH_
#define SCAN_MATCH_
#include <vector>
#include "pso.h"
#include "ponto.h"
#include "pose.h"
/*A funcao psoScanMatch recebe uma "diferenca de poses", i.e. uma transformacao,
e retorna o erro medio quadratico de alinhamento entre dois scans. */
pose psoScanMatch(std::vector<ponto> & src, std::vector<ponto> & dst, double estimativaInicial[3]);
pose psoScanMatch(std::vector<ponto> & src, std::vector<ponto> & dst, double estimativaInicial[3], std::vector<pose>& outrasPoses,
    double(*fRestricao)(Eigen::VectorXd, std::vector<pose>&));
#endif