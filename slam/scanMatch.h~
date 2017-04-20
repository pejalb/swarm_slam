#ifndef SCAN_MATCH_
#define SCAN_MATCH_
#include <vector>
#include <exception>
#include "pso.h"
#include "ponto.h"
#include "pose.h"
#include "gridMap.h"
/*A funcao psoScanMatch recebe uma "diferenca de poses", i.e. uma transformacao,
e retorna o erro medio quadratico de alinhamento entre dois scans. */
pose psoScanMatch(std::vector<ponto> & src, std::vector<ponto> & dst, double estimativaInicial[3],gridMap *m) throw(std::domain_error);
pose psoScanMatch(std::vector<ponto> & src, std::vector<ponto> & dst, double estimativaInicial[3], gridMap *m, std::vector<pose>& outrasPoses,
    double(*fRestricao)(Eigen::VectorXd, std::vector<pose>&)) throw(std::domain_error);
#endif