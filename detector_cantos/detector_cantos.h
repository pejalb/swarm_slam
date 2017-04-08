#ifndef DETECTOR_CANTOS_
#define DETECTOR_CANTOS_
#include "ponto.h"
#include <vector>

bool detectaCanto(const std::vector<ponto>& scan, int inicio, int fim, int &idx, double tolerancia = 1e-3);


#endif