#ifndef DETECTOR_CANTOS_
#define DETECTOR_CANTOS_
#include "ponto.h"
#include <vector>

bool detectaCanto(std::vector<ponto> scan, int inicio, int fim, ponto *canto, double tolerancia = 1e-3);

#endif