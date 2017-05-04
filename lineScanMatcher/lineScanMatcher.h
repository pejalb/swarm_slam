#ifndef LINE_SCAN_MATCHER
#define LINE_SCAN_MATCHER
#include "ponto.h"
#include <vector>

enum modoAjuste {eqNormais,iterativo};
double ajustaLinha(std::vector<ponto> &scan, double &coeffAngular, double &coeffLinear,modoAjuste modo = eqNormais);

#endif
