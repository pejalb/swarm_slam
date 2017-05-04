#include "ponto.h"
#include "lineScanMatcher.h"
#include <iostream>
#include <vector>

int main(int argc, char **argv)
{
    std::vector <ponto> scanTeste;
    double m = 0.0;
    double b = 0.0;
    scanTeste.push_back(ponto(0, 0));
    scanTeste.push_back(ponto(1, 1));
    scanTeste.push_back(ponto(2, 2));
    scanTeste.push_back(ponto(3, 3));
    double theta=ajustaLinha(scanTeste, m, b);
    std::cout << "\nLinha ajustada: m = " << m << " b = " << b<<"\n Angulo = "<<theta<<"\n";
    system("pause");
    return 0;
}