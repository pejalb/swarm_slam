#include "lineScanMatcher.h"
#include <Eigen/Dense>
#include <cmath>

inline void formaMatrizes(std::vector<ponto> &scan, Eigen::MatrixXd &X,Eigen::VectorXd &y)
{
    for (int i = 0; i < scan.size(); i++){
        X(i, 0) = 1;//define constante aditiva
        X(i, 1) = scan[i].x;//variavel
        y(i) = scan[i].y;
    }
}

double ajustaLinha(std::vector<ponto>& scan, double & coeffAngular, double &coeffLinear, modoAjuste modo)
{
    //retorna o angulo da reta com 0x
    Eigen::MatrixXd X(scan.size(), 2);
    Eigen::VectorXd y(scan.size());
    Eigen::Matrix<double, 2, 1> W;
    formaMatrizes(scan, X, y);
    W = (X.transpose()*X).inverse()*X.transpose()*y;
    coeffAngular = W(1, 0);
    coeffLinear = W(0, 0);
    return std::atan(coeffAngular);
}
