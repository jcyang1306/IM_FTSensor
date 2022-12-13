#include "AdmittanceController.hpp"
#define debug if(1) std::cout

AdmittanceController::AdmittanceController()
{

    Eigen::VectorXd D_a_diag(6);
    D_a_diag << 20, 20, 20, 10, 10, 10;
    D_a = D_a_diag.asDiagonal();

    Eigen::VectorXd M_a_diag(6);
    M_a_diag << 15, 15, 15, 7.5, 7.5, 7.5;
    M_a = M_a_diag.asDiagonal();
    M_a_inv = M_a.inverse();

    dt = 1.0 / 100;
    Pd_tilde = 0.25; 
    E_thres = 10;
    E_max = 50;

    dx_a = Eigen::VectorXd(6);
    F_h = Eigen::VectorXd(6);

    debug << "\n||D_a||: \n" << D_a << std::endl;
    debug << "\n||M_a||: \n" << M_a << std::endl;

}


double AdmittanceController::computeAdmittanceRatio()
{
    if (E_m > E_thres)
        h = (E_m - E_thres) / (E_max - E_thres);
    else 
        h = 0; // dead zone
    return h;
}

Eigen::VectorXd AdmittanceController::computeAdmtOutput(Eigen::VectorXd F_ext)
{
    // Control Loop
    Eigen::VectorXd ddx_a = M_a_inv * (-D_a * dx_a + F_ext);

    double Pi_tilde = dx_a.dot(F_ext);
    double Po_tilde = dx_a.dot(F_h);
    double Pt_tilde = (1 - h) * Pd_tilde;

    double dE = Pi_tilde - Po_tilde - Pt_tilde;
    E_m += dE;

    // Saturated in [0, Em]
    E_m = std::max(E_m, 0.);
    E_m = std::min(E_m, E_max);

    h = computeAdmittanceRatio();
    F_h = h * F_ext; 

    dx_a = dx_a + ddx_a * dt;
    return dx_a;
}

