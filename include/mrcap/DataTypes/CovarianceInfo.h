class CovariancePreset {
public:
    std::string name;
    double covX[3];
    double covTernary[3];
    double covObs[3];
    double covPriors[3];
    double covU[2];

    CovariancePreset(const std::string &n, 
                     double x0, double x1, double x2,
                     double t0, double t1, double t2,
                     double o0, double o1, double o2,
                     double p0, double p1, double p2,
                     double u0, double u1)
        : name(n) 
    {
        covX[0] = x0; covX[1] = x1; covX[2] = x2;
        covTernary[0] = t0; covTernary[1] = t1; covTernary[2] = t2;
        covObs[0] = o0; covObs[1] = o1; covObs[2] = o2;
        covPriors[0] = p0; covPriors[1] = p1; covPriors[2] = p2;
        covU[0] = u0; covU[1] = u1;
    }
};