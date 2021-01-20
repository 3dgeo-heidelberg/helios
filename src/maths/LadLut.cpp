#include "LadLut.h"
#include "MathConstants.h"

// ***  CONSTANTS  *** //
// ******************* //
const double LadLut::eps = 0.0000001;

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
LadLut::LadLut(LadLut &ll){
    X = ll.X;
    Y = ll.Y;
    Z = ll.Z;
    G = ll.G;
    angles = ll.angles;
}

// ***  M E T H O D S  *** //
// *********************** //
void LadLut::computeAngles(){
    // Compute angles with respect to (0, 0, 1)
    angles = std::vector<double>(0);
    for(size_t i = 0 ; i < X.size() ; i++){
        double angle = std::acos(Z[i]);
        if(X[i] < 0.0) angle = PI_2 - angle;
        angles.push_back(angle);
    }
}

double LadLut::computeSigma(double padBVTotal, double x, double y, double z){
    double g = interpolate(x, y, z);
    return padBVTotal * g / PI_2;
}

double LadLut::interpolate(double x, double y, double z){
    // Transform (x,y,z) to (wx, wy, wz)
    double wx, wy, wz; // Components of vector to interpolate g w.r.t
    fastTransformToLadLutDomain(x, y, z, wx, wy, wz);

    // Find immediately after-before directions in look-up table
    size_t uIdx, vIdx;
    double ux, uy, uz; // Components of known vector after w
    double vx, vy, vz; // Components of known vector after w
    double a, b; // Angular difference with respect to w for both, u and v
    findEnvelopmentDirections(wx,wy,wz, uIdx,ux,uy,uz, vIdx,vx,vy,vz, a,b);
    b = fabs(b); // Absolute value of b, to compute DELTA and beta later

    // Compute angular divergence between directions
    if(a > -eps && a < eps){ // u is w, no need to interpolate
        return G[uIdx];
    }
    if(b > -eps && b < eps){ // v is w, no need to interpolate
        return G[vIdx];
    }

    // Interpolate g for w
    double DELTA = a+b;
    double alpha = b/DELTA;
    double beta = a/DELTA;
    return alpha * G[uIdx] + beta * G[vIdx];
}

// ***  INNER METHODS  *** //
// *********************** //
void LadLut::transformToLadLutDomain(
    double a, double b, double c,
    double &x, double &y, double &z
){
    // Compute transformation rotation angle
    double alpha = std::atan(b/a);
    double beta = alpha/2.0;

    // Prepare transformation
    double bCos = std::cos(beta);
    double bCos2 = 2.0 * bCos;
    double bSin = std::sin(beta);

    // Apply transformation
    x = bCos2 * (bCos*a + bSin*b) - a;
    y = bCos2 * (bCos*b - bSin*a) - b;
    //z = 2.0 * (bCos*bCos*c + bSin*bSin*c) - c;
    z = c;
}

inline void LadLut::fastTransformToLadLutDomain(
    double a, double b, double c,
    double &x, double &y, double &z
){
    x = std::sqrt(a*a + b*b);
    y = 0.0;
    z = c;
}

void LadLut::findEnvelopmentDirections(
    double wx, double wy, double wz,
    size_t &uIdx, double &ux, double &uy, double &uz,
    size_t &vIdx, double &vx, double &vy, double &vz,
    double &a, double &b
){
    /*
     * If necessary, this function could be speed-up.
     * As long as it is possible to assume X,Y,Z vectors are ordered,
     * the loop could be improved speculating with step size so it possible
     * to find desired values faster.
     */
    // Find angle for input vector
    double wAngle = std::acos(wz);

    // Find u and v
    uIdx = 0;   vIdx = 0;
    a = std::numeric_limits<double>::max();
    b = std::numeric_limits<double>::lowest();
    for(size_t i = 0 ; i < X.size() ; i++){
        double angleDiff = angles[i] - wAngle;
        if(angleDiff >= 0 && angleDiff < a){ // update u
            a = angleDiff;
            uIdx = i;
        }
        if(angleDiff <= 0 && angleDiff > b){ // update v
            b = angleDiff;
            vIdx = i;
        }
    }

    // Extract output components
    ux = X[uIdx];   uy = Y[uIdx];   uz = Z[uIdx];
    vx = X[vIdx];   vy = Y[vIdx];   vz = Z[vIdx];
}
