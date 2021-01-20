/**
 * @author Alberto M. Esmoris Pena
 *
 * Compare differentiation strategies
 */

#include <iostream>
#include <vector>
#include <cmath>

using namespace std;

const double DELTA=1e-6;

double evaluate(double x, const vector<double>& params){
	double A = params[0];
	double B = params[1];
	double C = params[2];
	double D = params[3];
	return A + B * std::exp(-std::pow(((x - C) / D), 2));
}

double numericalDerivative(int k, double x, vector<double>& params){
    double b, a;
	params[k] -= DELTA;
	b = evaluate(x, params);
	params[k] += 2 * DELTA;
	a = evaluate(x, params);
	params[k] -= DELTA;
	return (a - b) / (2 * DELTA);
}

double hybridDerivative(int k, double & x, vector<double> & params) {
    // Hybrid derivative
    if(k==0){ // Use analytical derivative as it is faster for k[0], a
        return 1;
    }
    else if(k==1){ // Use analytical derivative as it is faster for k[1], b
        return std::exp(-std::pow((x-params[2])/params[3], 2));
    }

    // Use numerical derivative as it is faster for other cases
    return numericalDerivative(k, x, params);
}

double analyticalDerivative(double &x, vector<double>&params, vector<double>& out){
    double db = std::exp(-std::pow((x-params[2])/params[3], 2));
    double x_c = (x-params[2]);
    double b2x_c = 2*params[1]*x_c;
    double dsq = params[3]*params[3];
    double dc = b2x_c*db/dsq;
    double dd = dc/params[3]*x_c;

    out.push_back(1.0); // df/da
    out.push_back(db); // df/db
    out.push_back(dc); // df/dc
    out.push_back(dd); // df/dd
}


int main(void){
    // Build vector x
    vector<double> x;
    for(int i = 0 ; i < 200 ; i++){
        x.push_back((double)i);
    }

    // Build matrix of params
    vector<vector<double>> mparams;
    mparams.push_back(vector<double>({3794.2, 502667.0, 106.557, 28.597}));
    mparams.push_back(vector<double>({21688.5, -21638.9, 160.786, 58.3813}));
    mparams.push_back(vector<double>({4338.97, 571881, 106.616, 28.586}));
    mparams.push_back(vector<double>({3959.34, 521846, 106.616, 28.586}));
    mparams.push_back(vector<double>({4800.59, 503621, 106.642, 28.3111}));
    mparams.push_back(vector<double>({21688.5, -21638.9, 160.786, 58.3813}));
    mparams.push_back(vector<double>({3959.35, 521846, 106.616, 28.586}));
    mparams.push_back(vector<double>({4338.97, 571881, 106.616, 28.586}));
    mparams.push_back(vector<double>({4137.82, 499768, 106.131, 28.7049}));
    mparams.push_back(vector<double>({21688.5, -21638.9, 160.786, 58.3813}));
    mparams.push_back(vector<double>({3959.35, 521846, 106.616, 28.586}));
    mparams.push_back(vector<double>({4033.26, 531589, 106.616, 28.586}));
    mparams.push_back(vector<double>({14633.2, 500243, 106.865, 26.2888}));
    mparams.push_back(vector<double>({4338.97, 571881, 106.616, 28.586}));
    mparams.push_back(vector<double>({21688.5, -21638.9, 160.786, 58.3813}));
    mparams.push_back(vector<double>({3959.37, 521846, 106.616, 28.586}));
    mparams.push_back(vector<double>({4033.26, 531589, 106.616, 28.586}));
    mparams.push_back(vector<double>({43877.4, 430088, 101.828, 26.7593}));
    mparams.push_back(vector<double>({21688.5, -21638.9, 160.786, 58.3813}));
    mparams.push_back(vector<double>({4338.97, 571881, 106.616, 28.586}));
    mparams.push_back(vector<double>({3959.36, 521846, 106.616, 28.586}));
    mparams.push_back(vector<double>({44705.1, 438201, 101.828, 26.7593}));
    mparams.push_back(vector<double>({4033.26, 531589, 106.616, 28.586}));
    mparams.push_back(vector<double>({21688.5, -21638.9, 160.786, 58.3813}));
    mparams.push_back(vector<double>({96660.7, 311603, 99.9859, 13.2695}));
    mparams.push_back(vector<double>({3959.58, 521846, 106.616, 28.586}));
    mparams.push_back(vector<double>({98484.1, 317481, 99.9859, 13.2695}));
    mparams.push_back(vector<double>({21688.5, -21638.9, 160.786, 58.3813}));
    mparams.push_back(vector<double>({4033.26, 531589, 106.616, 28.586}));
    mparams.push_back(vector<double>({117689, 286581, 99.9924, 4.63329}));

    // Compute hybrid and numerical derivatives
    vector<vector<double>> numjacobi;
    vector<vector<double>> hybjacobi;
    vector<vector<double>> anljacobi;
    for(int i = 0 ; i < mparams.size() ; i++){
        for(int j = 0 ; j < x.size() ; j++){
            std::vector<double> nderiv;
            std::vector<double> hderiv;
            std::vector<double> aderiv;
            analyticalDerivative(x[j], mparams[i], aderiv);
            for(int k = 0 ; k < 4 ; k++){
                nderiv.push_back(numericalDerivative(k, x[j], mparams[i]));
                hderiv.push_back(hybridDerivative(k, x[j], mparams[i]));
            }
            numjacobi.push_back(nderiv);
            hybjacobi.push_back(hderiv);
            anljacobi.push_back(aderiv);
        }
    }

    // Compute error
    double n, h, a;
    double aeh, sumh=0.0, minh=99999999.999, maxh=0.0;
    double aea, suma=0.0, mina=99999999.999, maxa=0.0;
    for(int i = 0 ; i < numjacobi.size() ; i++){
        for(int j = 0 ; j < numjacobi[i].size() ; j++){
            n = numjacobi[i][j];
            h = hybjacobi[i][j];
            a = anljacobi[i][j];
            aeh = std::fabs(n-h);
            aea = std::fabs(n-a);
            sumh += aeh;
            suma += aea;
            if(aeh > maxh) maxh = aeh;
            if(aea > maxa) maxa = aea;
            if(aeh < minh) minh = aeh;
            if(aea < mina) mina = aea;
        }
    }

    // Summarize error
    size_t size = numjacobi.size()*numjacobi[0].size();
    double fsize = (double) size;
    cout << "        SUMMARY\n  -------------------\n" << endl;
    cout << "Size of sample: " << size << endl;
    cout << "\nNumerical-Hybrid summary: " << endl;
    cout << "Total absolute error: " << sumh << endl;
    cout << "Mean absolute error: " << sumh/(fsize) << endl;
    cout << "Min absolute error: " << minh << endl;
    cout << "Max absolute error: " << maxh << endl;
    cout << "\nNumerical-Analytical summary: " << endl;
    cout << "Total absolute error: " << suma << endl;
    cout << "Mean absolute error: " << suma/(fsize) << endl;
    cout << "Min absolute error: " << mina << endl;
    cout << "Max absolute error: " << maxa << endl;

}
