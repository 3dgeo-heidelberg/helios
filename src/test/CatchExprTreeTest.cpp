#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#undef WARN
#undef INFO
#include "logging.hpp"

#include <adt/exprtree/UnivarExprTreeNode.h>
#include <adt/exprtree/RegUnivarExprTreeNode.h>
#include <adt/exprtree/UnivarExprTreeStringFactory.h>
#include <adt/exprtree/RegUnivarExprTreeStringFactory.h>
#include <adt/bintree/BinaryTreeFastDepthIterator.h>

#include <memory>
#include <vector>

double eps = 0.00001;

template <typename InputType, typename OutputType>
bool validate(
    std::shared_ptr<IExprTreeNode<InputType, OutputType>> f,
    std::vector<InputType> const &t,
    std::vector<OutputType> const &y
) {
    std::vector<OutputType> ft;
    for (InputType const & ti : t) ft.push_back(f->eval(ti));
    if (ft.size() != y.size()) return false;
    for (size_t i = 0; i < ft.size(); ++i) {
        double const absDiff = std::fabs(ft[i]-y[i]);
        if(absDiff > eps) return false;
    }
    return true;
}

template <typename InputType, typename OutputType>
bool validate(
    std::shared_ptr<IExprTreeNode<InputType, OutputType>> f,
    std::shared_ptr<IExprTreeNode<InputType, OutputType>> g,
    std::vector<InputType> const &t
) {
    for (double const &ti : t) {
        double const absDiff = std::abs(f->eval(ti)-g->eval(ti));
        if(absDiff > eps) return false;
    }
    return true;
}

template <typename NumericType>
bool validatePowIPow(
    std::shared_ptr<UnivarExprTreeNode<NumericType>> f,
    size_t const expectedPowCount,
    size_t const expectedIPowCount
) {
    size_t powCount = 0, ipowCount = 0;
    BinaryTreeFastDepthIterator<UnivarExprTreeNode<NumericType>> btdi(f.get());
    while (btdi.hasNext()) {
        UnivarExprTreeNode<NumericType> *node = btdi.next();
        if (!node->isOperator()) continue;
        if (node->op == node->OP_POW) ++powCount;
        if (node->op == node->OP_IPOW) ++ipowCount;
    }
    return powCount == expectedPowCount && ipowCount == expectedIPowCount;
}

template <typename NumericType>
bool validatePowIPow(
    std::shared_ptr<IExprTreeNode<NumericType, NumericType>> f,
    size_t const expectedPowCount,
    size_t const expectedIPowCount
) {
    return validatePowIPow(
        std::static_pointer_cast<UnivarExprTreeNode<NumericType>>(f),
        expectedPowCount,
        expectedIPowCount
    );
}

TEST_CASE("Univariate Expression Tree Tests") {
    UnivarExprTreeStringFactory<double> uetsf;
    std::shared_ptr<IExprTreeNode<double, double>> node, node2;
    std::vector<double> t{
        -10, -3.14, -2.72, -1.1, -0.34, 0, 0.34, 1.1, 2.72, 3.14, 10
    }; // Input domain for all cases
    std::vector<double> y; // Expected output

    SECTION("1+(t-1)^2") {
        node = uetsf.makeShared("1+(t-1)^2");
        y = {122, 18.1396, 14.8384, 5.41, 2.7956, 2, 1.4356, 1.01, 3.9584, 5.5796, 82};
        REQUIRE(validate(node, t, y));
    }
    SECTION("exp(t)+3*cos(1+2*t)") {
        node = uetsf.makeShared("exp(t)+3*cos(1+2*t)");
        y = {2.96615925, 1.65614048, -0.741224555, 1.41994435, 3.55947658, 2.62090692, 1.07798733, 0.00928169656, 18.1435115, 24.7328066, 22024.8226};
        REQUIRE(validate(node, t, y));
    }
    SECTION("ln(t/2)*sin(t)") {
        node = uetsf.makeShared("ln(t/2)*sin(t)");
        y = {NAN, NAN, NAN, NAN, NAN, NAN, -0.590924735, -0.532796735, 0.125827096, 0.000718406901, -0.875568201};
        REQUIRE(validate(node, t, y));
    }
    SECTION("acos(1/3)+t/pi") {
        node = uetsf.makeShared("acos(1/3)+t/pi");
        y = {-1.95213944, 0.23146637, 0.36515653, 0.88081854, 1.12273406, 1.23095942, 1.33918478, 1.58110029, 2.09676231, 2.23045246, 4.41405828};
        REQUIRE(validate(node, t, y));
    }
    SECTION("5*t") {
        node = uetsf.makeShared("5*t");
        y = {-50, -15.7, -13.6, -5.5, -1.7, 0, 1.7, 5.5, 13.6, 15.7, 50};
        REQUIRE(validate(node, t, y));
    }
    SECTION("t") {
        node = uetsf.makeShared("t");
        y = t;
        REQUIRE(validate(node, t, y));
    }
    SECTION("7") {
        node = uetsf.makeShared("7");
        y = std::vector<double>(t.size(), 7);
        REQUIRE(validate(node, t, y));
    }
    SECTION("4*t^3 + 3*t^2 + 2*t - 1") {
        node = uetsf.makeShared("4*t^3 + 3*t^2 + 2*t - 1");
        y = {-3721, -101.537776, -64.739392, -4.894, -1.490416, -1, 0.184016, 10.154, 107.129792, 158.695376, 4319};
        REQUIRE(validate(node, t, y));
    }
    SECTION("3600 - t^(t+1)") {
        node = uetsf.makeShared("3600 - t^(t+1)");
        y = {3600, NAN, NAN, NAN, NAN, 3600, 3599.7644, 3598.77841, 3558.6385, 3485.89919, -99999996400.0};
        REQUIRE(validate(node, t, y));
    }
    SECTION("cos(t)") {
        node = uetsf.makeShared("cos(t)");
        y = {-0.83907153, -0.99999873, -0.91243836, 0.45359612, 0.94275467, 1, 0.94275467, 0.45359612, -0.91243836, -0.99999873, -0.83907153};
        REQUIRE(validate(node, t, y));
    }
    SECTION("sin(t^2 + 3)") {
        node = uetsf.makeShared("sin(t^2 + 3)");
        y = {0.62298863, 0.28904527, -0.82692784, -0.87643472, 0.02598973, 0.14112001, 0.02598973, -0.87643472, -0.82692784, 0.28904527, 0.62298863};
        REQUIRE(validate(node, t, y));
    }
    SECTION("3.1 + 5 * 7 + 4.4") {
        node = uetsf.makeShared("3.1 + 5.0 * 7 +4.4");
        y = std::vector<double>(t.size(), 42.5);
        REQUIRE(validate(node, t, y));
    }
    SECTION("-t") {
        node = uetsf.makeShared("-t");
        y = {10, 3.14, 2.72, 1.1, 0.34, 0, -0.34, -1.1, -2.72, -3.14, -10};
        REQUIRE(validate(node, t, y));
    }
    SECTION("-1") {
        node = uetsf.makeShared("-1");
        y = std::vector<double>(t.size(), -1);
        REQUIRE(validate(node, t, y));
    }
    SECTION("-t*7") {
        node = uetsf.makeShared("-t*7");
        y = {70, 21.98, 19.04, 7.7, 2.38, 0, -2.38, -7.7, -19.04, -21.98, -70};
        REQUIRE(validate(node, t, y));
    }
    SECTION("-1+7") {
        node = uetsf.makeShared("-1 +7");
        y = std::vector<double>(t.size(), 6);
        REQUIRE(validate(node, t, y));
    }
    SECTION("((t-1)*((1+t) / 2))/(t+0.01)^2") {
        node = uetsf.makeShared("((t-1)*((1+t) / 2))/(t+0.01)^2");
        y = {0.495991487, 0.452163439, 0.435614983, 0.0883763993, -4.06060606, -5000, -3.60979592, 0.0852203555, 0.429255726, 0.446439909, 0.494011483};
        REQUIRE(validate(node, t, y));
    }
    SECTION("(pi/3)^(abs(t)^(e-1))") {
        node = uetsf.makeShared("(pi/3)^(abs(t)^(e-1))");
        y = {11.14208743, 1.39014251, 1.29353943, 1.05582653, 1.00725079, 1, 1.00725079, 1.05582653, 1.29353943, 1.39014251, 11.14208743};
        REQUIRE(validate(node, t, y));
    }
    SECTION("atan2(t, 1) + pi") {
        node = uetsf.makeShared("atan2(t, 1) +pi");
        y = {1.67046498, 1.87911199, 1.92310505, 2.30861139, 2.81385415, 3.14159265, 3.46933116, 3.97457392, 4.36008026, 4.40407332, 4.61272033};
        REQUIRE(validate(node, t, y));
    }
    SECTION("t-(-t) == 2*t") {
        node = uetsf.makeShared("t-(-t)");
        node2 = uetsf.makeShared("2*t");
        REQUIRE(validate(node, node2, t));
    }
    SECTION("t+(1)") {
        node = uetsf.makeShared("t+(1)");
        y = {-9, -2.14, -1.72, -0.1, 0.66, 1, 1.34, 2.1, 3.72, 4.14, 11};
        REQUIRE(validate(node, t, y));
    }
    SECTION("(-1)+t") {
        node = uetsf.makeShared("(-1)+t");
        y = {-11, -4.14, -3.72, -2.1, -1.34, -1, -0.66, 0.1, 1.72, 2.14, 9};
        REQUIRE(validate(node, t, y));
    }
    SECTION("(-1+1)+t") {
        node = uetsf.makeShared("(-1+1)+t");
        REQUIRE(validate(node, t, t));
    }
    SECTION("(1-(-1))") {
        node = uetsf.makeShared("(1-(-1))");
        y = std::vector<double>(t.size(), 2);
        REQUIRE(validate(node, t, y));
    }
    SECTION("(1-(1))") {
        node = uetsf.makeShared("(1-(1))");
        y = std::vector<double>(t.size(), 0);
        REQUIRE(validate(node, t, y));
    }
    SECTION("abs(t) = abs(-t) = abs(-1*t)") {
        node = uetsf.makeShared("abs(t)");
        node2 = uetsf.makeShared("abs(-t)");
        REQUIRE(validate(node, node2, t));
        node = uetsf.makeShared("abs(-1*t)");
        REQUIRE(validate(node, node2, t));
    }
    SECTION("abs(-t) - abs(-1*t) + abs(t) - abs(-2*t+(t))") {
        node = uetsf.makeShared("abs(-t) - abs(-1*t) + abs(t) - abs(-2*t+(t))");
        y = std::vector<double>(t.size(), 0);
        REQUIRE(validate(node, t, y));
    }
    SECTION("1+atan2(3*t-1, 1)") {
        node = uetsf.makeShared("1 + atan2(3*t-1, 1)");
        y = {-0.53854944, -0.47512005, -0.46205665, -0.34229969, -0.11111695, 0.21460184, 1.01999733, 2.16066899, 2.43202915, 2.4525852, 2.53632723};
        REQUIRE(validate(node, t, y));
    }
    SECTION("1+atan2(3*t-1, (t-1)*2)") {
        node = uetsf.makeShared("1+atan2(3*t-1, (t-1)*2)");
        y = {-1.18798772, -1.24225167, -1.25294846, -1.34443033, -1.49570776, -1.67794504, 4.1264423, 2.48405799, 2.12291318, 2.10052003, 2.01530259};
        REQUIRE(validate(node, t, y));
    }
    SECTION("t*atan2(-1, -t)") {
        node = uetsf.makeShared("t*atan2(-1, -t)");
        y = {0.99668652, 0.96811118, 0.95827973, 0.81159657, 0.42263966, 0, -0.64550184, -2.64415535, -7.58685229, -8.89648975, -30.41924001};
        REQUIRE(validate(node, t, y));
    }
    SECTION("t+(-(t-1)) == t+(-(-1+t))") {
        node = uetsf.makeShared("t+(-(t-1))");
        node2 = uetsf.makeShared("t+(-(-1+t))");
        REQUIRE(validate(node, node2, t));
    }
    SECTION("-(t-1)*(t^2-2*t)/10^3") {
        node = uetsf.makeShared("-(t-1)*(t^2-2*t)/10^3");
        y = {1.32, 0.066817944, 0.047758848, 0.007161, 0.001066104, 0, -0.000372504, 0.000099, -0.003368448, -0.007660344, -0.72};
        REQUIRE(validate(node, t, y));
    }
    SECTION("-(-1)*(t^2-2*t)/10^3") {
        node = uetsf.makeShared("-(-1)*(t^2-2*t)/10^3");
        y = {0.12, 0.0161396, 0.0128384, 0.00341, 0.0007956, 0, -0.0005644, -0.00099, 0.0019584, 0.0035796, 0.08};
        REQUIRE(validate(node, t, y));
    }
    SECTION("(1000+((1-(1+sin(1/2+3*pi/3*t))*(2+t))*abs(t)^(2-1/2)))^(3/2)") {
        node = uetsf.makeShared("(1000+((1-(1+sin(1/2+3*pi/3*t))*(2+t))*abs(t)^(2-1/2)))^(3/2)");
        y = {52714.18945957, 32171.06013344, 31838.59857288, 31637.35056204, 31624.96953355, 31622.77660168, 31588.17650985, 31631.21333073, 30466.17238908, 31625.49824345, 10196.5206164};
        REQUIRE(validate(node, t, y));
    }
    SECTION("cos(3*t+sin(1+t)-5*(10+tan(t))^(2*sqrt(abs(t))))") {
        node = uetsf.makeShared("cos(3*t+sin(1+t)-5*(10+tan(t))^(2*sqrt(abs(t))))");
        y = {0.91282919, 0.97082196, 0.90930816, -0.9976885, -0.01534952, -0.52597404, 0.49827694, -0.99743291, 0.3236059, 0.7942002, -0.98567806};
        REQUIRE(validate(node, t, y));
    }
    SECTION("cos(3*t+atan2(t,1)-5*atan2(1/1+1-1,t/t*t)^(2*sqrt(abs(t))))") {
        node = uetsf.makeShared("cos(3*t+atan2(t,1)-5*atan2(1/1+1-1,t/t*t)^(2*sqrt(abs(t))))");
        y = {-0.45058035, -0.83062706, 0.94696144, -0.49875059, 0.79075789, 0.28366219, 0.37464262, 0.0799824, -0.9787685, -0.38051728, 0.99847693};
        REQUIRE(validate(node, t, y));
    }
    SECTION("(t+10)^2-(t+10)^3") {
        node = uetsf.makeShared("(t+10)^2-(t+10)^3");
        y = {0, -275.769256, -332.829952, -625.759, -808.113096, -900, -998.591704, -1244.421, -1896.277248, -2096.087544, -7600};
        REQUIRE(validate(node, t, y));
        REQUIRE(validatePowIPow<double>(node, 0, 2));
    }
    SECTION("(t+10)^2.0000001") {
        node = uetsf.makeShared("(t+10)^2.0000001");
        y = {0, 47.05960906, 52.99841052, 79.21001732, 93.31562116, 100.00002303, 106.91562498, 123.21002966, 161.79844115, 172.65964447, 400.00011983};
        REQUIRE(validate(node, t, y));
        REQUIRE(validatePowIPow<double>(node, 1, 0));
    }
    SECTION("1000*((t+10)^2.0000001-(t+10)^2)") {
        node = uetsf.makeShared("1000*((t+10)^2.0000001-(t+10)^2)");
        y = {0, 0.0090623, 0.01052088, 0.01731571, 0.02116392, 0.02302585, 0.0249757, 0.02965597, 0.04114818, 0.04447127, 0.11982931};
        REQUIRE(validate(node, t, y));
        REQUIRE(validatePowIPow<double>(node, 1, 1));
    }
    SECTION("5") {
        node = uetsf.makeShared("5");
        y = std::vector<double>(t.size(), 5);
        REQUIRE(validate(node, t, y));
    }
}

TEST_CASE("Register Univariate Expression Tree Tests") {
    double ER0, ER1, ER2, ER3, ER4, ER5, ER6, ER7; // The registers
    std::vector<double const*> registers{&ER0, &ER1, &ER2, &ER3, &ER4, &ER5, &ER6, &ER7};
    RegUnivarExprTreeStringFactory<double> ruetsf(registers);
    std::shared_ptr<IExprTreeNode<double, double>> node, node2;
    std::vector<double> t{ // Input domain for all cases
        -10, -3.14, -2.72, -1.1, -0.34, 0, 0.34, 1.1, 2.72, 3.14, 10
    };
    std::vector<double> y; // Expected output

    SECTION("1+(t-1)^2") {
        ER0 = 1; ER1 = 2;
        node = ruetsf.makeShared("ER0+(t-ER0)^ER1");
        y = {122, 18.1396, 14.8384, 5.41, 2.7956, 2, 1.4356, 1.01, 3.9584, 5.5796, 82};
        REQUIRE(validate(node, t, y));
    }
    SECTION("exp(t)+3*cos(1+2*t)") {
        ER0 = 3; ER1 = 1; ER2 = 2;
        node = ruetsf.makeShared("exp(t)+ER0*cos(ER1+ER2*t)");
        y = {2.96615925, 1.65614048, -0.741224555, 1.41994435, 3.55947658, 2.62090692, 1.07798733, 0.00928169656, 18.1435115, 24.7328066, 22024.8226};
        REQUIRE(validate(node, t, y));
    }
    SECTION("ln(t/2)*sin(t)") {
        ER0 = 2;
        node = ruetsf.makeShared("ln(t/ER0)*sin(t)");
        y = {NAN, NAN, NAN, NAN, NAN, NAN, -0.590924735, -0.532796735, 0.125827096, 0.000718406901, -0.875568201};
        REQUIRE(validate(node, t, y));
    }
    SECTION("acos(1/3)+t/pi") {
        ER0 = 1; ER1 = 3;
        node = ruetsf.makeShared("acos(ER0/ER1)+t/pi");
        y = {-1.95213944, 0.23146637, 0.36515653, 0.88081854, 1.12273406, 1.23095942, 1.33918478, 1.58110029, 2.09676231, 2.23045246, 4.41405828};
        REQUIRE(validate(node, t, y));
    }
    SECTION("5*t") {
        ER0 = 5;
        node = ruetsf.makeShared("ER0*t");
        y = {-50, -15.7, -13.6, -5.5, -1.7, 0, 1.7, 5.5, 13.6, 15.7, 50};
        REQUIRE(validate(node, t, y));
    }
    SECTION("4*t^3 + 3*t^2 + 2*t - 1") {
        ER0 = 4; ER1 = 3; ER2 = 2; ER3 = 1;
        node = ruetsf.makeShared("ER0*t^ER1 + ER1*t^ER2 + ER2*t - ER3");
        y = {-3721, -101.537776, -64.739392, -4.894, -1.490416, -1, 0.184016, 10.154, 107.129792, 158.695376, 4319};
        REQUIRE(validate(node, t, y));
    }
    SECTION("3600 - t^(t+1)") {
        ER0 = 3600; ER1 = 1;
        node = ruetsf.makeShared("ER0 - t^(t+ER1)");
        y = {3600, NAN, NAN, NAN, NAN, 3600, 3599.7644, 3598.77841, 3558.6385, 3485.89919, -99999996400.0};
        REQUIRE(validate(node, t, y));
    }
    SECTION("sin(t^2 + 3)") {
        ER0 = 2; ER1 = 3;
        node = ruetsf.makeShared("sin(t^ER0 + ER1)");
        y = {0.62298863, 0.28904527, -0.82692784, -0.87643472, 0.02598973, 0.14112001, 0.02598973, -0.87643472, -0.82692784, 0.28904527, 0.62298863};
        REQUIRE(validate(node, t, y));
    }
    SECTION("3.1 + 5 * 7 + 4.4") {
        ER0 = 3.1; ER1 = 5; ER2 = 7; ER3 = 4.4;
        node = ruetsf.makeShared("ER0 + ER1 * ER2 +ER3");
        y = std::vector<double>(t.size(), 42.5);
        REQUIRE(validate(node, t, y));
    }
    SECTION("-1") {
        ER0 = 1;
        node = ruetsf.makeShared("-ER0");
        y = std::vector<double>(t.size(), -1);
        REQUIRE(validate(node, t, y));
    }
    SECTION("-t*7") {
        ER0 = 7;
        node = ruetsf.makeShared("-t*ER0");
        y = {70, 21.98, 19.04, 7.7, 2.38, 0, -2.38, -7.7, -19.04, -21.98, -70};
        REQUIRE(validate(node, t, y));
    }
    SECTION("-1 +7") {
        ER0 = 1; ER1 = 7;
        node = ruetsf.makeShared("-ER0 +ER1");
        y = std::vector<double>(t.size(), 6);
        REQUIRE(validate(node, t, y));
    }
    SECTION("((t-1)*((1+t) / 2))/(t+0.01)^2") {
        ER0 = 1; ER1 = 2; ER2 = 0.01;
        node = ruetsf.makeShared("((t-ER0)*((ER0+t) / ER1))/(t+ER2)^ER1");
        y = {0.495991487, 0.452163439, 0.435614983, 0.0883763993, -4.06060606, -5000, -3.60979592, 0.0852203555, 0.429255726, 0.446439909, 0.494011483};
        REQUIRE(validate(node, t, y));
    }
    SECTION("(pi/3)^(abs(t)^(e-1))") {
        ER0 = 3; ER1 = 1;
        node = ruetsf.makeShared("(pi/ER0)^(abs(t)^(e-ER1))");
        y = {11.14208743, 1.39014251, 1.29353943, 1.05582653, 1.00725079, 1, 1.00725079, 1.05582653, 1.29353943, 1.39014251, 11.14208743};
        REQUIRE(validate(node, t, y));
    }
    SECTION("atan2(t, 1) +pi") {
        ER0 = 1;
        node = ruetsf.makeShared("atan2(t, ER0) +pi");
        y = {1.67046498, 1.87911199, 1.92310505, 2.30861139, 2.81385415, 3.14159265, 3.46933116, 3.97457392, 4.36008026, 4.40407332, 4.61272033};
        REQUIRE(validate(node, t, y));
    }
    SECTION("t-(-t) == 2*t") {
        ER0 = 2;
        node = ruetsf.makeShared("t-(-t)");
        node2 = ruetsf.makeShared("ER0*t");
        REQUIRE(validate(node, node2, t));
    }
    SECTION("t+(1)") {
        ER0 = 1;
        node = ruetsf.makeShared("t+(ER0)");
        y = {-9, -2.14, -1.72, -0.1, 0.66, 1, 1.34, 2.1, 3.72, 4.14, 11};
        REQUIRE(validate(node, t, y));
    }
    SECTION("(-1)+t") {
        ER0 = 1;
        node = ruetsf.makeShared("(-ER0)+t");
        y = {-11, -4.14, -3.72, -2.1, -1.34, -1, -0.66, 0.1, 1.72, 2.14, 9};
        REQUIRE(validate(node, t, y));
    }
    SECTION("(-1+1)+t") {
        ER0 = 1;
        node = ruetsf.makeShared("(-ER0+ER0)+t");
        REQUIRE(validate(node, t, t));
    }
    SECTION("(1-(-1))") {
        ER0 = 1;
        node = ruetsf.makeShared("(ER0-(-ER0))");
        y = std::vector<double>(t.size(), 2);
        REQUIRE(validate(node, t, y));
    }
    SECTION("(1-(1))") {
        ER0 = 1;
        node = ruetsf.makeShared("(ER0-(ER0))");
        y = std::vector<double>(t.size(), 0);
        REQUIRE(validate(node, t, y));
    }
    SECTION("abs(t) = abs(-t) = abs(1*t)") {
        ER0 = -1;
        node = ruetsf.makeShared("abs(t)");
        node2 = ruetsf.makeShared("abs(-t)");
        REQUIRE(validate(node, node2, t));
        node = ruetsf.makeShared("abs(ER0*t)");
        REQUIRE(validate(node, node2, t));
    }
    SECTION("abs(-t) - abs(-1*t) + abs(t) - abs(-2*t+(t))") {
        ER0 = 1; ER1 = 2;
        node = ruetsf.makeShared("abs(-t) - abs(-ER0*t) + abs(t) - abs(-ER1*t+(t))");
        y = std::vector<double>(t.size(), 0);
        REQUIRE(validate(node, t, y));
    }
    SECTION("1 + atan2(3*t-1, 1)") {
        ER0 = 1; ER1 = 3;
        node = ruetsf.makeShared("ER0 + atan2(ER1*t-ER0, ER0)");
        y = {-0.53854944, -0.47512005, -0.46205665, -0.34229969, -0.11111695, 0.21460184, 1.01999733, 2.16066899, 2.43202915, 2.4525852, 2.53632723};
        REQUIRE(validate(node, t, y));
    }
    SECTION("1+atan2(3*t-1, (t-1)*2)") {
        ER0 = 1; ER1 = 3; ER2 = 2;
        node = ruetsf.makeShared("ER0+atan2(ER1*t-ER0, (t-ER0)*ER2)");
        y = {-1.18798772, -1.24225167, -1.25294846, -1.34443033, -1.49570776, -1.67794504, 4.1264423, 2.48405799, 2.12291318, 2.10052003, 2.01530259};
        REQUIRE(validate(node, t, y));
    }
    SECTION("t*atan2(-1, -t)") {
        ER0 = 1;
        node = ruetsf.makeShared("t*atan2(-ER0, -t)");
        y = {0.99668652, 0.96811118, 0.95827973, 0.81159657, 0.42263966, 0, -0.64550184, -2.64415535, -7.58685229, -8.89648975, -30.41924001};
        REQUIRE(validate(node, t, y));
    }
    SECTION("t+(-(t-1)) == t+(-(-1+t))") {
        ER0 = 1;
        node = ruetsf.makeShared("t+(-(t-ER0))");
        node2 = ruetsf.makeShared("t+(-(-ER0+t))");
        REQUIRE(validate(node, node2, t));
    }
    SECTION("-(t-1)*(t^2-2*t)/10^3") {
        ER0 = 1; ER1 = 2; ER2 = 10; ER3 = 3;
        node = ruetsf.makeShared("-(t-ER0)*(t^ER1-ER1*t)/ER2^ER3");
        y = {1.32, 0.066817944, 0.047758848, 0.007161, 0.001066104, 0, -0.000372504, 0.000099, -0.003368448, -0.007660344, -0.72};
        REQUIRE(validate(node, t, y));
    }
    SECTION("-(-1)*(t^2-2*t)/10^3") {
        ER0 = 1; ER1 = 2; ER2 = 10; ER3 = 3;
        node = ruetsf.makeShared("-(-ER0)*(t^ER1-ER1*t)/ER2^ER3");
        y = {0.12, 0.0161396, 0.0128384, 0.00341, 0.0007956, 0, -0.0005644, -0.00099, 0.0019584, 0.0035796, 0.08};
        REQUIRE(validate(node, t, y));
    }
    SECTION("(1000+((1-(1+sin(1/2+3*pi/3*t))*(2+t))*abs(t)^(2-1/2)))^(3/2)") {
        ER0 = 1000; ER1 = 1; ER2 = 2; ER3 = 3;
        node = ruetsf.makeShared("(ER0+((ER1-(ER1+sin(ER1/ER2+ER3*pi/ER3*t))*(ER2+t))*abs(t)^(ER2-ER1/ER2)))^(ER3/ER2)");
        y = {52714.18945957, 32171.06013344, 31838.59857288, 31637.35056204, 31624.96953355, 31622.77660168, 31588.17650985, 31631.21333073, 30466.17238908, 31625.49824345, 10196.5206164};
        REQUIRE(validate(node, t, y));
    }
    SECTION("cos(3*t+sin(1+t)-5*(10+tan(t))^(2*sqrt(abs(t))))") {
        ER0 = 3; ER1 = 1; ER2 = 5; ER3 = 10; ER4 = 2;
        node = ruetsf.makeShared("cos(ER0*t+sin(ER1+t)-ER2*(ER3+tan(t))^(ER4*sqrt(abs(t))))");
        y = {0.91282919, 0.97082196, 0.90930816, -0.9976885, -0.01534952, -0.52597404, 0.49827694, -0.99743291, 0.3236059, 0.7942002, -0.98567806};
        REQUIRE(validate(node, t, y));
    }
    SECTION("cos(3*t+atan2(t,1)-5*atan2(1/1+1-1,t/t*t)^(2*sqrt(abs(t))))") {
        ER0 = 3; ER1 = 1; ER2 = 5; ER3 = 2;
        node = ruetsf.makeShared("cos(ER0*t+atan2(t,ER1)-ER2*atan2(ER1/ER1+ER1-ER1,t/t*t)^(ER3*sqrt(abs(t))))");
        y = {-0.45058035, -0.83062706, 0.94696144, -0.49875059, 0.79075789, 0.28366219, 0.37464262, 0.0799824, -0.9787685, -0.38051728, 0.99847693};
        REQUIRE(validate(node, t, y));
    }
    SECTION("(t+10)^2-(t+10)^3") {
        ER0 = 10; ER1 = 2; ER2 = 3;
        node = ruetsf.makeShared("(t+ER0)^ER1-(t+ER0)^ER2");
        y = {0, -275.769256, -332.829952, -625.759, -808.113096, -900, -998.591704, -1244.421, -1896.277248, -2096.087544, -7600};
        REQUIRE(validate(node, t, y));
        // No validate IPow here because ER cannot be assumed as integer for IPow
    }
    SECTION("(t+10)^2.0000001") {
        ER0 = 10; ER1 = 2.0000001;
        node = ruetsf.makeShared("(t+ER0)^ER1");
        y = {0, 47.05960906, 52.99841052, 79.21001732, 93.31562116, 100.00002303, 106.91562498, 123.21002966, 161.79844115, 172.65964447, 400.00011983};
        REQUIRE(validate(node, t, y));
        REQUIRE(validatePowIPow<double>(node, 1, 0));
    }
    SECTION("1000*((t+10)^2.0000001-(t+10)^2)") {
        ER0 = 1000; ER1 = 10; ER2 = 2.0000001;
        node = ruetsf.makeShared("ER0*((t+ER1)^ER2-(t+ER1)^2)");
        y = {0, 0.0090623, 0.01052088, 0.01731571, 0.02116392, 0.02302585, 0.0249757, 0.02965597, 0.04114818, 0.04447127, 0.11982931};
        REQUIRE(validate(node, t, y));
        REQUIRE(validatePowIPow<double>(node, 1, 1));
    }
    SECTION("5") {
        ER0 = 5;
        node = ruetsf.makeShared("ER0");
        y = std::vector<double>(t.size(), 5);
        REQUIRE(validate(node, t, y));
    }
}