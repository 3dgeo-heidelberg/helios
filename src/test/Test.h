#include <iostream>
#include <vector>
#include <random>

#include "MarquardtFitter.h"

using namespace std;

class Test {

public:
	void testRandom() {

		//std::srand(std::time(0));

		std::random_device rd;  //Will be used to obtain a seed for the random number engine
		std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
		std::uniform_real_distribution<> dis(0.0, 1.0);

		for (int i = 0; i < 100; i++) {			
			//cout << std::rand() / RAND_MAX << endl;
			//cout << std::rand() << endl;
			cout << dis(gen) << endl;
		}
	}

	void run() {
		std::vector<double> fullwave(200, 0.0); //TODO try std::array
		fullwave[71] = 0.0;
		fullwave[72] = 304.6258290320149;
		fullwave[73] = 1136.1249614927067;
		fullwave[74] = 2383.4607586575366;
		fullwave[75] = 3950.7983668633424;
		fullwave[76] = 5755.781220246707;
		fullwave[77] = 7727.982967423797;
		fullwave[78] = 9807.518177751515;
		fullwave[79] = 11943.796685780588;
		fullwave[80] = 14094.407804669252;
		fullwave[81] = 16224.12189173799;
		fullwave[82] = 18303.997892161336;
		fullwave[83] = 20310.58652932293;
		fullwave[84] = 22225.219761141412;
		fullwave[85] = 24033.377988536104;
		fullwave[86] = 25724.12729231791;
		fullwave[87] = 27289.619694741228;
		fullwave[88] = 28724.650097763853;
		fullwave[89] = 30026.26414725786;
		fullwave[90] = 31193.411816054955;
		fullwave[91] = 32226.641993430272;
		fullwave[92] = 33127.83381867297;
		fullwave[93] = 33899.96090564749;
		fullwave[94] = 34546.88497727351;
		fullwave[95] = 35073.17576690247;
		fullwave[96] = 35483.95435062383;
		fullwave[97] = 35784.757353321904;
		fullwave[98] = 35981.419724318;
		fullwave[99] = 36079.97400795409;
		fullwave[100] = 36086.56424258883;
		fullwave[101] = 36007.37281009028;
		fullwave[102] = 35848.55872876636;
		fullwave[103] = 35616.20603736408;
		fullwave[104] = 35316.28105774768;
		fullwave[105] = 34954.59745046197;
		fullwave[106] = 34536.78809181308;
		fullwave[107] = 34068.28290446155;
		fullwave[108] = 33554.291866833046;
		fullwave[109] = 32999.79251083113;
		fullwave[110] = 32409.521293225163;
		fullwave[111] = 31787.968294451945;
		fullwave[112] = 31139.37476010957;
		fullwave[113] = 30467.733055774166;
		fullwave[114] = 29776.78865551364;
		fullwave[115] = 29070.04382913681;
		fullwave[116] = 28350.76273328072;
		fullwave[117] = 27621.97764734261;
		fullwave[118] = 26886.496127403192;
		fullwave[119] = 26146.90888002677;
		fullwave[120] = 25405.598183491144;
		fullwave[121] = 24664.746706893926;
		fullwave[122] = 23926.34659797524;
		fullwave[123] = 23192.208728634778;
		fullwave[124] = 22463.97200322889;
		fullwave[125] = 21743.112649013565;
		fullwave[126] = 21030.953420735183;
		fullwave[127] = 20328.67266253104;
		fullwave[128] = 19637.313180135883;
		fullwave[129] = 18957.790885035913;
		fullwave[130] = 18290.90317979298;
		fullwave[131] = 17637.337060388763;
		fullwave[132] = 16997.676917215533;
		fullwave[133] = 16372.412021354836;
		fullwave[134] = 15761.94368712403;
		fullwave[135] = 15166.592105604374;
		fullwave[136] = 14586.602847062239;
		fullwave[137] = 14022.153032896964;
		fullwave[138] = 13473.35718004877;
		fullwave[139] = 12940.27272272731;
		fullwave[140] = 12422.90521791965;
		fullwave[141] = 11921.213242445208;
		fullwave[142] = 11435.112990379346;
		fullwave[143] = 10964.482580499178;
		fullwave[144] = 10509.166084042365;
		fullwave[145] = 10068.977283538303;
		fullwave[146] = 9643.703173792512;
		fullwave[147] = 9233.107216300932;
		fullwave[148] = 8836.932358457492;
		fullwave[149] = 8454.903828912795;
		fullwave[150] = 8086.731720357846;
		fullwave[151] = 7732.113370856265;
		fullwave[152] = 7390.735554643773;
		fullwave[153] = 7062.276493063163;
		fullwave[154] = 6746.407696015988;
		fullwave[155] = 6442.795643996111;
		fullwave[156] = 6151.103320431239;
		fullwave[157] = 5870.991603703011;
		fullwave[158] = 5602.120527848272;
		fullwave[159] = 5344.15042056905;
		fullwave[160] = 5096.742926799163;
		fullwave[161] = 4859.5619256956015;
		fullwave[162] = 4632.274348544305;
		fullwave[163] = 4414.550904696215;
		fullwave[164] = 4206.066722281406;
		fullwave[165] = 4006.5019100888726;
		fullwave[166] = 3815.5420466483015;
		fullwave[167] = 3632.8786022088548;
		fullwave[168] = 3458.209298979449;
		fullwave[169] = 3291.2384146760082;
		fullwave[170] = 3131.677034114027;

		std::vector<std::vector<double>> apMatrix;
		MarquardtFitter fit = MarquardtFitter(apMatrix);
		fit.setData(fullwave);
		int i = 100;
		fit.setParameters({ 0, fullwave[i], (double)i, 1 });
		fit.fitData();
		std::vector<double> result = fit.getParameters();
		for (double d : result) {
			std::cout << d << std::endl;
		}
	}
};