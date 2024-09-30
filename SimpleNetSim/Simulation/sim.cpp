// weibull_distribution
#include <iostream>
#include <random>
#include <vector>
#include <chrono>
#include <thread>
#include <boost/numeric/conversion/cast.hpp>


static const double distance_data_0[] = {2.0,5.0,10.0,15.0,20.0,30.0};
static const double k_data_0[] = {16.566151253635788, 17.97621241198601, 6.3858853724124085, 3.7043053126055225, 4.8078989387905935, 2.24308914153775};
static const double lambda_data_0[] = {5.543630764883076, 4.8978866265994885, 3.0472871099258754, 2.199103363551945 , 1.2793297962798627, 0.43230539218152975};

static const double distance_data_1[] = {2.0,5.0,10.0,15.0,20.0,30.0,40.0};
static const double k_data_1[] = {8.055053731672167, 13.095538470432244, 17.361852245076214, 6.196791524528894, 4.909501855491459, 5.230407299614457, 2.8225714889111297};
static const double lambda_data_1[] = {5.192666699797578, 5.038390674565392, 4.858697263620757, 3.9344934766315793, 2.7642056298343194, 1.6213144364702052, 0.46840007312263227};

int main(int argc, char *argv[]) {
	// Vectors with field data
	std::vector<std::vector<double>> distance_vctr;
	std::vector<std::vector<double>> k_vctr;
	std::vector<std::vector<double>> lambda_vctr;
	std::vector<int> counts;

	// Fill vectors with node types
	{
		int data_count0 = boost::numeric_cast<int>(sizeof(distance_data_0)/sizeof(distance_data_0[0]));

		std::vector<double> distance_0(distance_data_0, distance_data_0 + data_count0);
		distance_vctr.push_back(distance_0);
		std::vector<double> k_0(k_data_0, k_data_0 + data_count0);
		k_vctr.push_back(k_0);
		std::vector<double> lambda_0(lambda_data_0, lambda_data_0 + data_count0);
		lambda_vctr.push_back(lambda_0);
		counts.push_back(data_count0);

		int data_count1 = boost::numeric_cast<int>(sizeof(distance_data_1)/sizeof(distance_data_1[0]));

		std::vector<double> distance_1(distance_data_1, distance_data_1 + data_count1);
		distance_vctr.push_back(distance_1);
		std::vector<double> k_1(k_data_1, k_data_1 + data_count1);
		k_vctr.push_back(k_1);
		std::vector<double> lambda_1(lambda_data_1, lambda_data_1 + data_count1);
		lambda_vctr.push_back(lambda_1);
		counts.push_back(data_count1);
	}

	double distance = 10.0;
	int bytes = 1000000;
	int node_type = 1;
	double k, lambda;

	// Verify user input
	if(argc == 3) {
		// Extract 2 arguments
		distance = atof(argv[1]);
		bytes = atoi(argv[2]);
	}
	else if(argc == 4) {
		// Extract 3 arguments
		distance = atof(argv[1]);
		bytes = atoi(argv[2]);
		node_type = atoi(argv[3]);
	}
	else {
		fprintf(stderr, "Received %d args, expected 2.\nExpected use:\t./sim <distance> <bytes-sent> [node-type]\n\n", (argc-1));
		exit(1);
	}

	// Are we at the extremes?
	if(distance < distance_vctr.at(node_type).at(0)) {
		// Just use start values
		k = k_vctr.at(node_type).at(0);
		lambda = lambda_vctr.at(node_type).at(0);
	}
	else if(distance > distance_vctr.at(node_type).back()) {
		// Just use last values
		k = k_vctr.at(node_type).back();
		lambda = lambda_vctr.at(node_type).back();
	}
	else {
		// Determine which range we are in
		double nxt_dist = distance_vctr.at(node_type).at(0);
		double nxt_k = k_vctr.at(node_type).at(0);
		double nxt_lambda = lambda_vctr.at(node_type).at(0);
		double prev_dist = nxt_dist;
		double prev_k = nxt_k;
		double prev_lambda = nxt_lambda;
		for(int i = 1; i < counts.at(node_type); i++) {
			nxt_dist = distance_vctr.at(node_type).at(i);
			nxt_k = k_vctr.at(node_type).at(i);
			nxt_lambda = lambda_vctr.at(node_type).at(i);
			if(distance < distance_vctr.at(node_type).at(i)) {
				break;
			}
			else {
				prev_dist = nxt_dist;
				prev_k = nxt_k;
				prev_lambda = nxt_lambda;
			}
		}

		// Determine line equation between previous and next measurements (k)
		double m_k = (nxt_k-prev_k)/(nxt_dist-prev_dist);
		k = m_k*(distance - prev_dist) + prev_k;
		// Line equation for lambda
		double m_lambda = (nxt_lambda-prev_lambda)/(nxt_dist-prev_dist);
		lambda = m_lambda*(distance - prev_dist) + prev_lambda;
	}

	// Create Weibull distribution
	std::weibull_distribution<double> distribution(k,lambda);
	unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
	std::default_random_engine generator (seed);


	const int nrolls=10000;  // number of experiments
	const int nstars=100;    // maximum number of stars to distribute

	int p[10]={};
	int failure = 0;
	int max_rate = 0;

	for(int i=0; i<nrolls; ++i) {
		double number = distribution(generator);
		if(number < 0) {
			failure++;
		}
		else if(number > 0.3) {
			max_rate++;
		}
		else {
			int index = int(number*(10/0.3));
			++p[int(index)];
		}
	}

	printf("weibull_distribution (%f,%f):\n", k, lambda);

	for(int i=0; i<10; ++i) {
		std::cout << i*(0.3/10) << "-" << (i+1)*(0.3/10) << ": \t";
		std::cout << std::string(p[i]*nstars/nrolls,'*') << std::endl;
	}

	double tx_rate = distribution(generator);

	// Ensure that we can actually talk to the node...
	if(tx_rate >= 0.05) {
		double time = (bytes/1000000.0)/tx_rate;

		printf("TX-rate = %f\nTime to transfer data: %f s\n", tx_rate,time);
		std::this_thread::sleep_for(std::chrono::milliseconds(int(time*1000)));
	}
	else {
		// We weren't so lucky...
		printf("TX-rate too low: %f\n", tx_rate);
		return 1;
	}

	return 0;
}
