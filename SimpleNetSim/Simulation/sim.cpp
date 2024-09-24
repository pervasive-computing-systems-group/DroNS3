// weibull_distribution
#include <iostream>
#include <random>
#include <vector>
#include <chrono>
#include <thread>
#include <boost/numeric/conversion/cast.hpp>


static const double distance_data[] = {3.47472,10.24128,12.3444,20.1168,47.3202,75.2094,90.9828};
static const double k_data[] = {26.00845609,1.739347261,1,1,1,1,1};
static const double lambda_data[] = {0.2229104192,0.1129530764,0.08491171886,0.07503332593,0.06666895829,0.05762811814,0.04842520005};

int main(int argc, char *argv[]) {
	// Vectors with field data
	int data_count = boost::numeric_cast<int>(sizeof(distance_data)/sizeof(distance_data[0]));
	std::vector<double> distance_vctr(distance_data, distance_data + data_count);
	std::vector<double> k_vctr(k_data, k_data + data_count);
	std::vector<double> lambda_vctr(lambda_data, lambda_data + data_count);

	double distance = 10.0;
	int bytes = 1000000;
	double k, lambda;

	// Verify user input
	if(argc != 3) {
		fprintf(stderr, "Received %d args, expected 2.\nExpected use:\t./sim <distance> <bytes-sent>\n\n", (argc-1));
		exit(1);
	}
	else {
		// Extract arguments
		distance = atof(argv[1]);
		bytes = atoi(argv[2]);
	}

	// Are we at the extremes?
	if(distance < distance_vctr.at(0)) {
		// Just use start values
		k = k_vctr.at(0);
		lambda = lambda_vctr.at(0);
	}
	else if(distance > distance_vctr.back()) {
		// Just use last values
		k = k_vctr.back();
		lambda = lambda_vctr.back();
	}
	else {
		// Determine which range we are in
		double nxt_dist = distance_vctr.at(0);
		double nxt_k = k_vctr.at(0);
		double nxt_lambda = lambda_vctr.at(0);
		double prev_dist = nxt_dist;
		double prev_k = nxt_k;
		double prev_lambda = nxt_lambda;
		for(int i = 1; i < data_count; i++) {
			nxt_dist = distance_vctr.at(i);
			nxt_k = k_vctr.at(i);
			nxt_lambda = lambda_vctr.at(i);
			if(distance < distance_vctr.at(i)) {
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
