import numpy as np
import scipy.stats as stats
import matplotlib.pyplot as plt

debug = False


# Fit Weibull distribution
def fit_weibull(data, range='', fix_f0 = False):
	if fix_f0:
		shape, loc, scale = stats.weibull_min.fit(data, floc=0, f0=1)
	else:
		shape, loc, scale = stats.weibull_min.fit(data, floc=0)
	
	# k and lambda estimates
	k = shape
	lambda_ = scale

	if debug:
		print(f"Estimated shape (k): {k}")
		print(f"Estimated scale (lambda): {lambda_}")
		# Optional: Plot the histogram and fitted PDF
		x = np.linspace(min(data), max(data), 100)
		pdf_fitted = stats.weibull_min.pdf(x, shape, loc, scale)
		plt.hist(data, density=True, bins=30, alpha=0.5, label="Data")
		plt.plot(x, pdf_fitted, label=f"Fitted Weibull PDF - {range}", color='red')
		plt.legend()
		ax = plt.gca()
		ax.set_xlim([0, 6.1])
		ax.set_ylim([0, None])
		plt.show()
	
	return [k,lambda_]


def process_chunk(chunk, prev_k = 20):
	"""
	This function take in a chunk of data and fits a weibull distribution to
	the data (as well as some other stuff..)
	"""
	# Example: Calculate the mean of 'a' and 'y' in the chunk
	mean_a = np.mean(chunk[:, 0])
	median_y = np.median(chunk[:, 1])
	# You can replace this with any task you want to perform on the chunk
	if prev_k == 1:
		result = fit_weibull(np.array(chunk[:, 1]), f'{mean_a}', True)
	else:
		result = fit_weibull(np.array(chunk[:, 1]), f'{mean_a}')
	result.append(mean_a)
	result.append(median_y)
	if debug:
		print(f"Mean of a (x-values): {mean_a}, Median of y: {median_y}")
	else:
		print(f"{result[2]} {result[0]} {result[1]} {result[3]}")
	return result


# Read data from the CSV file
data = np.genfromtxt('field_data_pi4.csv', delimiter='\t', skip_header=1)

# Sort the data points
sorted_data = data[data[:, 0].argsort()]

# Group data by the unique values in the first column ('a' values)
unique_values = np.unique(sorted_data[:, 0])

# Process each group of data points that share the same 'a' value
for value in unique_values:
	group = data[data[:, 0] == value]  # Select rows where the first column equals the current value
	process_chunk(group)
