import numpy as np
import scipy.stats as stats
import matplotlib.pyplot as plt

debug = True


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
		ax.set_xlim([0, 0.25])
		ax.set_ylim([0, None])
		plt.show()
	
	return [k,lambda_]


# # Data samples from field testing
# data_3m = np.array([0.19,0.23,0.23,0.23,0.22,0.23,0.23,0.23,0.2,0.22,0.22,0.22,0.22,0.2,0.22,0.21,0.22,0.21,0.22,0.21,0.11,0.23,0.23,0.12])
# fit_weibull(data_3m, "3 m")
# data_10m = np.array([0.22,0.08,0.17,0.22,0.2,0.2,0.18,0.13,0.09,0.08,0.18,0.09,0.13,0.06,0.07,0.05])
# fit_weibull(data_10m, "10 m")
# data_15m = np.array([0,0,0.02,0.09,0.01,0.17,0,0,0.02,0.01,0.2,0.21,0.22,0.22])
# fit_weibull(data_15m, "15 m")
# data_22m = np.array([0,0,0,0,0.01,0.03,0.02,0.09,0.17,0.21,0.22,0.21,0.01,0.01,0.05,0.02,0.03,0,0,0.02,0.08,0.15,0.16,0.11,0.13])
# fit_weibull(data_22m, "22 m")
# # Ugly data:
# data_35m = np.array([0.16,0.2,0.2,0,0,0.01,0.07,0.15,0.13,0.13,0.06,0.15,0.02,0.21,0.04,0.19,0.18,0.13,0.09,0,0,0.01,0.03,0.2,0.21,0.19,0.17,0.06,0.12,0.16,0.1,0,0,0,0,0.01,0.03,0.02,0.07,0.18,0.16,0.12,0.12,0.01,0.05,0.09,0.17,0.12,0.08,0.04,0.11,0.08,0.14,0.06])
# fit_weibull(data_35m, "35 m")
# # Mostly ugly data:
# data_40m = np.array([0,0,0,0,0.01,0.03,0.02,0.07,0.01,0.05,0.09,0.17,0.12,0.08,0.04,0.06])
# fit_weibull(data_40m, "40 m")
# data_50m = np.array([0,0,0.03,0.08,0.08,0.01,0.01,0.03,0.01,0.02,0.04,0.05,0,0,0,0,0,0,0.01,0.01,0.03,0.02,0.01,0.03,0,0.04,0.01,0.14,0.02,0.05,0.02,0.12,0,0,0.02,0.06,0.19,0.21,0.02,0.19,0.14,0.14,0.16,0.08])
# fit_weibull(data_50m, "50 m")
# data_80m = np.array([0,0,0,0,0.02,0,0.05,0.02,0.19,0.18,0.12,0.1,0,0,0,0,0,0,0,0,0.01,0,0.01,0.01,0,0,0.01,0.07,0.03,0.04,0.01,0.06,0,0,0,0,0,0,0.01,0.01,0,0.09,0.05,0.02,0,0,0,0,0,0,0.02,0,0,0.22,0.16,0.06])
# fit_weibull(data_50m, "65 - 80 m")




# Step 1: Read data from the CSV file
# Assuming the CSV file has two columns: 'a' and 'y'
data = np.genfromtxt('field_data.csv', delimiter='\t', skip_header=1)

# Step 2: Sort the data by the 'a' values (x-values)
sorted_data = data[data[:, 0].argsort()]

# Step 3: Process the data in chunks of 25 points at a time
chunk_size = 20


def process_chunk(chunk, prev_k = 20):
	"""
	This function will receive a 25-point chunk of data
	and will perform some data science task with both 'a' and 'y' values.
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


# Step 4: Loop over the data in spans of chunk_size points
# result = [None,None,None,None]
for i in range(0, len(sorted_data), int(chunk_size/2)):
	chunk = sorted_data[i:i + chunk_size]
	process_chunk(chunk)
