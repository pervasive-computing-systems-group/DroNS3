import numpy as np
import pandas as pd
from sklearn.linear_model import RANSACRegressor
from scipy.optimize import minimize

# Step 1: Load the data from CSV
data = pd.read_csv('field_data_pi3.csv', sep='\t')
# data = pd.read_csv('field_data_pi4.csv', sep='\t')
x = data['distance'].values
y = data['rate'].values

# Good start for Pi3  a = 680.9115284926437, c = 110.18833920073857, b = -0.24631283412113067, r_max= 5.374491790439996
INIT_GUESS = [700, 100, 0.0, 4.5]
# Good start for Pi4  a = 2403.429060855952, c = 307.705527108798, b = -0.6544512891068248, r_max= 4.904492221598213
# INIT_GUESS = [3000, 300, -0.5, 5.0]

USE_RANSAC = False
USE_GS_L1 = True


# Step 2: Define the model function: y = min(a/x^2 + b, MAX_R)
def model(params, x):
    a, b, c, r_m = params
    return np.minimum(a / (x**2 + b) + c, r_m)


# Step 3: Define the error function for optimization (squared residuals)
def residuals(params, x, y):
    y_pred = model(params, x)
    return np.sum((y - y_pred)**2)


## Using L1 norm:
# Step 3: Define the error function (L1 norm, absolute residuals)
def residuals_l1(params, x, y):
    y_pred = model(params, x)
    return np.sum(np.abs(y - y_pred))  # L1 norm (sum of absolute differences)


# Step 4: Define the RANSAC model with a custom base estimator
class CustomModelRANSAC:
    def __init__(self):
        self.params_ = None

    def fit(self, x, y):
        # Step 5: Use optimization (minimize) to fit the model
        initial_guess = INIT_GUESS  # Initial guesses for 'a' and 'b'
        result = minimize(residuals, initial_guess, args=(x.ravel(), y))
        self.params_ = result.x

    def predict(self, x):
        if self.params_ is None:
            raise ValueError("The model has not been fit yet.")
        return model(self.params_, x.ravel())

    def score(self, x, y):
        """
        Score function, to be used in RANSAC.
        Computes the negative mean squared error.
        """
        y_pred = self.predict(x)
        return -np.mean((y - y_pred)**2)  # Negative MSE (larger is better for RANSAC)

    # Implement get_params and set_params for scikit-learn compatibility
    def get_params(self, deep=False):
        # Return any parameters that might affect the fit
        return {}

    def set_params(self, **params):
        # Set any parameters (in this case, none are needed)
        return self


# Use optimization (minimize) to fit the model using L1 norm
initial_guess = INIT_GUESS  # Initial guesses for 'a' and 'b'
result = minimize(residuals_l1, initial_guess, args=(x, y), method='SLSQP')

# Extract the optimized parameters
a_opt, b_opt, c_opt, rm_opt = result.x
print(f"Sequential Least Squares Programming: a = {a_opt}, b = {b_opt}, c = {c_opt}, r_max = {rm_opt}")

# Step 5: Predict and plot results
import matplotlib.pyplot as plt

# Plot original data
plt.scatter(x, y, label='Data')

# Plot the fitted model
x_plot = np.linspace(min(x), max(x), 100)
y_plot = model([a_opt, b_opt, c_opt, rm_opt], x_plot)
plt.plot(x_plot, y_plot, color='red', label='Fitted Model (L1 norm)')

plt.legend()
plt.xlabel('x')
plt.ylabel('y')
plt.show()


# Apply RANSAC
ransac = RANSACRegressor(estimator=CustomModelRANSAC(), min_samples=20, residual_threshold=0.5, max_trials=500)

# Reshape data for fitting (since RANSAC expects 2D input for x)
x = x.reshape(-1, 1)

# Fit the RANSAC model
ransac.fit(x, y)

# Get the inliers and outliers
inlier_mask = ransac.inlier_mask_
outlier_mask = np.logical_not(inlier_mask)

# Get the best parameters for 'a' and 'b'
best_model = ransac.estimator_
a_opt, b_opt, c_opt, rm_opt = best_model.params_
print(f"RANSAC: a = {a_opt}, b = {b_opt}, c = {c_opt}, r_max= {rm_opt}")

# Optional - Plotting the results
import matplotlib.pyplot as plt

# Plot inliers and outliers
plt.scatter(x[inlier_mask], y[inlier_mask], color='yellowgreen', label='Inliers')
plt.scatter(x[outlier_mask], y[outlier_mask], color='red', label='Outliers')

# Plot the fitted model
x_plot = np.linspace(min(x), max(x), 100).reshape(-1, 1)
y_plot = best_model.predict(x_plot)
plt.plot(x_plot, y_plot, color='blue', label='Fitted Model')

plt.legend()
plt.xlabel('x')
plt.ylabel('y')
plt.show()


