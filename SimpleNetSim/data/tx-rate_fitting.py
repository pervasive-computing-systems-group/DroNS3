import numpy as np
import pandas as pd
from sklearn.linear_model import RANSACRegressor
from scipy.optimize import minimize

# Step 1: Load the data from CSV
data = pd.read_csv('field_data.csv', sep='\t')  # Replace with your actual file path
x = data['distance'].values
y = data['rate'].values

# Step 2: Define the model function: y = min(a/x^2 + b, 0.23)
def model(params, x):
    a, b = params
    return np.minimum(a / x**2 + b, 0.23)

# Step 3: Define the error function for optimization (squared residuals)
def residuals(params, x, y):
    y_pred = model(params, x)
    return np.sum((y - y_pred)**2)

# Step 4: Define the RANSAC model with a custom base estimator
class CustomModelRANSAC:
    def __init__(self):
        self.params_ = None

    def fit(self, x, y):
        # Step 5: Use optimization (minimize) to fit the model
        initial_guess = [100, 0.01]  # Initial guesses for 'a' and 'b'
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

# Step 6: Apply RANSAC
ransac = RANSACRegressor(estimator=CustomModelRANSAC(), min_samples=20, residual_threshold=0.05, max_trials=100)

# Step 7: Reshape data for fitting (since RANSAC expects 2D input for x)
x = x.reshape(-1, 1)

# Fit the RANSAC model
ransac.fit(x, y)

# Get the inliers and outliers
inlier_mask = ransac.inlier_mask_
outlier_mask = np.logical_not(inlier_mask)

# Get the best parameters for 'a' and 'b'
best_model = ransac.estimator_
a, b = best_model.params_
print(f"Estimated values of a: {a}, b: {b}")

# Step 8: Optional - Plotting the results
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


## Using L1 norm:
# Step 3: Define the error function (L1 norm, absolute residuals)
def residuals_l1(params, x, y):
    y_pred = model(params, x)
    return np.sum(np.abs(y - y_pred))  # L1 norm (sum of absolute differences)


# Step 4: Use optimization (minimize) to fit the model using L1 norm
initial_guess = [100, 0.01]  # Initial guesses for 'a' and 'b'
result = minimize(residuals_l1, initial_guess, args=(x, y), method='BFGS')

# Extract the optimized parameters
a_opt, b_opt = result.x
print(f"Optimized parameters: a = {a_opt}, b = {b_opt}")

# Step 5: Predict and plot results
import matplotlib.pyplot as plt

# Plot original data
plt.scatter(x, y, label='Data')

# Plot the fitted model
x_plot = np.linspace(min(x), max(x), 100)
y_plot = model([a_opt, b_opt], x_plot)
plt.plot(x_plot, y_plot, color='red', label='Fitted Model (L1 norm)')

plt.legend()
plt.xlabel('x')
plt.ylabel('y')
plt.show()