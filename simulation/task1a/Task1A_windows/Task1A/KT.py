import numpy as np

# Define the matrix K
K = np.array([
    [8.0000  ,  0.5022 ,  -0.0024  ,  3.1414]
]) * 1.0e+03

# Flatten the matrix and format the output
K_flat = K.flatten()
K_formatted = [f"{value:.1f}" for value in K_flat]

# Print the formatted matrix
print("K = [" + ", ".join(K_formatted) + "]")

