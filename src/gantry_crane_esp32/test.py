import numpy as np

# Create a sample matrix
matrix = np.array([[2, -5, 0],
                   [7, 0, -3],
                   [-1, 4, -6]])

# Get the sign of every element in the matrix
sign_matrix = np.sign(matrix)

print("Original Matrix:")
print(matrix)

print("\nMatrix with Signs:")
print(sign_matrix)
