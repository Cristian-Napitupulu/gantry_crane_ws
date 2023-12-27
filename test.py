import numpy as np

# Example arrays
array1 = np.array([True, True, False, False])
array2 = np.array([True, False, True, False])

# Applying logical AND operation
result = np.logical_and(array1, array2)

# Output
print(result)
