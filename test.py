import matplotlib.pyplot as plt

# Create a simple plot
x = [1, 2, 3, 4]
y = [1, 4, 9, 16]
plt.plot(x, y)

# Add vector arrow annotation at a specific point
plt.annotate(r'$\vec{v}$', xy=(3, 9), xytext=(4, 16),
             arrowprops=dict(facecolor='black', shrink=0.5, width=2, headwidth=10))

# Add labels with vector notation
plt.xlabel(r'$\vec{x}$', fontsize=15)
plt.ylabel(r'$\vec{y}$', fontsize=15)

# Display the plot
plt.show()
