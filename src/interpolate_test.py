import numpy as np
from scipy.interpolate import interp1d
from matplotlib import pyplot as plt

t = np.array([0, 1, 1.2, 1.34, 1.57, 1.98, 2.8, 5, 6.12, 7.8, 9.1, 9.93, 9.97, 10])
x = np.exp(-t/3.0)
f = interp1d(t, x)

t_interp = np.arange(0,10,0.1)
x_interp = f(t_interp)

plt.plot(t,x)
plt.show()

plt.plot(t_interp,x_interp)
plt.show()

print(t_interp)
print(x_interp)