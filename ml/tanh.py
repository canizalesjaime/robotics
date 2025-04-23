import numpy as np

def tanh(x):
    return (np.exp(x)-np.exp(-x))/(np.exp(x)+np.exp(-x))


print(tanh(.17))
print(tanh(np.array([1.7,1.2])))