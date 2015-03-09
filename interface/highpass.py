import numpy as np
 
fc = 0.005  # Cutoff frequency as a fraction of the sample rate (in (0, 0.5)).
b = 0.003  # Transition band, as a fraction of the sample rate (in (0, 0.5)).
N = int(np.ceil((4 / b)))
if not N % 2: N += 1  # Make sure that N is odd.
n = np.arange(N)
 
# Compute a low-pass filter.
h = np.sinc(2 * fc * (n - (N - 1) / 2.))
w = np.blackman(N)
h = h * w
h = h / np.sum(h)
 
# Create a high-pass filter from the low-pass filter through spectral inversion.
h = -h
h[(N - 1) / 2] += 1