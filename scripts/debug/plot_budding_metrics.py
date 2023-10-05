#!/bin/python3

# Author : Alberto M. Esmoris Pena
# Script to plot budding metrics.
# It is useful to debug budding taks dropper behavior and its impact on idle
# time of pulse thread pool

# Imports
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# Read data from CSV file
PATH = '../../budding_metrics.csv'
X = pd.read_csv(PATH, header=None).to_numpy()
nTicks = X.shape[0]
T = np.linspace(0, 1, nTicks)
print(f'Read {X.shape} dataset from "{PATH}"')
idle_time = X[:, 0]/10e4
step = X[:, 2] * X[:, 4]

# Summary
print(f'Total idle time: {np.sum(idle_time)}')


# ---  P L O T  --- #
# ----------------- #
# Plot utils
def format_subplot(ax, ylabel=None, legend=True):
    ax.grid('both')
    ax.set_axisbelow(True)
    ax.set_xlabel(f'Execution timeline ({nTicks} ticks)')
    if ylabel is not None:
        ax.set_ylabel(ylabel)
    if legend:
        ax.legend(loc='best')


# Prepare plot
fig = plt.figure(figsize=(16, 9))

# Plot idle time
ax = fig.add_subplot(2, 3, 1)
ax.set_title('Idle time in $10^{-4}$ s')
ax.plot(T, idle_time, color='red', label='idle')
format_subplot(ax, ylabel='$10^{-4}$ s')

# Plot chunk size
ax = fig.add_subplot(2, 3, 2)
ax.set_title('Chunk size')
ax.plot(T, X[:, 1], color='black', label='chunk')
format_subplot(ax, ylabel='Chunk size')

# Plot step
ax = fig.add_subplot(2, 3, 3)
ax.set_title('Step')
ax.plot(T, X[:, 2], color='green', label='$|$step$|$')
ax.plot(T, step, color='darkcyan', label='step', alpha=0.7)
format_subplot(ax, ylabel='Step')

# Plot idle time + chunk size
ax = fig.add_subplot(2, 3, 4)
ax.set_title('Idle time + chunk size')
plots = ax.plot(T, idle_time, color='red', label='idle')
format_subplot(ax, ylabel='$10^{-4}$ s', legend=False)
axt = ax.twinx()
plots += axt.plot(T, X[:, 1], color='black', label='chunk')
axt.set_ylabel('Chunk size')
ax.legend(plots, [plot.get_label() for plot in plots], loc='best')

# Plot step size
ax = fig.add_subplot(2, 3, 5)
ax.set_title('Step size')
ax.plot(T, X[:, 2], color='green', label='step size')
format_subplot(ax, ylabel='Step size')

# Plot chunk size + step size
ax = fig.add_subplot(2, 3, 6)
ax.set_title('Chunk size + step size')
plots = ax.plot(T, X[:, 1], color='black', label='chunk')
format_subplot(ax, ylabel='Chunk size', legend=False)
axt = ax.twinx()
plots += axt.plot(T, X[:, 2], color='green', label='step')
axt.set_ylabel('Step size')
ax.legend(plots, [plot.get_label() for plot in plots], loc='best')


# Postprocess plot
plt.tight_layout()
plt.show()
