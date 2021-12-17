import pandas as pd
import numpy as np
import sys

# INPUT ARGUMENTS
inPath = sys.argv[1]
gpsTimeIdx = int(sys.argv[2])
outPath = sys.argv[3] if len(sys.argv) > 3 else inPath
inDelimiter = ' ' if len(sys.argv) < 5 else sys.argv[4]
outDelimiter = ' ' if len(sys.argv) < 6 else sys.argv[5]

# GENERATE OUTPUT
X = pd.read_csv(sys.argv[1], delimiter=inDelimiter).to_numpy()
offset = np.min(X[:, gpsTimeIdx])
X[:, gpsTimeIdx] = X[:, gpsTimeIdx] - offset

# WRITE OUTPUT
np.savetxt(outPath, X, delimiter=outDelimiter, newline='\n')
