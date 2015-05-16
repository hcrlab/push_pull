# A color histogram. If there are 2 bins, there are 2^3 = 8 entries in the histogram.
# [count(r0, g0, b0), count(r0, g0, b1), count(r0, g1, b0), ..., count(r1, g1, b1)]
# r0 is the number of colors between 0-127, r1 between 127-255, etc.
int32[] histogram

# The number of bins for each color.
int32 num_bins
