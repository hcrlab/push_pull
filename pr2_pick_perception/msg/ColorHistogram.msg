# A color histogram. If there are 4 bins, then the histogram is laid out like
# r1, r2, r3, r4, g1, g2, g3, g4, b1, b2, b3, b4
# r1 is the number of colors between 0-63, r2 between 64-127, etc.
int32[] histogram

# The number of bins for each color.
int32 num_bins
