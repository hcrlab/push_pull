# Item classification

## Color histograms
A preliminary experiment suggests that we can classify clusters with 85 - 90% accuracy just by using color histograms.
The classification is done by computing the distance from the current histogram to previously recorded color histograms.

### Data set
We recorded color histograms for 21 objects (some were missing from the lab).
The color histograms had 4 bins for each of the red, green, and blue channels, for a 12-dimensional histogram.
We put one object in bin J, got the cluster from the shelf cropping service, and recorded the histogram for that cluster.
In the training set, we recorded multiple histograms for some items, because some items may look different from different poses.
For the test set, we used the same procedure to get a single data point for each of the objects in the training set.
All objects were placed near the front of the shelf.
Some objects are impossible for the Kinect to see, such as the outlet plugs.

Assumptions:
- The object will be close to the front of the shelf and visible.
- The object will be mostly lit from overhead, since it is not deep in the shelf.
- The object appearence doesn't change when on the upper shelves.
- The presence of other objects in the same bin doesn't affect the appearance of an object.

- [Training set](https://gitlab.cs.washington.edu/amazon-picking-challenge-2015/pr2_pick/blob/93505bb901e316250b6fb8459566a6abc804ee24/pr2_pick_contest/config/item_models.json)
- [Test set](https://gitlab.cs.washington.edu/amazon-picking-challenge-2015/pr2_pick/blob/93505bb901e316250b6fb8459566a6abc804ee24/pr2_pick_contest/data/color_histogram_test_set.json)

### Experiment
For each element of the test set, we tried to classify what item it was, given a list of 2 or 3 items it could be.
This was done for every possible combination of 2 or 3 items for which we had data in the data set.
The classification is done by computing the distance from the current histogram to the training set histograms for the candidate items.
We tried L1, L2, and cosine distance functions.
We also output a confidence score, which is 1 - (minimum distance) / (2nd minimum distance)

### Results
Using L2, we got 90.0% accuracy on 2-item bins and 83.6% on 3-item bins.
Using L1, we got 91.0% accuracy on 2-item bins and 84.8% on 3-item bins.
Using cosine distance, we got 21.0% accuracy on 2-item bins and 8.65% on 3-item bins.

Across both 2-item and 3-item bins, the items that couldn't be classified with 100% accuracy were:
```
genuine_joe_plastic_stir_sticks
expo_dry_erase_board_eraser
kong_air_dog_squeakair_tennis_ball
kyjen_squeakin_eggs_plush_puppies
highland_6539_self_stick_notes
first_years_take_and_toss_straw_cup
sharpie_accent_tank_style_highlighters
safety_works_safety_glasses
```

The safety glasses were especially hard, with 40% accuracy in the 2-item bin case, and 14.7% accuracy in the 3-item bin case.

In both the 2-item and 3-item bins, all predictions with a confidence score below 0.14 were incorrect.

- [Data sheet](https://docs.google.com/a/cs.washington.edu/spreadsheets/d/1Ba5DRDRUkJ1C_SMGI-77Z_t5HaEpmnloH9Y1AOIeP24/edit?usp=sharing)

## Task performance
The experiment above doesn't translate exactly into overall task performance, because we might assign the same label to multiple clusters in the same bin.
In that case, we could filter out wrong predictions by accepting the most confident prediction, and re-predicting the other cluster or clusters given this information.