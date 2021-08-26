handD = readtable("projectOut/handData.csv");
%frequency of sample rate 
time = handD{:, 1};
diff = checkFrameDiff(time);
avgDiffHand = average(diff)

skeletonD = readtable("projectOut/skeleData.csv");
timeSkele = skeletonD{:, 19};
diff = checkFrameDiff(timeSkele);
avgDiffSkel = average(diff)


