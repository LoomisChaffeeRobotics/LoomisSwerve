package org.firstinspires.ftc.teamcode.swerve;

import java.util.Arrays;

public class OptimalAngleCalculator {
    public boolean[] requiresReversing = new boolean[4];


//    public OptimalAngleCalculator(double currentAngle, double vectorAngle)
//    {
//        calculateOptimalAngle(currentAngle, vectorAngle);
//    }
    // idt there are any parameters to need to be initialized (faster runtimes) but I might be wrong
    public double calculateOptimalAngle(double currentAngle, double vectorAngle, int m) {

//        double angleDifference = Math.abs(normalizedCurrentAngle - normalizedVectorAngle);
//        double zeroTolerance = 15;

        double[] angs = {
                vectorAngle,
                vectorAngle + 180,
                vectorAngle + 360,
                vectorAngle - 180,
                vectorAngle - 360
                };

        double[] diffs = new double[5];
        for (int i = 0; i < angs.length; i++) {
            diffs[i] = Math.abs(angs[i] - currentAngle);
        }

        Arrays.sort(diffs);

        double smallest = diffs[0];
        if (smallest == vectorAngle + 180 || smallest == vectorAngle - 180) {
            requiresReversing[m] = true;
        } else {
            requiresReversing[m] = false;
        }

        return smallest; //TODO; THIS IS BAD

//        if (Math.abs(differenceNormal) < Math.abs(differenceAdded) && Math.abs(differenceNormal) < Math.abs(differenceSubtracted)) {
//            //normal is least
//            requiresReversing = false;
//            return normalizedVectorAngle;
//        } else if (Math.abs(differenceAdded) < Math.abs(differenceNormal) && Math.abs(differenceAdded) < Math.abs(differenceSubtracted)) {
//            requiresReversing = true;
//            return addedAngle;
//        } else if {
//            requiresReversing = true;
//            return subtractedAngle;
//        }
//        if (Math.abs(normalizedVectorAngle) < zeroTolerance) {
//            requiresReversing = false;
//            return normalizedVectorAngle;
//        } else if (270 > angleDifference && angleDifference > 90 ) {
//            requiresReversing = true;
//            return normalizeAngle(normalizedVectorAngle + 180);
//            // might be the problem if the range is supposed to be 0 to 360
//            // but sometimes it's -180 + 180 is 0 and 0 + 180 is 180 and 180 + 180 is 360
//            // which is bigger than reachable
//        } else if (270 > angleDifference) {
//            requiresReversing = false;
//            return normalizeAngle(normalizedVectorAngle + 360);
//        } else {
//            requiresReversing = false;
//            return normalizedVectorAngle;
//        }
    }
    public boolean requiresReversing(int m) {
        return requiresReversing[m];
    }
    public boolean[] reverses(){
        return requiresReversing;
    }
    
}
