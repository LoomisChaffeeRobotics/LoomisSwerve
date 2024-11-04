package org.firstinspires.ftc.teamcode.swerve;

public class OptimalAngleCalculator {
    public boolean requiresReversing;

//    public OptimalAngleCalculator(double currentAngle, double vectorAngle)
//    {
//        calculateOptimalAngle(currentAngle, vectorAngle);
//    }
    // idt there are any parameters to need to be initialized (faster runtimes) but I might be wrong
    public double calculateOptimalAngle(double currentAngle, double vectorAngle) {
        double normalizedCurrentAngle = normalizeAngle(currentAngle);
        double normalizedVectorAngle = normalizeAngle(vectorAngle);
        double angleDifference = Math.abs(normalizedCurrentAngle - normalizedVectorAngle);
        double zeroTolerance = 15;

        if (Math.abs(normalizedVectorAngle) < zeroTolerance) {
            requiresReversing = false;
            return normalizedVectorAngle;
        } else if (270 > angleDifference && angleDifference > 90 ) {
            requiresReversing = true;
            return normalizeAngle(normalizedVectorAngle + 180);
            // might be the problem if the range is supposed to be 0 to 360
            // but sometimes it's -180 + 180 is 0 and 0 + 180 is 180 and 180 + 180 is 360
            // which is bigger than reachable
        } else if (270 > angleDifference) {
            requiresReversing = false;
            return normalizeAngle(normalizedVectorAngle + 360);
        } else {
            requiresReversing = false;
            return normalizedVectorAngle;
        }
    }
    public double normalizeAngle(double angle) {
        return (angle % 360);
    }

    public boolean requiresReversing() {
        return requiresReversing;
    }
    
}
