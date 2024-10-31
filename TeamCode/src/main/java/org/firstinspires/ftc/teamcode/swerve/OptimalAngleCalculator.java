package org.firstinspires.ftc.teamcode.swerve;

public class OptimalAngleCalculator {
    public double targetAngle;
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
            return normalizedVectorAngle;
        } else if (angleDifference > 90) {

                requiresReversing = true;
                return normalizeAngle(normalizedVectorAngle + 180);
        }
        else {
            // Go towards the vector's angle

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
