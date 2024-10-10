package org.firstinspires.ftc.teamcode.swerve;

public class OptimalAngleCalculator {
    public double targetAngle;
    public boolean requiresReversing;

    public OptimalAngleCalculator(double currentAngle, double vectorAngle)
    {
        calculateOptimalAngle(currentAngle, vectorAngle);
    }
    public void calculateOptimalAngle(double currentAngle, double vectorAngle)
    {
        double normalizedCurrentAngle = normalizeAngle(currentAngle);
        double normalizedVectorAngle = normalizeAngle(vectorAngle);
        double angleDifference = Math.abs(normalizedCurrentAngle - normalizedVectorAngle);
        if (angleDifference > 90) {
            targetAngle = normalizeAngle(normalizedVectorAngle + 180);
            requiresReversing = true;
        } else {
            // Go towards the vector's angle
            targetAngle = normalizedVectorAngle;
            requiresReversing = false;
        }
    }
    }
    public double normalizeAngle(double angle) {
        return (angle % 360);
    }
    public double getTargetAngle() {
        return targetAngle;
    }

    public boolean requiresReversing() {
        return requiresReversing;
    }
    
}
