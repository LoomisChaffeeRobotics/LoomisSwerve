package org.firstinspires.ftc.teamcode.subsystems.swerve;

import com.qualcomm.robotcore.hardware.IMU;

public class gamepadToVectors {
    public double maxTranslationSpeed = 2;
    public double maxRotationSpeed = 2;
    public double ROBOT_LENGTH = 1.0;  // Length
    public double ROBOT_WIDTH = 1.0;   // Width
    IMU imu;
    public double[] limitVector(double[] vector) {
        // normalizes a greater vector
        double magnitude = Math.sqrt(vector[0] * vector[0] + vector[1] * vector[1]);
        if (magnitude > 1) {
            vector[0] = (vector[0] / magnitude);
            vector[1] = (vector[1] / magnitude);
        }
        return vector;
    }

    public double[] getTranslationVector(double translateX, double translateY) {
        double[] translationVector = {translateX, translateY};
        return limitVector(translationVector);
    }
    public double[] fieldCentrifyVector(double theta, double rx, double[] targetVector, Wheel wheel) {
        // another field centric option at earlier stage than SwerveDrive's loopFC
        double x = targetVector[0];
        double y = targetVector[1];
        double theta2 = Math.toRadians(theta);
        double fieldX = x * Math.cos(theta2) - y * Math.sin(theta2);
        double fieldY = x * Math.sin(theta2) + y * Math.cos(theta2);

        double a = fieldX + rx * (1 / Math.sqrt(2));
        double b = fieldX - rx * (1 / Math.sqrt(2));
        double c = fieldY + rx * (1 / Math.sqrt(2));
        double d = fieldY - rx * (1 / Math.sqrt(2));

        double[] output = new double[2];
        switch (wheel) {
            case bl:
                output = new double[]{b, d};
                break;
            case br:
                output = new double[]{b, c};
                break;
            case fl:
                output = new double[]{a, d};
                break;
            case fr:
                output = new double[]{a, c};
                break;
        }

        return output;
    }
    public double[] getCombinedVector (double x, double y,double rx, Wheel wheel) {
            double[] translationVector = getTranslationVector(x, y);
            double rotationSpeed = rx;
            double[] combinedVector;
            if (translationVector[0] == 0 && translationVector[1] == 0) {
                combinedVector = new double[]{
                        // allow faster rotation in place
                        translationVector[0] + 1.5 * rotationSpeed * Math.sin(getWheelAngle(wheel)),
                        translationVector[1] + 1.5 * rotationSpeed * Math.cos(getWheelAngle(wheel)),
                };
            } else {
                 combinedVector = new double[]{
                        translationVector[0] + 1.25 *rotationSpeed*Math.sin(getWheelAngle(wheel)),
                        translationVector[1] + 1.25 *rotationSpeed*Math.cos(getWheelAngle(wheel)),
                };
            }
            return combinedVector;
    }
    public enum Wheel {
        fl, fr, bl, br
    }
    public double getRotationRadius() {
        return 0.5 * Math.sqrt(Math.pow(ROBOT_WIDTH, 2) + Math.pow(ROBOT_LENGTH, 2));
    }

//arc tan calculations
    public double getWheelAngle(Wheel wheel) {
        double angle = 0.0;
        switch (wheel) {
            case br:
                angle = Math.atan(ROBOT_WIDTH / ROBOT_LENGTH) + Math.PI / 2;
                break;
            case fr:
                angle = 3 * Math.PI / 2 - Math.atan(ROBOT_WIDTH / ROBOT_LENGTH);
                break;
            case bl:
                angle = Math.PI/2 - (Math.atan(ROBOT_WIDTH / ROBOT_LENGTH));
                break;
            case fl:
                angle =  Math.PI * 2 + Math.atan(ROBOT_WIDTH / ROBOT_LENGTH) - Math.PI/2;
                break;
        }
        return angle;
    }
}