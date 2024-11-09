package org.firstinspires.ftc.teamcode.swerve;

import androidx.annotation.NonNull;


import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.NonConst;

public class gamepadToVectors {
    public double maxTranslationSpeed = 1;
    public double maxRotationSpeed = 0.5;
    public double ROBOT_LENGTH = 1.0;  // Length
    public double ROBOT_WIDTH = 1.0;   // Width


    public double[] limitVector(double[] vector, double maxSpeed) {
        double magnitude = Math.sqrt(vector[0] * vector[0] + vector[1] * vector[1]);
        if (magnitude > maxSpeed) {
            vector[0] = (vector[0] / magnitude) * maxSpeed;
            vector[1] = (vector[1] / magnitude) * maxSpeed;
        }
        return vector;
    }

    public double[] getTranslationVector(double translateX, double translateY) {
        double[] translationVector = {translateX, translateY};

        return limitVector(translationVector, maxTranslationSpeed);
    }

    public double getRotationSpeed(double rotationX) {
        return rotationX * maxRotationSpeed; // ask emma about gamepad
    }

    public double[] getCombinedVector (double x, double y,double rx, Wheel wheel) {
            double[] translationVector = getTranslationVector(x, y);
            double rotationSpeed = getRotationSpeed(rx);

            double[] combinedVector = {
                    translationVector[0] + rotationSpeed*Math.sin(getWheelAngle(wheel)),
                    translationVector[1] + rotationSpeed*Math.cos(getWheelAngle(wheel)),
            };

            // public static variables for Length and Width
            //some way to tell which wheel is being talked about
            //arctan calculations as seen above
            //return based on which wheel it is after adding it

            return combinedVector;

    }



        // public static variables for robot dimensions

        //which wheel
    public enum Wheel {
        fl, bl, fr, br
    }

    public double getRotationRadius() {
        return 0.5 * Math.sqrt(Math.pow(ROBOT_WIDTH, 2) + Math.pow(ROBOT_LENGTH, 2));
    }

//arc tan calculations
    public double getWheelAngle(Wheel wheel) {
        double angle = 0.0;
        switch (wheel) {
            case fl:
                angle = Math.atan(ROBOT_WIDTH / ROBOT_LENGTH) + Math.PI / 2;
                break;
            case fr:
                angle = Math.PI / 2 - Math.atan(ROBOT_WIDTH / ROBOT_LENGTH);
                break;
            case bl:
                angle = 3 * Math.PI/2 - (Math.atan(ROBOT_WIDTH / ROBOT_LENGTH));
                break;
            case br:
                angle =  Math.PI * 2 + Math.atan(ROBOT_WIDTH / ROBOT_LENGTH) - Math.PI/2;
                break;
        }
        return angle;
    }
}