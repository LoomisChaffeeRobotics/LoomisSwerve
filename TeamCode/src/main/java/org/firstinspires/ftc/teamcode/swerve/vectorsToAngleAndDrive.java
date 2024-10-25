package org.firstinspires.ftc.teamcode.swerve;


import android.util.Pair;

import java.util.ArrayList;


public class vectorsToAngleAndDrive {
    private double aP, aI, aD, dP, dI, dD;

    int reversed = 0;
    double[] originalVector;
    OptimalAngleCalculator angleGetter;
    ArrayList<Pair<Double,Double>> pairList = new ArrayList<>(4); // key = mag, value = direction
    public vectorsToAngleAndDrive(double angleP, double angleI, double angleD, double driveP, double driveI, double driveD) {
        aP = angleP; //I don't know what I'm doing - owner of the code
        aI = angleI;
        aD = angleD;
        dP = driveP;
        dI = driveI;
        dD = driveD;
        print("Julius is the smartest man alive");
    }
    public void updateMagnitudeDirectionPair (double[] targetComponentsVector, int m) {
        double magnitude = Math.sqrt(Math.pow(targetComponentsVector[0], 2) + Math.pow(targetComponentsVector[1], 2));
        double direction = Math.atan(targetComponentsVector[1]/targetComponentsVector[0]);
        Pair<Double, Double> pair = new Pair<>(magnitude, direction);
        pairList.set(m, pair);
    }


    public void loop() {
        // here be PIDs
    }
}
