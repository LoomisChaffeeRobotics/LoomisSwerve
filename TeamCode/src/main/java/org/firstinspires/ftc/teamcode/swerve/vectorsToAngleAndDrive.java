package org.firstinspires.ftc.teamcode.swerve;


import android.util.Pair;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;


public class vectorsToAngleAndDrive {
    Gamepad gamepad;
    private double aP, aI, aD, dP, dI, dD;
    voltageToAngleConstants angleGetter;
    int reversed = 0;
    double[] originalVector;
    gamepadToVectors vectorGetter;
    PIDController[] anglePID;
    PIDController[] drivePID;
    String[] driveNames, angleNames;
    OptimalAngleCalculator angleFixer;
    DcMotor dFL, dFR, dBL, dBR;
    CRServo aFL, aFR, aBL, aBR;
    ArrayList<Pair<Double,Double>> targetADPairList = new ArrayList<>(4); // key = mag, value = direction
    public vectorsToAngleAndDrive(OpMode opmode, Gamepad GP, HardwareMap hw, String[] encoderNames, String[] driveNames, String[] angleNames, double angleP, double angleI, double angleD, double driveP, double driveI, double driveD) {
        gamepad = GP;
        aP = angleP; //I don't know what I'm doing - owner of the code
        aI = angleI;
        aD = angleD;
        dP = driveP;
        dI = driveI;
        dD = driveD;
        anglePID = new PIDController[]{new PIDController(aP, aI, aD)};
        drivePID = new PIDController[]{new PIDController(dP, dI, dD)};
        angleGetter = new voltageToAngleConstants(opmode, hw, encoderNames);
        vectorGetter = new gamepadToVectors();
        angleFixer = new OptimalAngleCalculator();
        dFL = hw.get(DcMotor.class, driveNames[0]);
        dFR = hw.get(DcMotor.class, driveNames[1]);
        dBL = hw.get(DcMotor.class, driveNames[2]);
        dBR = hw.get(DcMotor.class, driveNames[3]);
        aFL = hw.get(CRServo.class, angleNames[0]);
        aFR = hw.get(CRServo.class, angleNames[1]);
        aBL = hw.get(CRServo.class, angleNames[2]);
        aBR = hw.get(CRServo.class, angleNames[3]);
        // init the other devices
    }
    public void updateMagnitudeDirectionPair (double currentAngle, int m) {
        double[] componentsVector = vectorGetter.getCombinedVector(gamepad.left_stick_x, gamepad.left_stick_y, gamepad.right_stick_x, gamepadToVectors.Wheel.values()[m]);
        double magnitude = Math.sqrt(Math.pow(componentsVector[0], 2) + Math.pow(componentsVector[1], 2));
        double direction = Math.atan(componentsVector[1]/componentsVector[0]);

        Pair<Double, Double> pair = new Pair<>(magnitude, direction);
        targetADPairList.set(m, pair);
    }

    public void loop() {
        for (int i = 0; i < targetADPairList.size(); i++) {
            updateMagnitudeDirectionPair(i);
            anglePID[i].setSetPoint(targetADPairList.get(i).second);
            drivePID[i].setSetPoint(targetADPairList.get(i).first);
        }

        // here be PIDs
    }
}
