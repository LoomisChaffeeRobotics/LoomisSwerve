package org.firstinspires.ftc.teamcode.swerve;


import android.util.Pair;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;


public class vectorsToAngleAndDrive {
    OpMode OM;
    Gamepad gamepad; // perhaps a set power method would be better here?
    private static double aP, aI, aD, dP, dI, dD;
    public static double inPerTick = (7.421/2)/537.7;
    voltageToAngleConstants angleGetter;
    gamepadToVectors vectorGetter;
    PIDController[] anglePID;
    PIDController[] drivePID;
    double[] anglePowers = new double[4];
    double[] drivePowers = new double[4];
    DcMotor[] driveMotors = new DcMotor[4];
    CRServo[] angleMotors = new CRServo[4];
    int[] lastDriveEncoders = new int[4];
    ElapsedTime motorTimer;
    OptimalAngleCalculator angleFixer;
    double[] angles = new double[4];
    ArrayList<Pair<Double,Double>> targetADPairList = new ArrayList<>(4); // key = mag, value = direction
    public vectorsToAngleAndDrive(double length, double width, double maxRot, double maxTrans, OpMode opmode, Gamepad GP, HardwareMap hw, String[] encoderNames, String[] driveNames, String[] angleNames, double angleP, double angleI, double angleD, double driveP, double driveI, double driveD) {
        OM = opmode;
        gamepad = GP;
        aP = angleP; //I don't know what I'm doing - owner of the code
        aI = angleI;
        aD = angleD;
        dP = driveP;
        dI = driveI;
        dD = driveD;
        anglePID = new PIDController[]{new PIDController(aP, aI, aD), new PIDController(aP, aI, aD), new PIDController(aP, aI, aD), new PIDController(aP, aI, aD)};
        drivePID = new PIDController[]{new PIDController(dP, dI, dD), new PIDController(dP, dI, dD), new PIDController(dP, dI, dD), new PIDController(dP, dI, dD)};
        angleGetter = new voltageToAngleConstants(opmode, hw, encoderNames);
        vectorGetter = new gamepadToVectors();
        angleFixer = new OptimalAngleCalculator();
        vectorGetter.maxRotationSpeed = maxRot;
        vectorGetter.maxTranslationSpeed = maxTrans;
        vectorGetter.ROBOT_LENGTH = length;
        vectorGetter.ROBOT_WIDTH = width;
        driveMotors[0] = hw.get(DcMotor.class, driveNames[0]);
        driveMotors[1] = hw.get(DcMotor.class, driveNames[1]);
        driveMotors[2] = hw.get(DcMotor.class, driveNames[2]);
        driveMotors[3] = hw.get(DcMotor.class, driveNames[3]);
        angleMotors[0] = hw.get(CRServo.class, angleNames[0]);
        angleMotors[1] = hw.get(CRServo.class, angleNames[1]);
        angleMotors[2] = hw.get(CRServo.class, angleNames[2]);
        angleMotors[3] = hw.get(CRServo.class, angleNames[3]);
        motorTimer = new ElapsedTime();
        for (int i = 0; i < driveMotors.length; i++) {
            lastDriveEncoders[i] = driveMotors[i].getCurrentPosition();
        }
        targetADPairList.ensureCapacity(4);
        targetADPairList.add(new Pair<>(0.0, 0.0));
        targetADPairList.add(new Pair<>(0.0, 0.0));
        targetADPairList.add(new Pair<>(0.0, 0.0));
        targetADPairList.add(new Pair<>(0.0, 0.0));

        // init the other devices
    }
    public void init_loop () {
        angleGetter.init_loop();
    }
    public void updateMagnitudeDirectionPair(double currentAngle, int m) {
        // Get the combined vector from gamepad inputs
        double[] componentsVector = vectorGetter.getCombinedVector(
                gamepad.left_stick_x,
                -gamepad.left_stick_y,
                gamepad.right_stick_x,
                gamepadToVectors.Wheel.values()[m]
        );

        // Calculate magnitude and direction
        double magnitude = Math.sqrt(Math.pow(componentsVector[0], 2) + Math.pow(componentsVector[1], 2));

        // Check if there is minimal or no input from the gamepad
        if (Math.abs(magnitude) < 0.05) { // Threshold for zero input (adjust if necessary)
            // Retain the previous Pair if there is minimal input
            return;
        }
        // the problem is that the below line goes from -180 to 180, but the voltage lookups go from 0 to 360.
        // we tried adding 180 to it before and it didn't really solve it*, but might have missed some others
        // *possibly because -180 + 180 is NOT the same angle math-wise as 0, but code wise it is
        // so maybe it's not this that's the source of the problem? something larger or elsewhere?
        double direction = Math.toDegrees(Math.atan2(componentsVector[1], componentsVector[0]));
        if (direction < 0) {
            direction += 360;
        }
        direction = angleFixer.calculateOptimalAngle(currentAngle, direction); // perhaps this is where fixes needed?

        // Adjust the magnitude if a direction reversal is needed
        if (angleFixer.requiresReversing()) {
            magnitude = -magnitude;
        }

        // Update the targetADPairList with the new Pair values
        Pair<Double, Double> pair = new Pair<>(magnitude, direction);
        targetADPairList.set(m, pair);
    }

    public double getVelocity(int tickChange, double timeChange) {
        double ticksPerSecond = tickChange/timeChange;
        return inPerTick * ticksPerSecond;
    }
    public void loop() {
        angles = angleGetter.getBigPulleyAngles();
        angleGetter.loop();
//        OM.telemetry.addData("Angles", angles.length);
//        OM.telemetry.addData("Angle PId", anglePID.length);
//        OM.telemetry.addData("Drive PId", drivePID.length);
//        OM.telemetry.update();

        for (int i = 0; i < angles.length; i++) {
            updateMagnitudeDirectionPair(angles[i], i);
            // refresh target numbers based on current pose
            int tickChange = driveMotors[i].getCurrentPosition() - lastDriveEncoders[i];
            // calculate current velocity
            anglePID[i].setSetPoint(targetADPairList.get(i).second);
            drivePID[i].setSetPoint(targetADPairList.get(i).first);
            // set PID target to be the ones calculated earlier
            double speedOutput;

        // here be PIDs
        // control motors, setpower
            double angleOutput = anglePID[i].calculate(angles[i]); // TODO: This returns NaN but input is not the problem
            if (Math.abs(angles[i] - targetADPairList.get(i).second) < 5) {
                if (angleFixer.requiresReversing) {
                    speedOutput = Math.sqrt(Math.pow(gamepad.left_stick_x, 2) + Math.pow(gamepad.left_stick_y, 2)) * -1;
                } else {
                    speedOutput = Math.sqrt(Math.pow(gamepad.left_stick_x, 2) + Math.pow(gamepad.left_stick_y, 2));
                }
            } else {
                speedOutput = 0;
            }


//            // this is returning 0 for some reason
//            // set PID current to current things
            angleMotors[i].setPower(-angleOutput);
            driveMotors[i].setPower(speedOutput);

            lastDriveEncoders[i] = driveMotors[i].getCurrentPosition();

            anglePowers[i] = angleOutput;
            drivePowers[i] = speedOutput;
            // reset the last position and time for velocity calcs
        }

    }
    public void setPID(double ap, double ai, double ad, double dp, double di, double dd) {
        for (PIDController pid : anglePID) {
            pid.setPID(ap, ai, ad);
        }
        for (PIDController pid : drivePID) {
            pid.setPID(dp, di, dd);
        }
    }
    public void getTelemetry(Telemetry t) {
        t.addData("FLTargAng", targetADPairList.get(0).second);
        t.addData("FRTargAng", targetADPairList.get(1).second);
        t.addData("BLTargAng", targetADPairList.get(2).second);
        t.addData("BRTargAng", targetADPairList.get(3).second);
        t.addData("FLIn", angles[0]);
        t.addData("FRIn", angles[1]);
        t.addData("BLIn", angles[2]);
        t.addData("BRIn", angles[3]);
        t.addData("FLAng", anglePowers[0]);
        t.addData("FRAng", anglePowers[1]);
        t.addData("BLAng", anglePowers[2]);
        t.addData("BRAng", anglePowers[3]);
        t.addData("FLDrive", drivePowers[0]);
        t.addData("FRDrive", drivePowers[1]);
        t.addData("BLDrive", drivePowers[2]);
        t.addData("BRDrive", drivePowers[3]);
        t.update();
    }
}
