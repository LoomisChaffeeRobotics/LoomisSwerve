package org.firstinspires.ftc.teamcode.swerve;


import android.text.style.EasyEditSpan;
import android.util.Pair;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.Arrays;


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
    public void updateMagnitudeDirectionPair (double currentAngle, int m) {
        double[] componentsVector = vectorGetter.getCombinedVector(gamepad.left_stick_x, gamepad.left_stick_y, gamepad.right_stick_x, gamepadToVectors.Wheel.values()[m]);
        double magnitude = Math.sqrt(Math.pow(componentsVector[0], 2) + Math.pow(componentsVector[1], 2));
        double direction = Math.atan(componentsVector[1]/componentsVector[0]);
        angleFixer.calculateOptimalAngle(currentAngle, direction);
        direction = angleFixer.getTargetAngle();
        if (angleFixer.requiresReversing()) {
            magnitude = -magnitude;
        }
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
        OM.telemetry.addData("Angles", angles.length);
        OM.telemetry.addData("Angle PId", anglePID.length);
        OM.telemetry.addData("Drive PId", drivePID.length);
        OM.telemetry.update();

        for (int i = 0; i < angles.length; i++) {
            updateMagnitudeDirectionPair(angles[i], i);
            // refresh target numbers based on current pose
            int tickChange = driveMotors[i].getCurrentPosition() - lastDriveEncoders[i];
            double currentVelocity = getVelocity(tickChange, motorTimer.seconds());
            // calculate current velocity
            anglePID[i].setSetPoint(targetADPairList.get(i).second); // TODO: This is NaN for some reason?
            if (Double.isNaN(angles[i]) || Double.isInfinite(angles[i])) {
                OM.telemetry.addData("Warning", "Invalid angle value at index " + i);
                continue; // Skip this iteration if angle is invalid
            }

            drivePID[i].setSetPoint(targetADPairList.get(i).first);
            // set PID target to be the ones calculated earlier


        // here be PIDs
        // control motors, setpower
            double angleOutput = anglePID[i].calculate(angles[i]); // TODO: This returns NaN but input is not the problem
            double speedOutput = drivePID[i].calculate(currentVelocity);
//            // this is returning 0 for some reason
//            // set PID current to current things
//            angleMotors[i].setPower(angleOutput);
//            driveMotors[i].setPower(speedOutput);

            lastDriveEncoders[i] = driveMotors[i].getCurrentPosition();
            motorTimer.reset();
            OM.telemetry.addData("outputs", angleOutput + ", " + speedOutput);
            OM.telemetry.addData("input", angles[i]);
            OM.telemetry.addData("setpoint", targetADPairList.get(i).second);
            OM.telemetry.addData("magnitude", targetADPairList.get(i).first);
            OM.telemetry.update();

//            anglePowers[i] = angleOutput;
//            drivePowers[i] = speedOutput;
            // reset the last position and time for velocity calcs
        }

    }
    public String getTelemetry() {
        return Arrays.toString(anglePowers) + Arrays.toString(drivePowers);
    }
}
