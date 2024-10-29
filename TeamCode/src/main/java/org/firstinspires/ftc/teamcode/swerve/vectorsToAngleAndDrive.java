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


public class vectorsToAngleAndDrive {
    Gamepad gamepad;
    private static double aP, aI, aD, dP, dI, dD;
    public static double inPerTick = (7.421/2)/537.7;
    voltageToAngleConstants angleGetter;
    int reversed = 0;
    double[] originalVector;
    gamepadToVectors vectorGetter;
    PIDController[] anglePID;
    PIDController[] drivePID;
    String[] driveNames, angleNames;
    DcMotor[] driveMotors;
    CRServo[] angleMotors;
    int[] lastDriveEncoders;
    ElapsedTime motorTimer;
    OptimalAngleCalculator angleFixer;
    double[] angles;
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
        // init the other devices
    }
    public void updateMagnitudeDirectionPair (double currentAngle, int m) {
        double[] componentsVector = vectorGetter.getCombinedVector(gamepad.left_stick_x, gamepad.left_stick_y, gamepad.right_stick_x, gamepadToVectors.Wheel.values()[m]);
        double magnitude = Math.sqrt(Math.pow(componentsVector[0], 2) + Math.pow(componentsVector[1], 2));
        double direction = Math.atan(componentsVector[1]/componentsVector[0]);
        angleFixer.calculateOptimalAngle(currentAngle, direction);
        direction = angleFixer.getTargetAngle();
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
        // based on voltage get the angles
        for (int i = 0; i < targetADPairList.size(); i++) {
            updateMagnitudeDirectionPair(angles[i], i);
            // refresh target numbers based on current pose
            int tickChange = driveMotors[i].getCurrentPosition() - lastDriveEncoders[i];
            double currentVelocity = getVelocity(tickChange, motorTimer.seconds());
            // calculate current velocity
            anglePID[i].setSetPoint(targetADPairList.get(i).second);
            drivePID[i].setSetPoint(targetADPairList.get(i).first);
            // set PID target to be the ones calculated earlier


        // here be PIDs
        // control motors, setpower
            double angleOutput = anglePID[i].calculate(angles[i]);
            double speedOutput = drivePID[i].calculate(currentVelocity);
            // set PID current to current things
            angleMotors[i].setPower(angleOutput);
            driveMotors[i].setPower(speedOutput);

            lastDriveEncoders[i] = driveMotors[i].getCurrentPosition();
            motorTimer.reset();
            // reset the last position and time for velocity calcs
        }

    }
}
