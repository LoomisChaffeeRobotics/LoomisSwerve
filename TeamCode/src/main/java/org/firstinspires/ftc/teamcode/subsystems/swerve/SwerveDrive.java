package org.firstinspires.ftc.teamcode.subsystems.swerve;


import android.util.Pair;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.SlewRateLimiter;

import java.util.ArrayList;
import java.util.Arrays;


@Config
public class SwerveDrive {
    OpMode OM;
    Gamepad gamepad;
    public final double DIST_MULT = 4/3; // Actual dist/desired dist
    public static double aP = 0.02;
    public static double aI = 0.01;
    public static double aD = 0.0015;
    SlewRateLimiter[] wheelLimiters = new SlewRateLimiter[4];
    boolean dontMove;
    boolean reverse;
    public SwerveDriveOdometry odo;
    public ElapsedTime timer;
    public SwerveModuleState[] states = new SwerveModuleState[4];
    SwerveDriveKinematics kinematics; // FTCLib used only for auto code
    public static double inchesPerTick = (0.061*Math.PI / 2.54 * 100)/770;
    voltageToAngleConstants angleGetter;
    gamepadToVectors vectorGetter;
    PIDController[] anglePID;
    double[] driveSpeeds = new double[4];
    double[] anglePowers = new double[4];
    double[] drivePowers = new double[4];
    public DcMotorEx[] driveMotors = new DcMotorEx[4];
    CRServo[] angleMotors = new CRServo[4];
    int[] lastDriveEncoders = new int[4];
    OptimalAngleCalculator angleFixer;
    double[] angles = new double[4];
    public Pose2d nowPose;
    public IMU imu;
    ArrayList<Pair<Double,Double>> targetADPairList = new ArrayList<>(4); // List of pairs (magnitude, direction)
    public SwerveDrive(double length, double width, double maxRot, double maxTrans,
                       OpMode opmode, Gamepad GP, HardwareMap hw,
                       String[] encoderNames, String[] driveNames, String[] angleNames,
                       double startFTCLibX, double startFTCLibY, double startHeadingRads) {
        OM = opmode;
        gamepad = GP;
        anglePID = new PIDController[]{new PIDController(aP, aI, aD), new PIDController(aP, aI, aD), new PIDController(aP, aI, aD), new PIDController(aP, aI, aD)};
        angleGetter = new voltageToAngleConstants(opmode, hw, encoderNames);
        vectorGetter = new gamepadToVectors();
        angleFixer = new OptimalAngleCalculator();
        vectorGetter.maxRotationSpeed = maxRot;
        vectorGetter.maxTranslationSpeed = maxTrans;
        vectorGetter.ROBOT_LENGTH = length;
        vectorGetter.ROBOT_WIDTH = width;
        for (int i = 0; i < driveNames.length; i++) {
            driveMotors[i] = (DcMotorEx) hw.get(DcMotor.class, driveNames[i]);
            driveMotors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveMotors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            angleMotors[i] = hw.get(CRServo.class, angleNames[i]);
            wheelLimiters[i] = new SlewRateLimiter(1/2.625); // max servo-specific acceleration
        }
        imu = hw.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);
        imu.resetYaw();

        for (int i = 0; i < driveMotors.length; i++) {
            driveSpeeds[i] = getVelocity(driveMotors[i]);
            lastDriveEncoders[i] = driveMotors[i].getCurrentPosition();
        }
        targetADPairList.ensureCapacity(4);
        targetADPairList.add(new Pair<>(0.0, 0.0));
        targetADPairList.add(new Pair<>(0.0, 0.0));
        targetADPairList.add(new Pair<>(0.0, 0.0));
        targetADPairList.add(new Pair<>(0.0, 0.0));

        // only for auto
        kinematics = new SwerveDriveKinematics(
                new Translation2d((6.5), (5.5)),
                new Translation2d((6.5),(-5.5)),
                new Translation2d((-4.5), (5.5)),
                new Translation2d((-4.5), (-5.5)));
        odo = new SwerveDriveOdometry(kinematics, new Rotation2d(imu.getRobotYawPitchRollAngles().getYaw()), new Pose2d(startFTCLibX, startFTCLibY, new Rotation2d(startHeadingRads)));
        timer = new ElapsedTime();
        nowPose = new Pose2d();
        for (int i = 0; i < driveMotors.length; i++) {
                states[i] = new SwerveModuleState(driveSpeeds[i], new Rotation2d(Math.toRadians(angles[i])));
        }

    }
    public void resetIMU() { imu.resetYaw();}
    public void init_loop () {
        angleGetter.init_loop();
        timer.reset();
    }
    public void updateMagnitudeDirectionPair(double x, double y, double rx, double currentAngle, int m) {
        // Get the translation & rotation vector from gamepad inputs
        double[] componentsVector = vectorGetter.getCombinedVector(
                x,
                y,
                -rx,
                gamepadToVectors.Wheel.values()[m]
        );

        // Calculate magnitude and direction
        double magnitude = Math.sqrt(Math.pow(componentsVector[0], 2) + Math.pow(componentsVector[1], 2));

        double direction;
        // don't try to get to a point if the joystick is in the deadzone
        if (Math.abs(magnitude) > 0.1) {
            direction = Math.toDegrees(Math.atan2(componentsVector[0], componentsVector[1])) + 180;
            direction = angleFixer.calculateOptimalAngle(currentAngle, direction, m);
            dontMove = false;
        } else {
            Pair<Double, Double> pairNoMove = new Pair<>(0.0, targetADPairList.get(m).second);
            targetADPairList.set(m, pairNoMove);
            dontMove = true;
            return;
        }

        // cosine compensation
        magnitude *= Math.abs(Math.cos(Math.toRadians(Math.abs(angles[m]-direction))));

        // Adjust the magnitude if a direction reversal is needed
        if (angleFixer.requiresReversing(m)) {
            reverse = true;
            Pair<Double, Double> pair = new Pair<>(-magnitude, direction);
            targetADPairList.set(m, pair);
        } else {
            reverse = false;
            Pair<Double, Double> pair = new Pair<>(magnitude, direction);
            targetADPairList.set(m, pair);
        }
    }
    public void stop(){
        angleGetter.stopAndLog();
    }
    public double getVelocity(DcMotorEx motor) {
        double ticksPerSecond = motor.getVelocity();
        return inchesPerTick * ticksPerSecond * DIST_MULT; // inches per second
    }
    public void loopFC(double theta, double x, double y, double rx) {
        // Field Centrify
        double theta2 = Math.toRadians(theta);
        double fieldX = x * Math.cos(theta2) - y * Math.sin(theta2);
        double fieldY = x * Math.sin(theta2) + y * Math.cos(theta2);

        this.loop(fieldX, fieldY, rx);
    }
    public void loop(double x, double y, double rx) {
        angles = angleGetter.getBigPulleyAngles();
        angleGetter.loop();

        for (int i = 0; i < angles.length; i++) {
            updateMagnitudeDirectionPair(x, y, rx, angles[i], i);
            // anglePID controls servo to get error to 0 only when trying to move
            double set;
            double diff;
            double angleOutput;
            if (!dontMove) {
                set = targetADPairList.get(i).second;
            } else {
                set = angles[i];
            }
            diff = angles[i] - set;
            angleOutput = anglePID[i].calculate(diff, 0);
            // rate limits change dynamically based on distance wheel needs to rotate
            // this is to prevent unnecessary slow motor acceleration
            double magnitude = targetADPairList.get(i).first; // can be + or -
            double speedOutput;
            if (Math.abs(magnitude) < 0.05) {
                wheelLimiters[i].updateRateLimit(1/((2.53 * (Math.abs(angles[i]-targetADPairList.get(i).second))/90)+0.00000000000000001));
                speedOutput = wheelLimiters[i].calculate(magnitude);
            } else {
                speedOutput = magnitude;
                wheelLimiters[i].reset(0);
            }
            angleMotors[i].setPower(-angleOutput);
            driveMotors[i].setPower(speedOutput);

            lastDriveEncoders[i] = driveMotors[i].getCurrentPosition();

            anglePowers[i] = angleOutput;
            drivePowers[i] = speedOutput;
            driveSpeeds[i] = getVelocity(driveMotors[i]);
            lastDriveEncoders[i] = driveMotors[i].getCurrentPosition();
            states[i].speedMetersPerSecond = driveSpeeds[i];
            states[i].angle = new Rotation2d(Math.toRadians((angles[i])));
        }
        nowPose = odo.updateWithTime(timer.seconds(), new Rotation2d(Math.toRadians(imu.getRobotYawPitchRollAngles().getYaw())), states);
    }
    public void getTelemetry(Telemetry t) {
        t.addData("FLTargAng", targetADPairList.get(0).second);
        t.addData("FRTargAng", targetADPairList.get(1).second);
        t.addData("BLTargAng", targetADPairList.get(2).second);
        t.addData("BRTargAng", targetADPairList.get(3).second);
        t.addData("inFL", angles[0]);
        t.addData("inFR", angles[1]);
        t.addData("inBL", angles[2]);
        t.addData("inBR", angles[3]);
        t.addData("FLVelocity", states[0].speedMetersPerSecond);
        t.addData("FRVelocity", states[1].speedMetersPerSecond);
        t.addData("BLVelocity", states[2].speedMetersPerSecond);
        t.addData("BRVelocity", states[3].speedMetersPerSecond);
        t.addData("FLState", states[0].angle.getDegrees());
        t.addData("FRState", states[1].angle.getDegrees());
        t.addData("BLState", states[2].angle.getDegrees());
        t.addData("BRState", states[3].angle.getDegrees());
        t.addData("PoseX", nowPose.getX());
        t.addData("PoseY", nowPose.getY());
        t.addData("PoseHeading", nowPose.getHeading());
        t.update();
    }
}
