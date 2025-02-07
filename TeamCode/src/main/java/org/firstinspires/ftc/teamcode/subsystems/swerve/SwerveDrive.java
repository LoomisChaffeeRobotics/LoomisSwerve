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
    public TrajectoryVelocityConstraint velocityConstraint = new MinVelocityConstraint(Arrays.asList(
            new AngularVelocityConstraint(Math.toRadians(180)),
            new MecanumVelocityConstraint(20, 11)
    )); // i made this up
    public TrajectoryAccelerationConstraint accelerationConstraint = new ProfileAccelerationConstraint(15);
    public double maxAngAccel = Math.PI;
    public double maxAngVel = Math.PI;
    OpMode OM;
    Gamepad gamepad; // perhaps a set power method would be better here?
    public final double DIST_MULT = 4/3; // Actually traveled/desired dist
    public static double aP = 0.02;
    public static double aI = 0.01;
    public static double aD = 0.0015;
    SlewRateLimiter[] wheelLimiters = new SlewRateLimiter[4];
    boolean dontMove;
    boolean reverse;
    public SwerveDriveOdometry odo;
    public ElapsedTime timer;
    public SwerveModuleState[] states = new SwerveModuleState[4];
    SwerveDriveKinematics kinematics; // FTCLIB FOR AUTO
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
    ArrayList<Pair<Double,Double>> targetADPairList = new ArrayList<>(4); // key = mag, value = direction
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
            wheelLimiters[i] = new SlewRateLimiter(1/2.625);
        }
        imu = hw.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                //CHANGE THESE ONCE ORIENTATION IS KNOW
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
        imu.resetYaw();
        // Adjust the orientation parameter`s to match your robot


        for (int i = 0; i < driveMotors.length; i++) {
            driveSpeeds[i] = getVelocity(driveMotors[i]);
            lastDriveEncoders[i] = driveMotors[i].getCurrentPosition();
        }
        targetADPairList.ensureCapacity(4);
        targetADPairList.add(new Pair<>(0.0, 0.0));
        targetADPairList.add(new Pair<>(0.0, 0.0));
        targetADPairList.add(new Pair<>(0.0, 0.0));
        targetADPairList.add(new Pair<>(0.0, 0.0));
        kinematics = new SwerveDriveKinematics(
                new Translation2d((6.5), (5.5)),
                new Translation2d((6.5),(-5.5)),
                new Translation2d((-4.5), (5.5)),
                new Translation2d((-4.5), (-5.5)));
        odo = new SwerveDriveOdometry(kinematics, new Rotation2d(imu.getRobotYawPitchRollAngles().getYaw()), new Pose2d(startFTCLibX, startFTCLibY, new Rotation2d(startHeadingRads)));

        timer = new ElapsedTime();
        nowPose = new Pose2d();

        for (int i = 0; i < driveMotors.length; i++) {
            if (Math.signum(driveSpeeds[i]) == -1) {
                states[i] = new SwerveModuleState(-1 * driveSpeeds[i], new Rotation2d(Math.toRadians((angles[i] + 180) % 360)));
            } else {
                states[i] = new SwerveModuleState(driveSpeeds[i], new Rotation2d(Math.toRadians(angles[i])));
            }

        }
        // init the other devices

    }
    public void resetIMU() { imu.resetYaw();}
    public void init_loop () {
        angleGetter.init_loop();
        timer.reset();
    }
    public void updateMagnitudeDirectionPair(double x, double y, double rx, double currentAngle, int m) {
            // Get the combined vector from gamepad inputs
        double[] componentsVector = vectorGetter.getCombinedVector(
                x,
                y,
                -rx,
                gamepadToVectors.Wheel.values()[m]
        );

        // Calculate magnitude and direction
        double magnitude = Math.sqrt(Math.pow(componentsVector[0], 2) + Math.pow(componentsVector[1], 2));

        double direction;
        if (Math.abs(magnitude) > 0.1) {
            direction = Math.toDegrees(Math.atan2(componentsVector[0], componentsVector[1])) + 180;
            direction = angleFixer.calculateOptimalAngle(currentAngle, direction, m); // perhaps this is where fixes needed?
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
        return inchesPerTick * ticksPerSecond * DIST_MULT; // quick fix for now , x 100 but core issue is unk
    }
    public void loopFC(double theta, double x, double y, double rx) {
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
            // refresh target numbers based on current pose
            // calculate current velocity
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
            double magnitude = targetADPairList.get(i).first; // can be + or -
            double speedOutput;
            // possibly: check if positive or negative, if newly positive or negative, use a diff limiter
            if (magnitude != 0) {
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
        /* I think this isn't updating the odo's pose, but it's doing smth else such
        that calculating things where all are 0 deg etc somehow makes it think diagonally?

        * check the angle fixing code for being wrong - States takes radians
        * unknown angle range?
        * something I did when typing htat comment last night fixed it

        The update method of the odometry class updates the robot position on the field.
        The update method takes in the gyro angle of the robot, along with an array of
        SwerveModulePosition objects. It is important that the order in which you pass the
        SwerveModulePosition objects is the same as the order in which you created the kinematics
         object.
         */
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
