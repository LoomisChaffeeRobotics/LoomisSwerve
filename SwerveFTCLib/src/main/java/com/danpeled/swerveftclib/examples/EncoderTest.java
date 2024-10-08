package com.danpeled.swerveftclib.examples;

import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
@Config
public class EncoderTest extends LinearOpMode {
    Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381); // Fix later for actual values in meters(?)
    Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
    Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
    Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);
    SwerveDriveKinematics kinematics = new SwerveDriveKinematics
            (
                    m_frontLeftLocation, m_frontRightLocation,
                    m_backLeftLocation, m_backRightLocation
            );
    // ftclib usage? idk man
    CRServo fl_angle;
    FtcDashboard dashboard;
    AnalogInput fl_encoder;
    DcMotor fl_motor;
    IMU imu;
    Telemetry telemetry2;
    //TODO: Implement servo PID, attach to detect spin of small pulley shaft
    public static double kP = 3e-2;
    public static double kI = 0;
    public static double kD = 0.00075;
    public static double targetAngle;
    double currentAngle;
    double adjustedAngle;
    private final double degreesPerVolt = 360/3.3;
    double currentVoltage;
    double offset;
    double error;
    double lastError;
    double integral;
    double derivative;
    double output;
    ElapsedTime timer;
    public void runOpMode() {
        dashboard = FtcDashboard.getInstance();
        fl_motor = hardwareMap.get(DcMotor.class, "fl_drive");
        fl_angle = hardwareMap.get(CRServo.class, "fl_angle");
        fl_encoder = hardwareMap.get(AnalogInput.class, "fl_encoder"); // AS5600
        telemetry2 = dashboard.getTelemetry(); // this is for the dashboard, normal telemetry is for DS
        timer = new ElapsedTime();
        waitForStart();
        while (!isStopRequested()) {
            currentVoltage = fl_encoder.getVoltage();
            currentAngle = currentVoltage * degreesPerVolt;
            adjustedAngle = currentAngle + offset;

            if (Math.abs(targetAngle-currentAngle) > 90 && Math.signum(targetAngle - currentAngle) == 1 ) {

            }

            // note that it currently cycles between 0 and 3 or so volts, there's some code to fix this but i forgot ask dylan
            error = targetAngle - adjustedAngle;
            derivative = (error - lastError) / timer.seconds();
            integral += error * timer.seconds();
            output = (kP * error) + (kI * integral) + (kD * derivative);

            fl_angle.setPower(output);
            lastError = error;
            timer.reset();


            if (gamepad1.x) {
                targetAngle += 0.2;
            } else if (gamepad1.y) {
                targetAngle -= 0.2;
            }


            telemetry.addData("Voltage", fl_encoder.getVoltage());
            telemetry2.addData("Voltage", fl_encoder.getVoltage());
            telemetry.addData("TargetAngle", targetAngle);
            telemetry2.addData("TargetAngle", targetAngle);
            telemetry.addData("Angle", currentAngle);
            telemetry2.addData("Angle", currentAngle);
            telemetry.addData("Servo pwer", fl_angle.getPower());
            telemetry2.addData("Servo pwer", fl_angle.getPower());
            telemetry2.update();
            telemetry.update();
        }
    }
}