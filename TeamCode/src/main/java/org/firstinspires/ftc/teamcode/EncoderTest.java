package org.firstinspires.ftc.teamcode;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
    FtcDashboard dashboard;
    AnalogInput encoder;
    Telemetry telemetry2;
    //TODO: Implement servo PID, attach to detect spin of small pulley shaft
    public static double kP = 0.001;
    public static double kI = 0.0005;
    public static double kD = 0;
    double targetAngle;
    private final double degreesPerVolt = 360/3.3;
    double currentVoltage;
    public void runOpMode() {
        dashboard = FtcDashboard.getInstance();
        encoder = hardwareMap.get(AnalogInput.class, "sensor"); // AS5600
        telemetry2 = dashboard.getTelemetry(); // this is for the dashboard, normal telemetry is for DS
        targetAngle = encoder.getVoltage()*degreesPerVolt;
        waitForStart();
        while (!isStopRequested()) {
            currentVoltage = encoder.getVoltage();

            if (gamepad1.x) {
                targetAngle += 45;
            } else if (gamepad1.y) {
                targetAngle -= 45;
            }




            telemetry.addData("Voltage", encoder.getVoltage());
            telemetry2.addData("Voltage", encoder.getVoltage());
            telemetry2.update();
            telemetry.update();
        }
    }
}
