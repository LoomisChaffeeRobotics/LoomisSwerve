package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
@Config
public class EncoderTest extends LinearOpMode {
    FtcDashboard dashboard;
    AnalogInput encoder;
    Telemetry telemetry2;
    //TODO: Implement servo PID, attach to detect spin of small pulley shaft
    public void runOpMode() {
        dashboard = FtcDashboard.getInstance();
        encoder = hardwareMap.get(AnalogInput.class, "sensor"); // AS5600
        telemetry2 = dashboard.getTelemetry(); // this is for the dashboard, normal telemetry is for DS

        waitForStart();
        while (!isStopRequested()) {
            telemetry.addData("Voltage", encoder.getVoltage());
            telemetry2.addData("Voltage", encoder.getVoltage());
            telemetry2.update();
            telemetry.update();
        }
    }
}
