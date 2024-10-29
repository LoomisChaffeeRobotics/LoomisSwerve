package org.firstinspires.ftc.teamcode.swerve.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.swerve.vectorsToAngleAndDrive;

public class driveTest extends OpMode {
    public static double P = 3e-2;
    public static double I = 0;
    public static double D = 0.00075;
    public static double dP = 0;
    public static double dI = 0;
    public static double dD = 0;
    vectorsToAngleAndDrive SwerveDrive;
    String[] encoderNames = {
            "fl_encoder",
            "fr_encoder",
            "bl_encoder",
            "br_encoder"
    };
    String[] driveNames = {
            "fl_motor",
            "fr_motor",
            "bl_motor",
            "br_motor"
    };
    String[] angleNames = {
            "fl_angle",
            "fr_angle",
            "bl_angle",
            "br_angle"
    };

    @Override
    public void init() {
        SwerveDrive = new vectorsToAngleAndDrive(
                18, 18, 12, 12,
                this, gamepad1, hardwareMap,
                encoderNames, driveNames, angleNames, P, I, D, dP, dI, dD);
    }
    @Override
    public void init_loop () {
        SwerveDrive.init_loop();
    }
    @Override
    public void loop() {
        SwerveDrive.loop();
    }
}
