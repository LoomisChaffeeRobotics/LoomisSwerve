package org.firstinspires.ftc.teamcode.swerve;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class SwerveDrive {
//    OptimalAngleCalculator calculator;
    gamepadToVectors gamepadConverter;
    voltageToAngleConstants angleConverter;
    public static String[] encoderNames = {
            "fl_encoder",
            "fr_encoder",
            "bl_encoder",
            "br_encoder"
    };
    public SwerveDrive (double W, double L, double maxTranslationalSpeed, double maxRotSpeed) {

    }

    public void setDrivePower(double x, double y, double rx) {

    }
    public void init() {
//        voltageToAngleConstants = new voltageToAngleConstants(opMode, hw, encoderNames);
    }
    public void init_loop() {
//        angleConverter.init_loop();
    }
    public void loop() {
//        angleConverter.loop();
    }
    public void controlAngle() {

    }

}
