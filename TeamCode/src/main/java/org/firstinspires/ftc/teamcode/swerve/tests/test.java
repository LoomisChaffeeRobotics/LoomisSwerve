package org.firstinspires.ftc.teamcode.swerve.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.swerve.voltageToAngleConstants;

import java.io.IOException;

@TeleOp
public class test extends OpMode {
    voltageToAngleConstants angles;
    String[] encoders = {
            "fl_encoder",
            "fr_encoder",
            "bl_encoder",
            "br_encoder"
    };
    @Override
    public void init() {
//        try {
        angles = new voltageToAngleConstants(this, hardwareMap, encoders);
//        } catch (IOException e) {
//            throw new RuntimeException(e);
//        }
    }
    public void init_loop() {
        angles.init_loop();
    }
    @Override
    public void loop() {
        angles.loop();
    }
}
