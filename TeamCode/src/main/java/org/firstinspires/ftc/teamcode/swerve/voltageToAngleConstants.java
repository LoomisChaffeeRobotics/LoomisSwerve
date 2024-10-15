package org.firstinspires.ftc.teamcode.swerve;
import java.util.HashMap;
import java.lang.Double;
import android.os.Environment;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.Writer;
import java.util.Map;

public class voltageToAngleConstants {
    // (Angle, Voltage)
    //TODO: Implement tracking code to keep track of direction of motion knowing the gear ratio
    double degBigPerRev = 0.3947368 * 360;
//    double degreesPerVolt = 43.0100675418;
//    double[] startingOffset;
    int i;
    int j;
    int k;
    int l;
    Map<Double,Double> fl = new HashMap<Double, Double>() {{
        put(0.0, 1.827);
        put(90.0, 1.047);
        put(45.0, 1.457);
        put(135.0, 0.777);
        put(180.0, 0.228);
        put(204.0, 0.0);
        put(204.0, 3.307);
        put(225.0, 3.147);
        put(270.0, 2.686);
        put(315.0, 2.271);
        put(360.0, 1.827);
    }};
    Map<Double,Double> fr = new HashMap<Double, Double>() {{

    }};
    Map<Double,Double> bl = new HashMap<Double, Double>() {{

    }};
    Map<Double,Double> br = new HashMap<Double, Double>() {{

    }};

//    public void voltageToAngleConstants() {
//        fl.put(74.0, 1.700);
//        fl.put(65.0, 2.000);
//        fl.put(40.0, 2.503);
//        fl.put(17.0, 3.00);
//        fl.put(0.0, 0.0);
//        fl.put(-24.0, 0.500);
//        fl.put(-27.5, 0.585);
//        fl.put(-47.0, 1.000);
//        fl.put(-74.0, 1.625);
//
//
//        // Servo Mode: R 1.04, P 0.393, L 3.07
//
//        fr.put(0.0, 0.113);
//        fr.put(45.0, 2.311);
//        fr.put(-10.0, 0.286);
//        fr.put(-20.0, 0.500);
//        fr.put(-31.0, 0.741);
//        fr.put(-52.0,1.2);
//        fr.put(-79.0, 1.7);
//        fr.put(-87.0, 1.976);
//        fr.put(-90.0, 2.026);
//
//        bl.put(0.0, 0.0);
//        bl.put(0.0, 0.0);
//        bl.put(0.0, 0.0);
//        bl.put(0.0, 0.0);
//        bl.put(0.0, 0.0);
//        bl.put(0.0, 0.0);
//
//        br.put(0.0, 0.0);
//        br.put(0.0, 0.0);
//        br.put(0.0, 0.0);
//        br.put(0.0, 0.0);
//        br.put(0.0, 0.0);
//        br.put(0.0, 0.0);
//
//
//    }

//    public double angleFL(double voltage) {
//        double raw = (voltage * -45) - 0.738431;
//        if (raw < 0) {
//            return raw;
//        } else {
//            return (voltage * -45) + 160.73;
//        }
//    }
//
//    public double angleFR(double voltage) {
//        double raw = (voltage * -45) + 3.93225;
//        if (raw < 3.93225) {
//            return raw;
//        } else {
//            return (voltage * -45) + 148.995;
//        }
//
//    }
//
//    public double angleBL(double voltage) {
//        double raw = voltage * 0;
//        if (raw < 0) {
//            return raw;
//        } else {
//            return (voltage * -45);
//        }
//    }
//    public double angleBR(double voltage) {
//        double raw = voltage * 0;
//        if (raw < 0) {
//            return raw;
//        } else {
//            return (voltage * -45);
//        }
//    }




}
