package org.firstinspires.ftc.teamcode.swerve;
import java.util.HashMap;
import java.lang.Double;
public class voltageToAngleConstants {
    // (Angle, Voltage)
    HashMap <Double,Double> fl = new HashMap<>();
    HashMap <Double,Double> fr = new HashMap<>();
    HashMap <Double,Double> bl = new HashMap<>();
    HashMap <Double,Double> br = new HashMap<>();

    public void voltageToAngleConstants() {

        fl.put(65.0, 2.000);
        fl.put(40.0, 2.503);
        fl.put(17.0, 3.00);
        fl.put(0.0, 0.0);
        fl.put(-24.0, 0.500);
        fl.put(-27.5, 0.585);
        fl.put(-47.0, 1.000);
        fl.put(-74.0, 1.625);


        // Servo Mode: R 1.04, P 0.393, L 3.07

        fr.put(0.0, 0.113);
        fr.put(45.0, 2.311);
        fr.put(-10.0, 0.286);
        fr.put(-20.0, 0.500);
        fr.put(-31.0, 0.741);
        fr.put(-52.0,1.2);
        fr.put(-79.0, 1.7);
        fr.put(-87.0, 1.976);
        fr.put(-90.0, 2.026);

        bl.put(0.0, 0.0);
        bl.put(0.0, 0.0);
        bl.put(0.0, 0.0);
        bl.put(0.0, 0.0);
        bl.put(0.0, 0.0);
        bl.put(0.0, 0.0);

        br.put(0.0, 0.0);
        br.put(0.0, 0.0);
        br.put(0.0, 0.0);
        br.put(0.0, 0.0);
        br.put(0.0, 0.0);
        br.put(0.0, 0.0);


    }

    public double angleFL(double voltage) {
        double raw = (voltage * -45) - 0.738431;
        if (raw < 0) {
            return raw;
        } else {
            return (voltage * -45) + 160.73;
        }
    }

    public double angleFR(double voltage) {
        double raw = (voltage * -45) + 3.93225;
        if (raw < 3.93225) {
            return raw;
        } else {
            return (voltage * -45) + 148.995;
        }

    }




}
