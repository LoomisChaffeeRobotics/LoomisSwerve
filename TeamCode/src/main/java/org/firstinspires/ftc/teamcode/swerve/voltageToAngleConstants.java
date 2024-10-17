package org.firstinspires.ftc.teamcode.swerve;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.HashMap;
import java.lang.Double;
import android.os.Environment;

import com.opencsv.CSVReader;
import com.opencsv.CSVWriter;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.apache.commons.lang3.ArrayUtils;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.Writer;
import java.util.List;
import java.util.Map;
import java.util.Set;

public class voltageToAngleConstants {
    // (Angle, Voltage)
    /* TODO: it's a problem that rotations --> small pulley, but stored csv file is in big pulley measurements
        !! Combine both Big pulley angles and small pulley ang/rotations there, and parse it after !!
     */
    double smallToBigPulley = 0.3947368; // small:big times 360 deg -- how much big revolves per rotation of small pulley
//    double degreesPerVolt = 43.0100675418;
//    double[] startingOffset;
    int[] rotations; // full small pulley rotations, added to how much degrees of current rotation
    AnalogInput[] encoders; // list of objects, comes from user getting hardware map inputs
    String logFilePath = String.format("%s/FIRST/data/wheelAngles.csv", Environment.getExternalStorageDirectory().getAbsolutePath());
    FileWriter fileWriter = new FileWriter(logFilePath, false);
    CSVWriter csvWriter = new CSVWriter(fileWriter);
    FileReader fileReader = new FileReader(logFilePath);
    CSVReader csvReader = new CSVReader(fileReader);
    double[] voltages; // these are from the analog inputs
    double[] sm; // small pulley angle values NO ROTATIONS
    double[] angle; // big pulley angle values
    double[] lastVoltage; // for checking for wraparounds, and which direction
    double[] lastAngle; // last big pulley angle values what is this for??
    String[] lastAngStringsWriting; // last angle, but as strings
    String[] smallAngString;
    String[] smallRotString;
    String[] valuesReading = null; // used for reading at the beginning of opmodes
    OpMode opMode; // for telemetry when done reading
    public voltageToAngleConstants(OpMode opMode, HardwareMap hw, String[] encoderNames) throws IOException {
        this.opMode = opMode;
        for (int i = 0; i < encoderNames.length; i++) {
            encoders[i] = hw.get(AnalogInput.class, encoderNames[0]);
        }

    }

    public void init_loop() {
        // Supposed to check for the last value in the csv file for most recent rotations
        // TODO: check this very much
        // Also, what if we just rewrote over and over on the same line? how?
        try {
            String[] line = csvReader.readNext(); // I want this to stop happening once the else
            if (line != null) {
                valuesReading = line;
            }
            // this is supposed to go line by line replacing valuesReading with new line content
            // when it ends, valueReading should be the last values of opMode
            // basically, can't move the wheels unpowered because bad
        } catch (IOException e) {
            // go through values reading to split off the three things and assign them appropriately
            angle = Arrays.stream(Arrays.copyOfRange(valuesReading,0,4)).mapToDouble(Double::parseDouble).toArray();
            lastAngle = angle;
            rotations = Arrays.stream(Arrays.copyOfRange(valuesReading,4,8)).mapToInt(Integer::parseInt).toArray();
            sm = Arrays.stream(Arrays.copyOfRange(valuesReading,8,12)).mapToDouble(Double::parseDouble).toArray();
            opMode.telemetry.addLine("Done Reading");
            opMode.telemetry.addData("Big Angles", angle);
            opMode.telemetry.addData("Small rotations", rotations);
            opMode.telemetry.addData("Small Angles", sm);
            opMode.telemetry.update();
        }
    }
    Map<Double,Double> original = new HashMap<Double, Double>() {{
        put(0.0, 204.0);
        put(0.228, 180.0);
        put(0.777, 135.0);
        put(1.047, 90.0);
        put(1.457, 45.0);
        put(1.827, 0.0);
        put(1.828, 360.0);
        put(2.271, 315.0);
        put(2.686, 270.0);
        put(3.147, 225.0);
        put(3.307, 204.0);
    }};
    Map<Double,Double> fr = new HashMap<Double, Double>() {{

    }};
    Map<Double,Double> bl = new HashMap<Double, Double>() {{

    }};
    Map<Double,Double> br = new HashMap<Double, Double>() {{

    }};
    List<Map<Double, Double>> modulesTable = new ArrayList<Map<Double, Double>>() {{
        add(0, original);
        add(1, fr);
        add(2, bl);
        add(3, br);
    }}; // list of maps lol

    public void loop() {
        String[] writingFinal;
        for (int m = 0; m < modulesTable.size(); m++) {
            voltages[m] = encoders[m].getVoltage();
            smallPulleyAngleAccumulator(voltages[m], m);
            bigPulleyCalculator(m);
            lastAngle[m] = angle[m];
            lastAngStringsWriting[m] = Double.toString(lastAngle[m]);
            smallAngString[m] = Double.toString(sm[m]);
            smallRotString[m] = Integer.toString(rotations[m]);
        }
        // go through for each module, get the newest voltage, update angle measurement, update last angles
        writingFinal = ArrayUtils.addAll(lastAngStringsWriting,smallRotString);
        writingFinal = ArrayUtils.addAll(writingFinal, smallAngString);
        // it won't let me do all 3 args at a time?? idk why but i think this works to do the same thing
        csvWriter.writeNext(writingFinal);
        // write to the csv everything
        // everything being big pulley angle > small pulley full rotations > small pulley angle pose
    }
    public double voltsToAngle(double voltage, int module) {
        double out;
        Map<Double, Double> targetTable = modulesTable.get(module);
        Set<Double> keySet = targetTable.keySet();
        Collection<Double> valueSet = targetTable.values();
        // both of the below are doubles but it won't let me
        Object[] values = valueSet.toArray();
        Object[] keys = keySet.toArray();
        for (int i = 0; i < targetTable.size(); i++) {
            if (voltage <= Double.parseDouble(keys[i].toString()) && voltage >= Double.parseDouble(keys[i+1].toString())) {
                // this might have trouble with voltages that are equal?
                //TODO: Double check this logic esp in if
                double y1, x1, y2, x2, m;
                x2 = Double.parseDouble(keys[i+1].toString());
                x1 = Double.parseDouble(keys[i].toString());
                y1 = Double.parseDouble(values[i].toString());
                y2 = Double.parseDouble(values[i+1].toString());
                m = (y2-y1)/(x2-x1);
                // point slope of the line b/w the points its between
                out = (m*(voltage-x1))+y1;
                // input x as the voltage into the formula
                return out;
            }
        }
        return 0;
    }
    public void smallPulleyAngleAccumulator(double inputVoltage, int module) {
        // TODO: needs checking code for sm in the beginning from new files
        sm[module] += voltsToAngle(inputVoltage, module) - lastAngle[module];
        // checks if that accumulation made it go over or under
        if (sm[module] >= 360) {
            rotations[module]++;
        } else if (sm[module] <= 0) {
            rotations[module]--;
        }

        // this updates the small pulley things
    }
    public double bigPulleyCalculator(int m) {
        double degreesRaw = sm[m] + rotations[m]; 
        return degreesRaw * smallToBigPulley; // calculates the angle :D now put it in the optimal angle calculator
    }
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
