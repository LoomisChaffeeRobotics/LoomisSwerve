package org.firstinspires.ftc.teamcode.swerve;

import static org.slf4j.MDC.put;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.HashMap;
import java.lang.Double;
import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.apache.commons.lang3.ArrayUtils;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.Spliterator;

public class voltageToAngleConstants {
    // (Angle, Voltage)
    /* TODO: Something is very wrong with filewriter it's not putting it on the control hub, it's putting it straight in this repository
     */
    double smallToBigPulley = 0.3947368; // small:big times 360 deg -- how much big revolves per rotation of small pulley
//    double degreesPerVolt = 43.0100675418;
//    double[] startingOffset;
    int[] rotations; // full small pulley rotations, added to how much degrees of current rotation
    ArrayList<AnalogInput> encoders = new ArrayList<>(); // list of objects, comes from user getting hardware map inputs
    String logFilePath = String.format("%s/FIRST/wheelAngles.txt", Environment.getExternalStorageDirectory().getAbsolutePath());
//    FileWriter fileWriter = new FileWriter(logFilePath, false);
//    CSVWriter csvWriter = new CSVWriter(fileWriter);
//    FileReader fileReader = new FileReader(logFilePath);
//    CSVReader csvReader = new CSVReader(fileReader);

    File dataLog = AppUtil.getInstance().getSettingsFile(logFilePath);
    double[] voltages; // these are from the analog inputs
    double[] sm; // small pulley angle values NO ROTATIONS
    double[] lastSm; //last small pulley angle for counting up/down rotations
    double[] angle; // big pulley angle values
//    double[] lastVoltage; // for checking for wraparounds, and which direction
//    double[] lastAngle; // last big pulley angle values what is this for??
    String[] lastAngStringsWriting; // last angle, but as strings
    String[] smallAngString;
    String[] smallRotString;
    String[] valuesReading = null; // used for reading at the beginning of opmodes
    String lastLineValue = null;
    String fileDataRaw;
    OpMode opMode; // for telemetry when done reading
    public voltageToAngleConstants(OpMode opMode, HardwareMap hw, String[] encoderNames) {
        this.opMode = opMode;

        for (String encoderName : encoderNames) {
            encoders.add(hw.get(AnalogInput.class, encoderName));
        }


        voltages = new double[encoderNames.length];
        lastSm = new double[encoderNames.length];
//        lastVoltage = new double[encoderNames.length];

        lastAngStringsWriting = new String[encoderNames.length];
        smallAngString = new String[encoderNames.length];
        smallRotString = new String[encoderNames.length];

        fileDataRaw = ReadWriteFile.readFile(dataLog);
    }

    public void init_loop() {
        // Supposed to check for the last value in the csv file for most recent rotations
        // TODO: check this very much
        // Also, what if we just rewrote over and over on the same line? how?
        lastLineValue = fileDataRaw;
        lastLineValue = lastLineValue.replace("[", "");
        lastLineValue = lastLineValue.replace("]", "");
        valuesReading = lastLineValue.split(", ");

        // breaks right here, "NumberFormatException: empty String"
        angle = Arrays.stream(Arrays.copyOfRange(valuesReading, 0, 4)).mapToDouble(Double::parseDouble).toArray();
        rotations = Arrays.stream(Arrays.copyOfRange(valuesReading, 4, 8)).mapToInt(Integer::parseInt).toArray();
        sm = Arrays.stream(Arrays.copyOfRange(valuesReading, 8, 12)).mapToDouble(Double::parseDouble).toArray();
        opMode.telemetry.addLine("Done Reading");
        opMode.telemetry.addData("Big Angles", Arrays.toString(angle));
        opMode.telemetry.addData("Small rotations", Arrays.toString(rotations));
        opMode.telemetry.addData("Small Angles", Arrays.toString(sm));
        opMode.telemetry.update();

//        try {
//
////            String[] line = csvReader.readNext(); // I want this to stop happening once the else
////            if (line != null) {
////                valuesReading = line;
////            }
//
//
//            // this is supposed to go line by line replacing valuesReading with new line content
//            // when it ends, valueReading should be the last values of opMode
//            // basically, can't move the wheels unpowered because bad
//        } catch (IOException e) {
//            // go through values reading to split off the three things and assign them appropriately
//            angle = Arrays.stream(Arrays.copyOfRange(valuesReading,0,4)).mapToDouble(Double::parseDouble).toArray();
//            lastAngle = angle;
//            rotations = Arrays.stream(Arrays.copyOfRange(valuesReading,4,8)).mapToInt(Integer::parseInt).toArray();
//            sm = Arrays.stream(Arrays.copyOfRange(valuesReading,8,12)).mapToDouble(Double::parseDouble).toArray();
//            opMode.telemetry.addLine("Done Reading");
//            opMode.telemetry.addData("Big Angles", angle);
//            opMode.telemetry.addData("Small rotations", rotations);
//            opMode.telemetry.addData("Small Angles", sm);
//            opMode.telemetry.update();
//        }
    }
    Map<Double,Double> bl = new LinkedHashMap<Double, Double>() {{ // Voltage up, degrees down
        put(0.0, 114.0);
        put(0.228, 90.0);
        put(0.777, 45.0);
        put(1.047, 0.0);
        put(1.048, 360.0); //synthetic
        put(1.457, 315.0);
        put(1.892, 270.0);
        put(2.271, 225.0);
        put(2.686, 180.0);
        put(3.147, 135.0);
        put(3.307, 114.0);
    }};
    Map<Double,Double> fr = new LinkedHashMap<Double, Double>() {{ // Voltage up, degrees up
        put(0.0, 342.0);
        put(0.134, 360.0); //synthetic
        put(0.135, 0.0);
        put(0.536, 45.0);
        put(0.918, 90.0);
        put(1.335, 135.0);
        put(1.78, 180.0);
        put(2.181, 225.0);
        put(2.584, 270.0);
        put(3.056, 315.0);
        put(3.307, 342.0);
    }};
    Map<Double,Double> fl = new LinkedHashMap<Double, Double>() {{ // Voltage up, degrees down.
        put(0.0, 199.0);
        put(0.166, 180.0);
        put(0.573, 135.0);
        put(1.037, 90.0);
        put(1.433, 45.0);
        put(1.829, 0.0);
        put(1.830, 360.0); // synthetic
        put(2.256, 315.0);
        put(2.703, 270.0);
        put(3.000, 225.0);
        put(3.307, 199.0);
    }};
    Map<Double,Double> br = new LinkedHashMap<Double, Double>() {{
        put(0.0, 55.0);
        put(0.073, 45.0);
        put(0.482, 0.0);
        put(0.483, 360.0);
        put(0.916, 315.0);
        put(1.367, 270.0);
        put(1.681, 225.0);
        put(2.213, 180.0);
        put(2.529, 135.0);
        put(3.046, 90.0);
        put(3.307, 55.0);
    }};
    List<Map<Double, Double>> modulesTable = new ArrayList<Map<Double, Double>>() {{
        add(0, fl);
        add(1, fr);
        add(2, bl);
        add(3, br);
    }}; // list of maps lol
    public void loop() {
//        ArrayList<Object> writingFinal;
        /*      writing final looks like this
       Big Angle            M1, M2... MN
       Small rotation       ...
       Small Angle          ...
        */
//        List<Triplet<Double,Integer,Double>> tripletList = new ArrayList<>();
        String[] writingFinal;
        for (int m = 0; m < modulesTable.size(); m++) {
            voltages[m] = encoders.get(m).getVoltage();
            smallPulleyAngleAccumulator(voltages[m], m);
//            lastAngle[m] = angle[m];
            updateBigPulleyCalculator(m);
            lastAngStringsWriting[m] = Double.toString(angle[m]);
            smallAngString[m] = Double.toString(sm[m]);
            smallRotString[m] = Integer.toString(rotations[m]);

//            tripletList.add(new Triplet<>(lastAngle[m], rotations[m], sm[m]));

        }
        // go through for each module, get the newest voltage, update angle measurement, update last angles


        writingFinal = ArrayUtils.addAll(lastAngStringsWriting,smallRotString);
        writingFinal = ArrayUtils.addAll(writingFinal, smallAngString);

//        for (int i = 0; i < tripletList.size(); i++) {
//            writingFinal[i] = tripletList.get(i).toString();
//            // turn each triplet into a string and add it to writing final
//        }
        ReadWriteFile.writeFile(dataLog, Arrays.toString(writingFinal));
//        tripletList.clear();
        Arrays.fill(writingFinal, null);
        opMode.telemetry.addData("Big Angles", Arrays.toString(angle));
        opMode.telemetry.addData("Small rotations", Arrays.toString(rotations));
        opMode.telemetry.addData("Small Angles", Arrays.toString(sm));
        opMode.telemetry.update();
        // write to the txt everything
        // everything being big pulley angle > small pulley full rotations > small pulley angle pose
    }
    public double[] getBigPulleyAngles() {
        return angle;
    }
    public double voltsToAngle(double voltage, int module) {
        double out;
        Map<Double, Double> targetTable = modulesTable.get(module);
        Set<Double> keySet = targetTable.keySet();
        Collection<Double> valueSet = targetTable.values();
        // both of the below are doubles but it won't let me
        Object[] values = valueSet.toArray();
        Object[] keys = keySet.toArray();
//        Spliterator<Double> sldkfj;
        if (keySet.contains(voltage)) {
            return targetTable.get(voltage);
        } else {
            for (int i = 0; i < keys.length - 1; i++) {
                if (voltage < Double.parseDouble(keys[i].toString()) ) {

                    double y1, x1, y2, x2, m;
                    x2 = Double.parseDouble(keys[i+1].toString());
                    x1 = Double.parseDouble(keys[i].toString());
                    y1 = Double.parseDouble(values[i].toString());
                    y2 = Double.parseDouble(values[i+1].toString());
                    m = (y2 - y1)/(x2 - x1);
                    // point slope of the line b/w the points its between
                    out = (m * (voltage - x1)) + y1;
                    // input x as the voltage into the formula

                    return out;
                    // somehow, this is between 200 and 520 instead of 0 and 360????
                }
            }
        }
        return 0;
    }
    public void smallPulleyAngleAccumulator(double inputVoltage, int module) {
        // TODO: needs checking code for sm in the beginning from new files
        sm[module] = voltsToAngle(inputVoltage, module);

        double difference = sm[module] - lastSm[module];
        double threshold = 5.0; // Adjust threshold based on testing

// Going from 0 to 360 (wrap-around)
        if (Math.abs(difference) > 180 && sm[module] > lastSm[module] + threshold) {
            rotations[module]--;
        }
// Going from 360 to 0 (wrap-around)
        else if (Math.abs(difference) > 180 && sm[module] < lastSm[module] - threshold) {
            rotations[module]++;
        }
        lastSm[module] = sm[module];


        // this updates the small pulley things
    }
    public void updateBigPulleyCalculator(int m) {
        double degreesRaw = sm[m] + rotations[m]; 
        angle[m] = degreesRaw * smallToBigPulley; // calculates the angle :D now put it in the optimal angle calculator
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
