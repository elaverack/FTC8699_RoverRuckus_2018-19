package org.firstinspires.ftc.teamcode.robotHandlers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.robots.AutonomusJorge;

/**
 * Created by Chandler on 4/12/2017.
 */

public class JorgeAutonomusFunctions {

    private static final int
            LINE_THRESHOLD = 6501;

    public static void GO_TO_WHITE_LINE (AutonomusJorge jorge, AutonomusJorge.COLOR_SENSOR sensor) {
        if (!jorge.opMode.opModeIsActive()) return;
        jorge.drive.setAllPowers(.1);
        while (jorge.colorSensors.colorTemp(sensor.port) < LINE_THRESHOLD) {if (!jorge.opMode.opModeIsActive()) return;}
        jorge.drive.stopAll();
    }

    public static void STRAIGHTEN_ON_WHITE_LINE (AutonomusJorge jorge, AutonomusJorge.COLOR_SENSOR primarySensor, AutonomusJorge.COLOR_SENSOR auxSensor) {
        if (!jorge.opMode.opModeIsActive()) return;
        //while (colorSensors.colorTemp(port) > 5601) {
        //    telemetry.addData("Status", "Waiting..."); telemetry.update();
        //}
        jorge.drive.setPowers(new String[]{"lf", "rf"}, new double[]{-.2, .2});
        while (jorge.colorSensors.colorTemp(primarySensor.port) < LINE_THRESHOLD) {
            if (!jorge.opMode.opModeIsActive()) {jorge.drive.stopAll(); return;}
            /*if (jorge.colorSensors.colorTemp(auxSensor.port) < LINE_THRESHOLD)
            {jorge.drive.setPowers(new String[]{"lf", "rb","lb", "rf"}, new double[]{.2, .2, -.2, -.2});}
            else {jorge.drive.setPowers(new String[]{"lf", "rb", "lb", "rf"}, new double[]{-.2, 0, 0, .2});}*/
            //for (int i = 0; i < NUMBER_OF_SENSORS; i++) {
            //    int[] CRGB = colorSensors.getCRGB(i);
            //    //telemetry.addData("", "Values for sensor " + (i + 1));
            //    telemetry.addData("Clear" + i, CRGB[0]);
            //    telemetry.addData("Red" + i, CRGB[1]);
            //    telemetry.addData("Green" + i, CRGB[2]);
            //    telemetry.addData("Blue" + i, CRGB[3]);
            //    telemetry.addData("Color Temp" + i, colorSensors.colorTemp(i));
            //}
            //telemetry.update();
        }
        jorge.drive.stopAll();
        if (!jorge.opMode.opModeIsActive()) return;
        jorge.drive.setPowers(new String[]{"lf", "rf", "lb", "rb"}, new double[]{-.2, .2, -.18, .18});
        while (jorge.colorSensors.colorTemp(primarySensor.port) < LINE_THRESHOLD && jorge.colorSensors.colorTemp(auxSensor.port) < LINE_THRESHOLD) {
            if (!jorge.opMode.opModeIsActive()) {jorge.drive.stopAll(); return;}
        }
        jorge.drive.stopAll();
    }

}
