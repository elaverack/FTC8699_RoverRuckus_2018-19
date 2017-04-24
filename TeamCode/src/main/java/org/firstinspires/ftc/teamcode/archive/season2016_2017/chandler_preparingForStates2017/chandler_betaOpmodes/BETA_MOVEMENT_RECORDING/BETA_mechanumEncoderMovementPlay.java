package org.firstinspires.ftc.teamcode.archive.season2016_2017.chandler_preparingForStates2017.chandler_betaOpmodes.BETA_MOVEMENT_RECORDING;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.Arrays;
import java.util.Vector;

// Created on 1/18/2017 at 4:42 PM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode.archive.season2016_2017.chandler_preparingForStates2017.chandler_betaOpmodes

@Autonomous(name = "MECH-EN-Play-Test", group = "BETA")
@Disabled
public class BETA_mechanumEncoderMovementPlay extends OpMode {
    //Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftMotor1;
    private DcMotor leftMotor2;
    private DcMotor rightMotor1;
    private DcMotor rightMotor2;

    private int LF = 0;
    private int LB = 0;
    private int RF = 0;
    private int RB = 0;

    private double[][] values;
    private File loadFile;
    int counter = 0;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        leftMotor1  = hardwareMap.dcMotor.get("lf");
        leftMotor2  = hardwareMap.dcMotor.get("lb");
        rightMotor1 = hardwareMap.dcMotor.get("rf");
        rightMotor2 = hardwareMap.dcMotor.get("rb");

        leftMotor1.setDirection(DcMotor.Direction.FORWARD);
        leftMotor2.setDirection(DcMotor.Direction.FORWARD);
        rightMotor1.setDirection(DcMotor.Direction.REVERSE);
        rightMotor2.setDirection(DcMotor.Direction.REVERSE);

        loadFile= new File(Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOWNLOADS) + "/Servo Test Saves", "ENSAVE.txt");
        if (!loadFile.exists()) {
            //Double check if the file exists in order to avoid errors later.
            telemetry.addData("Error", "Couldn't find load file");
        }

    }

    @Override
    public void init_loop() {



    }

    @Override
    public void start() {

        Vector load = new Vector();
        String[] e;
        try {
            FileInputStream f = new FileInputStream(loadFile);
            BufferedReader reader = new BufferedReader(new InputStreamReader(f));

            String line = reader.readLine();
            while(line != null){
                load.addElement(line);
                line = reader.readLine();
            }

        } catch (FileNotFoundException er) {

        } catch (IOException er) {

        } finally {
            e = Arrays.copyOf(load.toArray(), load.toArray().length, String[].class);
        }

        values = new double[e.length-2][5];

        for (int i = 1; i < e.length-1; i++) {
            //double TIME = Double.parseDouble(e[i].substring(e[i].indexOf('{')+1, e[i].indexOf('}')));
            //double POWER = Double.parseDouble(e[i].substring(e[i].lastIndexOf('{')+1, e[i].lastIndexOf('}')));
            double[] timeAndPositions = processString(e[i]);

            values[i-1] = timeAndPositions;
        }

        leftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runtime.reset();

    }

    @Override
    public void loop() {
        telemetry.addData("Status", "Running: " + runtime.toString());

        if (counter < values.length) {
            if (runtime.seconds() > values[counter][0]) {
                //servo.setPosition(values[counter][1]);
                leftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                LF = (int)values[counter][1];
                LB = (int)values[counter][2];
                RF = (int)values[counter][3];
                RB = (int)values[counter][4];
                leftMotor1.setTargetPosition(LF);
                leftMotor2.setTargetPosition(LB);
                rightMotor1.setTargetPosition(RF);
                rightMotor2.setTargetPosition(RB);
                counter++;
            }

            telemetry.addData("Values Length", values.length);
            telemetry.addData("Counter", counter);
            telemetry.addData("Position", "Right Front: " + RF);
            telemetry.addData("Position", "Right Back: " + RB);
            telemetry.addData("Position", "Left Front: " + LF);
            telemetry.addData("Position", "Left Back: " + LB);

        } else {
            telemetry.addData("counter>values.length", "Finished.");
            leftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftMotor1.setPower(0);
            leftMotor2.setPower(0);
            rightMotor1.setPower(0);
            rightMotor2.setPower(0);
        }

    }

    @Override
    public void stop() {

        leftMotor1.setPower(0);
        leftMotor2.setPower(0);
        rightMotor1.setPower(0);
        rightMotor2.setPower(0);

    }

    private double[] processString(String string) {
        double TIME = Double.parseDouble(string.substring(string.indexOf('{')+1, string.indexOf('}')));
        String newString = string.substring(string.indexOf('}')+1);
        int LF = Integer.parseInt(newString.substring(newString.indexOf('{')+1, newString.indexOf('}')));
        newString = newString.substring(newString.indexOf('}')+1);
        int LB = Integer.parseInt(newString.substring(newString.indexOf('{')+1, newString.indexOf('}')));
        newString = newString.substring(newString.indexOf('}')+1);
        int RF = Integer.parseInt(newString.substring(newString.indexOf('{')+1, newString.indexOf('}')));
        newString = newString.substring(newString.indexOf('}')+1);
        int RB = Integer.parseInt(newString.substring(newString.indexOf('{')+1, newString.indexOf('}')));
        return new double[]{TIME, LF, LB, RF, RB};
    }

}
