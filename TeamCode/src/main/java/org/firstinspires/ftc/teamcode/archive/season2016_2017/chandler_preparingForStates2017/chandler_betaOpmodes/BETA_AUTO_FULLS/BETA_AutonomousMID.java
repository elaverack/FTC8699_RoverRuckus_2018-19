/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode.archive.season2016_2017.chandler_preparingForStates2017.chandler_betaOpmodes.BETA_AUTO_FULLS;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.archive.season2016_2017.robotCoreFunctions.MultiplexColorSensor;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.Arrays;
import java.util.Vector;

// Created on 1/28/2017 at 11:09 PM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode.archive.season2016_2017.chandler_preparingForStates2017.chandler_betaOpmodes

@Autonomous(name = "BETA_AutonomousTEST", group = "BETA")
@Disabled
public class BETA_AutonomousMID extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftMotor1;
    private DcMotor leftMotor2;
    private DcMotor rightMotor1;
    private DcMotor rightMotor2;

    private Servo beacon_presser;
    private Servo shooter;

    private MultiplexColorSensor colorSensors;

    final private String MUX_NAME = "mux";
    final private String ADA_NAME = "ada";
    final private int[] PORTS = {0, 1, 2, 3, 4, 5, 6};

    private int[] lastCRGB1 = new int[4];
    private int[] lastCRGB2 = new int[4];
    private int[] crgb1;
    private int[] crgb2;
    private int[] crgb3;

    private OpticalDistanceSensor odsSensorLeft;
    private OpticalDistanceSensor odsSensorRight;

    private int[] startCRGB1;
    private int[] startCRGB2;

    // TODO: Create necessary variables for camera functions

    @Override
    public void runOpMode() {
        int[] leftBeaconColors = new int[2];
        int[] rightBeaconColors = new int[2];

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftMotor1  = hardwareMap.dcMotor.get("lf");
        leftMotor2  = hardwareMap.dcMotor.get("lb");
        rightMotor1 = hardwareMap.dcMotor.get("rf");
        rightMotor2 = hardwareMap.dcMotor.get("rb");
        leftMotor1.setDirection(DcMotor.Direction.FORWARD);
        leftMotor2.setDirection(DcMotor.Direction.FORWARD);
        rightMotor1.setDirection(DcMotor.Direction.REVERSE);
        rightMotor2.setDirection(DcMotor.Direction.REVERSE);
        beacon_presser = hardwareMap.servo.get("beacon");
        shooter = hardwareMap.servo.get("shoot");
        odsSensorLeft = hardwareMap.opticalDistanceSensor.get("left");
        odsSensorRight = hardwareMap.opticalDistanceSensor.get("right");

        colorSensors = new MultiplexColorSensor(hardwareMap, MUX_NAME, ADA_NAME, PORTS, 1, MultiplexColorSensor.GAIN_16X);
        colorSensors.startPolling();

        startCRGB1 = colorSensors.getCRGB(0);
        startCRGB2 = colorSensors.getCRGB(1);

        // TODO: Write any initialization for camera functions

        waitForStart();
        runtime.reset();

        beacon_presser.setPosition(0);
        shooter.setPosition(.84);
        sleep(250);
        shooter.setPosition(.39);
        sleep(250);
        //load(Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOWNLOADS) + "/Servo Test Saves", "MOVE_TO_BEACON.txt");
        goToWhiteLine(0);
        alignForBeacon();
        //boolean isRedLeft = isRedLeft();
        /*if (isRedLeft) {
            leftBeaconColors[0] = Color.RED;
            leftBeaconColors[1] = Color.BLUE;
        } else {
            leftBeaconColors[0] = Color.BLUE;
            leftBeaconColors[1] = Color.RED;
        }*/
        telemetry.clearAll();

        runtime.reset();
            while (!overWhiteLine(startCRGB2, colorSensors.getCRGB(1)) && opModeIsActive() && runtime.milliseconds() < 1000) {
                leftMotor1.setPower(0);
                leftMotor2.setPower(0);
                rightMotor1.setPower(.10);
                rightMotor2.setPower(.10);
            }

        runtime.reset();
        while (!overWhiteLine(startCRGB2, colorSensors.getCRGB(1)) && opModeIsActive() && runtime.milliseconds() < 1000) {
            leftMotor1.setPower(0);
            leftMotor2.setPower(0);
            rightMotor1.setPower(.10);
            rightMotor2.setPower(.10);
        }

        while (opModeIsActive()) {
            telemetry.addData("Is red left?", isRedLeft(1));
            telemetry.addData("Right", odsSensorRight.getLightDetected());
            telemetry.addData("Left", odsSensorLeft.getLightDetected());
            telemetry.addData("STClear", startCRGB2[0]);
            telemetry.addData("STRed  ", startCRGB2[1]);
            telemetry.addData("STGreen", startCRGB2[2]);
            telemetry.addData("STBlue ", startCRGB2[3]);
            telemetry.addData("Clear", colorSensors.getCRGB(1)[0]);
            telemetry.addData("Red  ", colorSensors.getCRGB(1)[1]);
            telemetry.addData("Green", colorSensors.getCRGB(1)[2]);
            telemetry.addData("Blue ", colorSensors.getCRGB(1)[3]);
            telemetry.addData("WHITE? ", overWhiteLine(startCRGB2, colorSensors.getCRGB(1)));
            telemetry.update();
        }

        // TODO: Write the rest of this

    }

    private boolean overWhiteLine(int[] startCRGB, int[] crgb) {
        double startAVG = (startCRGB[0]+startCRGB[1]+startCRGB[2]+startCRGB[3])/4;
        double nowAVG = (crgb[0]+crgb[1]+crgb[2]+crgb[3])/4;
        return !(nowAVG > startAVG-2000 && nowAVG < startAVG+2000);
    }

    private String colorToString(int color) {
        switch (color) {
            case Color.BLUE:
                return "BLUE";
            case Color.RED:
                return "RED";
            default:
                return "ERROR";
        }
    }

    // 1 for right, -1 for left
    private boolean isRedLeft(int lookingAt) {
        int Case = 0;
        switch (checkColor((crgb3 = colorSensors.getCRGB(2)))) {
            case 1:
                Case = 1;
                break;
            case -1:
                Case = -1;
                break;
            case 0:
                Case = 0;
                break;
        }

        if (Case == 0) {
            return false;
        } else {
            return !(Case == lookingAt);
        }
    }

    private void align () {
        /* TODO: Write align code. Something like:
            Check optical distance sensors
            Straighten robot
         */
    }

    private void alignForBeacon () {

        telemetry.clearAll();

        while ((odsSensorLeft.getLightDetected() < 0.001 || odsSensorRight.getLightDetected() == 0) && opModeIsActive()) {
            leftMotor1.setPower(.10);
            leftMotor2.setPower(.10);
            rightMotor1.setPower(.10);
            rightMotor2.setPower(.10);
            telemetry.addData("Status", "Going straight...");
            telemetry.addData("Right", odsSensorRight.getLightDetected());
            telemetry.addData("Left", odsSensorLeft.getLightDetected());
            telemetry.update();
        }

        telemetry.clearAll();

        /*while ((odsSensorLeft.getLightDetected() == 0 || odsSensorRight.getLightDetected() == 0) && opModeIsActive()) {
            if (odsSensorLeft.getLightDetected() == 0) {

                leftMotor1.setPower(.10);
                leftMotor2.setPower(.10);
                rightMotor1.setPower(0);
                rightMotor2.setPower(0);
                telemetry.addData("Status", "Turning left...");
                telemetry.addData("Right", odsSensorRight.getLightDetected());
                telemetry.addData("Left", odsSensorLeft.getLightDetected());
                telemetry.update();

            } else if (odsSensorRight.getLightDetected() == 0) {

                leftMotor1.setPower(0);
                leftMotor2.setPower(0);
                rightMotor1.setPower(.10);
                rightMotor2.setPower(.10);
                telemetry.addData("Status", "Turing right...");
                telemetry.addData("Right", odsSensorRight.getLightDetected());
                telemetry.addData("Left", odsSensorLeft.getLightDetected());
                telemetry.update();

            }
        }*/

        telemetry.clearAll();

        if (odsSensorLeft.getLightDetected() > 0.005 || odsSensorRight.getLightDetected() > 0.005) {
            while ((odsSensorLeft.getLightDetected() > 0.005 || odsSensorRight.getLightDetected() > 0.005) && opModeIsActive()) {
                leftMotor1.setPower(odsSensorLeft.getLightDetected() > 0.005 ? -0.1 : 0);
                leftMotor2.setPower(odsSensorLeft.getLightDetected() > 0.005 ? -0.1 : 0);
                rightMotor1.setPower(odsSensorRight.getLightDetected() > 0.005 ? -0.1 : 0);
                rightMotor2.setPower(odsSensorRight.getLightDetected() > 0.005 ? -0.1 : 0);
                telemetry.addData("Status", "Backing up...");
                telemetry.addData("Right", odsSensorRight.getLightDetected());
                telemetry.addData("Left", odsSensorLeft.getLightDetected());
                telemetry.update();
            }

            telemetry.clearAll();

            while (odsSensorLeft.getLightDetected() > odsSensorRight.getLightDetected() && opModeIsActive()) {
                leftMotor1.setPower(-.1);
                leftMotor2.setPower(-.1);
                rightMotor1.setPower(0);
                rightMotor2.setPower(0);
                telemetry.addData("Status", "Backing up (left)...");
                telemetry.addData("Right", odsSensorRight.getLightDetected());
                telemetry.addData("Left", odsSensorLeft.getLightDetected());
                telemetry.update();
            }
        }

        telemetry.clearAll();

        /*while (checkColor((crgb3 = colorSensors.getCRGB(2))) == 0 && opModeIsActive()) {
            leftMotor1.setPower(.1);
            leftMotor2.setPower(.1);
            rightMotor1.setPower(.1);
            rightMotor2.setPower(.1);
            telemetry.addData("Status", "Going straight for beacon...");
            telemetry.addData("Right", odsSensorRight.getLightDetected());
            telemetry.addData("Left", odsSensorLeft.getLightDetected());
            telemetry.addData("Clear", crgb3[0]);
            telemetry.addData("Red  ", crgb3[1]);
            telemetry.addData("Green", crgb3[2]);
            telemetry.addData("Blue ", crgb3[3]);
            telemetry.addData("Color? ", checkColor(crgb3));
            telemetry.update();
            sleep(250);
        }*/

        leftMotor1.setPower(0);
        leftMotor2.setPower(0);
        rightMotor1.setPower(0);
        rightMotor2.setPower(0);

        telemetry.clearAll();
    }

    private int checkColor(float blue, float red) {
        if (red - blue > 299) {
            return 1;
        } else if (blue - red > 299) {
            return -1;
        } else {
            return 0;
        }
    }

    // 1 for red, -1 for blue
    private int checkColor(int crgb[]) {
        if (
                crgb[0] > 15000 &&
                        crgb[1] > 10000 &&
                        crgb[1] > crgb[3]*1.5
                ) {
            return 1;
        } else if (
                crgb[0] > 19000 &&
                        crgb[3] > 10000 &&
                        crgb[3] > crgb[1]*1.5) {
            return -1;
        } else {
            return 0;
        }
    }

    private String colorsToString (int[] colors) {
        String Return = "";
        for (int i = 0; i < colors.length; i++) {
            if (i < colors.length-1) {
                Return += colors[i] + ", ";
            } else {
                Return += colors[i];
            }
        }
        return Return;
    }


    /**
     * @PARAM direction Either 0 (left) or 1 (right).
     */
    private void goToWhiteLine(int direction) {
        /*
        OLD CODE
        crgb1 = colorSensors.getCRGB(0);
        crgb2 = colorSensors.getCRGB(1);
        lastCRGB1 = crgb1;
        lastCRGB2 = crgb2;
        telemetry.clearAll();

        while (opModeIsActive() && !compareCRGB(lastCRGB1, crgb1, 0)) {

            crgb1 = colorSensors.getCRGB(0);

            leftMotor1.setPower(.25);
            leftMotor2.setPower(-.25);
            rightMotor1.setPower(-.25);
            rightMotor2.setPower(.25);
            telemetry.addData("Compare", compareCRGB(lastCRGB1, crgb1, 0));
            lastCRGB1 = crgb1;

        }
        */
        crgb1 = colorSensors.getCRGB(0);
        crgb2 = colorSensors.getCRGB(1);
        lastCRGB1 = crgb1;
        lastCRGB2 = crgb2;
        telemetry.clearAll();
        boolean Start = true;
        boolean Break = false;
        boolean Compare = false;
        int crgb[] = new int[4];
        int lastcrgb[] = new int[4];
        int port;
        double power1 = 0;
        double power2 = 0;
        if (direction == 0) {
            crgb = crgb2;
            lastcrgb = lastCRGB2;
            port = 1;
            power1 = -.2;
            power2 = .2;
        } else if (direction == 1) {
            crgb = crgb1;
            lastcrgb = lastCRGB1;
            port = 0;
            power1 = .2;
            power2 = -.2;
        } else {
            Break = true;
            port = 0;
            crgb = null;
            lastcrgb = null;
        }

        while (opModeIsActive() && !Break) {

            if (!Start) {
                crgb = colorSensors.getCRGB(port);
                Compare = compareCRGB(lastcrgb, crgb, 850);
            } else {
                crgb = colorSensors.getCRGB(port);
                lastcrgb = crgb;
                startCRGB1 = colorSensors.getCRGB(0);
                startCRGB2 = colorSensors.getCRGB(1);
                Start = false;
            }

            leftMotor1.setPower(power1);
            leftMotor2.setPower(power2);
            rightMotor1.setPower(power2);
            rightMotor2.setPower(power1);
            telemetry.addData("Compare", Compare);
            telemetry.addData("Last", colorsToString(lastcrgb));
            telemetry.addData("Now", colorsToString(crgb));
            telemetry.addData("Break", Break);
            telemetry.update();

            if (compareCRGB(lastcrgb, crgb) > 100 && !Compare) {
                lastcrgb = crgb;
            }

            Break = Compare;

        }

        leftMotor1.setPower(0);
        leftMotor2.setPower(0);
        rightMotor1.setPower(0);
        rightMotor2.setPower(0);
        telemetry.clearAll();

    }

    private boolean compareCRGB(int[] pre, int[] now, int threshold) {
        final int rDif = Math.abs(now[1]-pre[1]);
        final int gDif = Math.abs(now[2]-pre[2]);
        final int bDif = Math.abs(now[3]-pre[3]);
        if (rDif+gDif+bDif > threshold) {
            return true;
        } else {
            return false;
        }
    }

    private int compareCRGB(int[] pre, int[] now) {
        final int rDif = Math.abs(now[1]-pre[1]);
        final int gDif = Math.abs(now[2]-pre[2]);
        final int bDif = Math.abs(now[3]-pre[3]);
        return rDif+gDif+bDif;
    }

    private void load (String path, String fileName) {
        double[][] values;
        File loadFile;
        int counter = 0;
        double LF = 0;
        double LB = 0;
        double RF = 0;
        double RB = 0;

        loadFile= new File(path, fileName);
        if (!loadFile.exists()) {
            //Double check if the file exists in order to avoid errors later.
            telemetry.addData("Error", "Couldn't find load file");
        }



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
            double[] timeAndPowers = processString(e[i]);

            values[i-1] = timeAndPowers;
        }

        runtime.reset();

        while (counter < values.length) {
            if (runtime.seconds() > values[counter][0]) {
                //servo.setPosition(values[counter][1]);
                LF = values[counter][1];
                LB = values[counter][2];
                RF = values[counter][3];
                RB = values[counter][4];
                counter++;
            }

            leftMotor1.setPower(LF);
            leftMotor2.setPower(LB);
            rightMotor1.setPower(RF);
            rightMotor2.setPower(RB);

            telemetry.addData("Values Length", values.length);
            telemetry.addData("Counter", counter);
            telemetry.addData("Power", "Right Front: " + RF);
            telemetry.addData("Power", "Right Back: " + RB);
            telemetry.addData("Power", "Left Front: " + LF);
            telemetry.addData("Power", "Left Back: " + LB);

        }

        telemetry.clearAll();

    }

    private double[] processString(String string) {
        double TIME = Double.parseDouble(string.substring(string.indexOf('{')+1, string.indexOf('}')));
        String newString = string.substring(string.indexOf('}')+1);
        double LF = Double.parseDouble(newString.substring(newString.indexOf('{')+1, newString.indexOf('}')));
        newString = newString.substring(newString.indexOf('}')+1);
        double LB = Double.parseDouble(newString.substring(newString.indexOf('{')+1, newString.indexOf('}')));
        newString = newString.substring(newString.indexOf('}')+1);
        double RF = Double.parseDouble(newString.substring(newString.indexOf('{')+1, newString.indexOf('}')));
        newString = newString.substring(newString.indexOf('}')+1);
        double RB = Double.parseDouble(newString.substring(newString.indexOf('{')+1, newString.indexOf('}')));
        return new double[]{TIME, LF, LB, RF, RB};
    }
}
