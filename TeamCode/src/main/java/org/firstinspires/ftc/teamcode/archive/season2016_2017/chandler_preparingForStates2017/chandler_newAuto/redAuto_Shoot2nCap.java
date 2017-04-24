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
package org.firstinspires.ftc.teamcode.archive.season2016_2017.chandler_preparingForStates2017.chandler_newAuto;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.Arrays;
import java.util.Vector;

// Created on 2/16/2017 at 8:32 AM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode.archive.season2016_2017.chandler_preparingForStates2017.chandler_newAuto

@Autonomous(name = "RED: 2-S2-C", group = "RED")
// @Autonomous(...) is the other common choice
@Disabled
public class redAuto_Shoot2nCap extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private final String SAVE_DIRECTORY = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOWNLOADS) + "/Servo Test Saves";
    private DcMotor leftMotor1;
    private DcMotor leftMotor2;
    private DcMotor rightMotor1;
    private DcMotor rightMotor2;

    private DcMotor shooter;
    private final int SHOOTER_LOAD = 1150;
    private final int SHOOTER_FIRE = 1680;

    private Servo loader; //TODO: Test and set these values
    private final double LOAD = 0;
    private final double UNLOAD = 0.67;

    @Override
    public void runOpMode() throws InterruptedException {

        leftMotor1  = hardwareMap.dcMotor.get("lf");
        leftMotor2  = hardwareMap.dcMotor.get("lb");
        rightMotor1 = hardwareMap.dcMotor.get("rf");
        rightMotor2 = hardwareMap.dcMotor.get("rb");
        leftMotor1.setDirection(DcMotor.Direction.FORWARD);
        leftMotor2.setDirection(DcMotor.Direction.FORWARD);
        rightMotor1.setDirection(DcMotor.Direction.REVERSE);
        rightMotor2.setDirection(DcMotor.Direction.REVERSE);

        shooter = hardwareMap.dcMotor.get("shoot");
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        loader = hardwareMap.servo.get("load");

        telemetry.addData("Status", "Initialized");
        telemetry.update(); // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        load(SAVE_DIRECTORY, "RED2_GO_TO_SHOOTING_POSITION.txt");
        shoot();
        load(SAVE_DIRECTORY, "RED2_HIT_CAPBALL.txt");

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();


        }
    }

    private boolean shooting = false;
    private boolean set = false;
    private int shot = 0;
    private void shoot () throws InterruptedException {

        while (shot != 2) {

            if (!shooting && shot == 1) {
                loader.setPosition(LOAD);
                sleep(1000);
                loader.setPosition(UNLOAD);
                sleep(1000);
            }

            if (!shooting) {
                shooting = true;
            }

            if (shooting) {
                if (!set) {
                    shooter.setTargetPosition(SHOOTER_FIRE);
                    shooter.setPower(.2);
                    set = true;
                }
                if ((shooter.getCurrentPosition() > SHOOTER_FIRE || shooter.getCurrentPosition() == SHOOTER_FIRE) && set) {
                    shooter.setPower(0);
                    shooter.setTargetPosition(0);
                    shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    shooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    shooting = false;
                    set = false;
                    shot++;
                }
            }

        }

    }

    private void load (String path, String fileName) throws InterruptedException {
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
            // meh
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
