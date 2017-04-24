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
package org.firstinspires.ftc.teamcode.archive.season2016_2017.chandler_preparingForStates2017.chandler_betaOpmodes.BETA_AUTO_TESTING;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.archive.season2016_2017.robotCoreFunctions.MultiplexColorSensor;

// Created on 2/1/2017 at 5:47 PM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode.archive.season2016_2017.chandler_preparingForStates2017.chandler_betaOpmodes

@TeleOp(name = "TEST5", group = "BETA")
// @Autonomous(...) is the other common choice
@Disabled
public class BETA_White5 extends LinearOpMode {

    private DcMotor leftMotor1;
    private DcMotor leftMotor2;
    private DcMotor rightMotor1;
    private DcMotor rightMotor2;

    private MultiplexColorSensor colorSensors;

    final private String MUX_NAME = "mux";
    final private String ADA_NAME = "ada";
    final private int[] PORTS = {0, 1};

    private int[] lastCRGB1 = new int[4];
    private int[] lastCRGB2 = new int[4];
    private int[] crgb1;
    private int[] crgb2;

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
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

        colorSensors = new MultiplexColorSensor(hardwareMap, MUX_NAME, ADA_NAME, PORTS, 1, MultiplexColorSensor.GAIN_16X);

        waitForStart();
        runtime.reset();

        goToWhiteLine(1);

        while (opModeIsActive()) {
            leftMotor1.setPower(0);
            leftMotor2.setPower(0);
            rightMotor1.setPower(0);
            rightMotor2.setPower(0);
            telemetry.addData("Finished", "");
            telemetry.update();
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

}


