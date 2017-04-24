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

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.archive.season2016_2017.robotCoreFunctions.MultiplexColorSensor;

// Created on 1/29/2017 at 1:28 PM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode.archive.season2016_2017.chandler_preparingForStates2017.chandler_betaOpmodes

@TeleOp(name = "BETA_Multiplexer_Testing", group = "BETA")
// @Autonomous(...) is the other common choice
@Disabled
public class BETA_Multiplexer extends OpMode {
    //Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    MultiplexColorSensor colorSensors;

    final private String MUX_NAME = "mux";
    final private String ADA_NAME = "ada";
    final private int[] PORTS = {0, 1};

    private int[] lastCRGB1 = new int[4];
    private int[] lastCRGB2 = new int[4];

    //Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        // eg: Set the drive motor directions:
        // Reverse the motor that runs backwards when connected directly to the battery
        // Set to REVERSE if using AndyMark motors
        // Set to FORWARD if using AndyMark motors

        colorSensors = new MultiplexColorSensor(hardwareMap, MUX_NAME, ADA_NAME, PORTS, 1, MultiplexColorSensor.GAIN_16X);

    }

    //Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
    @Override
    public void init_loop() {



    }

    //Code to run ONCE when the driver hits PLAY
    @Override
    public void start() {

        runtime.reset();

    }

    //Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {
        int[] crgb1 = colorSensors.getCRGB(0);
        int[] crgb2 = colorSensors.getCRGB(1);
        telemetry.addData("Status", "Running: " + runtime.toString());
        telemetry.addData("CRGB, Sensor 1", colorsToString(crgb1));
        telemetry.addData("CRGB, Sensor 2", colorsToString(crgb2));



        telemetry.addData("Test 1", checkForTape(crgb1));
        telemetry.addData("Test 2", checkForTape(crgb2));
        telemetry.addData("Test 3", compareCRGB(lastCRGB1, crgb1, 500));
        telemetry.addData("Test 4", compareCRGB(lastCRGB2, crgb2, 500));

        // eg: Run wheels in tank mode (note: The joystick goes negative when pushed forwards)

        lastCRGB1 = crgb1;
        lastCRGB2 = crgb2;

    }

    //Code to run ONCE after the driver hits STOP
    @Override
    public void stop() {

        // eg: Set all motor powers to 0


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

    private String colorIntToString(int color) {
        switch (color) {
            case Color.BLACK:
                return "BLACK";
            case Color.BLUE:
                return "BLUE";
            case Color.CYAN:
                return "CYAN";
            case Color.DKGRAY:
                return "DARK GREY";
            case Color.GRAY:
                return "GREY";
            case Color.GREEN:
                return "GREEN";
            case Color.LTGRAY:
                return "LIGHT GRAY";
            case Color.MAGENTA:
                return "MAGENTA";
            case Color.RED:
                return "RED";
            case Color.TRANSPARENT:
                return "TRANSPARENT";
            case Color.WHITE:
                return "WHITE";
            case Color.YELLOW:
                return "YELLOW";
            default:
                return "ERROR";
        }
    }

    private boolean checkForTape(int[] crgb) {
        final int RED_MIN = 8000;
        final int GREEN_MIN = 7000;
        final int BLUE_MIN = 5000;
        if (crgb[1] > RED_MIN && crgb[2] > GREEN_MIN && crgb[3] > BLUE_MIN) {
            return true;
        } else {
            return false;
        }
    }

    private boolean compareCRGB(int[] pre, int[] now, int threshold) {
        final int rDif = now[1]-pre[1];
        final int gDif = now[2]-pre[2];
        final int bDif = now[3]-pre[3];
        if (rDif+gDif+bDif > threshold) {
            return true;
        } else {
            return false;
        }
    }

}
