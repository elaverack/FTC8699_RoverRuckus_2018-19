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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.archive.season2016_2017.robotCoreFunctions.MultiplexColorSensor;

@TeleOp(name="MUX Test", group="BETA")
@Disabled
public class BETA_Multiplexer3TEST extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();

    private Servo beacon_presser;
    private MultiplexColorSensor colorSensors;

    final private String MUX_NAME = "mux";
    final private String ADA_NAME = "ada";
    final private int[] PORTS = {0, 1, 2, 3, 4, 5, 6};
    private boolean seeBeacon = false;
    private boolean beaconRed = false;
    private boolean beaconBlue = false;
    private int[] crgb3;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initiation of motors named in configuration file.
        // Basically gives the code the motor to look for and assigns it to a variable.
        beacon_presser = hardwareMap.servo.get("beacon");
        colorSensors = new MultiplexColorSensor(hardwareMap, MUX_NAME, ADA_NAME, PORTS, 1, MultiplexColorSensor.GAIN_16X);

        // A trial and error experience really.
        // This just sets the motor directions so that they all run correctly according to the code.
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        telemetry.addData("Status", "Running: " + runtime.toString());

        switch (checkColor(colorSensors.getCRGB(2))) {
            case 1:
                seeBeacon = true;
                beaconRed = true;
                beaconBlue = false;
                break;
            case -1:
                seeBeacon = true;
                beaconBlue = true;
                beaconRed = false;
                break;
            case 0:
                seeBeacon = false;
                beaconBlue = false;
                beaconRed = false;
                break;
        }

        // This sets the motors to the calculated powers and then outputs data to the
        //  driver station phone, mainly for troubleshooting purposes.
        beacon_presser.setPosition(gamepad1.left_trigger);
        telemetry.addData("Clear1", colorSensors.getCRGB(0)[0]);
        telemetry.addData("Red1  ", colorSensors.getCRGB(0)[1]);
        telemetry.addData("Green1", colorSensors.getCRGB(0)[2]);
        telemetry.addData("Blue1 ", colorSensors.getCRGB(0)[3]);
        telemetry.addData("Clear2", colorSensors.getCRGB(1)[0]);
        telemetry.addData("Red2  ", colorSensors.getCRGB(1)[1]);
        telemetry.addData("Green2", colorSensors.getCRGB(1)[2]);
        telemetry.addData("Blue2 ", colorSensors.getCRGB(1)[3]);
        telemetry.addData("Clear3", colorSensors.getCRGB(2)[0]);
        telemetry.addData("Red3  ", colorSensors.getCRGB(2)[1]);
        telemetry.addData("Green3", colorSensors.getCRGB(2)[2]);
        telemetry.addData("Blue3 ", colorSensors.getCRGB(2)[3]);
        telemetry.addData("Clear4", colorSensors.getCRGB(3)[0]);
        telemetry.addData("Red4  ", colorSensors.getCRGB(3)[1]);
        telemetry.addData("Green4", colorSensors.getCRGB(3)[2]);
        telemetry.addData("Blue4 ", colorSensors.getCRGB(3)[3]);
        telemetry.addData("Clear5", colorSensors.getCRGB(4)[0]);
        telemetry.addData("Red5  ", colorSensors.getCRGB(4)[1]);
        telemetry.addData("Green5", colorSensors.getCRGB(4)[2]);
        telemetry.addData("Blue5 ", colorSensors.getCRGB(4)[3]);
        telemetry.addData("Clear6", colorSensors.getCRGB(5)[0]);
        telemetry.addData("Red6  ", colorSensors.getCRGB(5)[1]);
        telemetry.addData("Green6", colorSensors.getCRGB(5)[2]);
        telemetry.addData("Blue6 ", colorSensors.getCRGB(5)[3]);
        telemetry.addData("Clear7", colorSensors.getCRGB(6)[0]);
        telemetry.addData("Red7  ", colorSensors.getCRGB(6)[1]);
        telemetry.addData("Green7", colorSensors.getCRGB(6)[2]);
        telemetry.addData("Blue7 ", colorSensors.getCRGB(6)[3]);
        telemetry.addData("Beacon? ", seeBeacon);
        telemetry.addData("Blue? ", beaconBlue);
        telemetry.addData("Red? ", beaconRed);
    }

    @Override
    public void stop() {

    }

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

    private int checkColor(float blue, float red) {
        if (red - blue > 299) {
            return 1;
        } else if (blue - red > 299) {
            return -1;
        } else {
            return 0;
        }
    }

}
