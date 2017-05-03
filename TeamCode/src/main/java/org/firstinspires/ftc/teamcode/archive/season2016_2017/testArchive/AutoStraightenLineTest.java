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
package org.firstinspires.ftc.teamcode.archive.season2016_2017.testArchive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robotHandlers.MultiplexedColorSensors;
import org.firstinspires.ftc.teamcode.robotHandlers.StandardRobotDrive;

// Created on 3/2/2017 at 8:10 PM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode

@Autonomous(name = "OldAutoStraightenLineTest", group = "Linear Opmode")
@Disabled
public class AutoStraightenLineTest extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private StandardRobotDrive drive;
    private MultiplexedColorSensors colorSensors;
    private final int NUMBER_OF_SENSORS = 2;

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new StandardRobotDrive(hardwareMap);
        drive.setSideDirection(StandardRobotDrive.SIDE.LEFT, DcMotorSimple.Direction.FORWARD);
        drive.setSideDirection(StandardRobotDrive.SIDE.RIGHT, DcMotorSimple.Direction.REVERSE);

        colorSensors = new MultiplexedColorSensors(this.hardwareMap, "mux", "ada", NUMBER_OF_SENSORS, MultiplexedColorSensors.ATIME.FASTEST, MultiplexedColorSensors.GAIN._16X);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        straightenOnWhiteLine(1, 0);

        telemetry.clear();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Straightened on white line.");
            telemetry.addData("c0", "" + colorSensors.colorTemp(0));
            telemetry.addData("c1", "" + colorSensors.colorTemp(1));
            telemetry.addData("c2", "" + colorSensors.colorTemp(2));
            telemetry.update();
        }
    }

    private void straightenOnWhiteLine (int port, int auxPort) {
        if (port == auxPort) return;
        final int WHITE_MIN = 6000;
        //drive.setSidePower(StandardRobotDrive.SIDE.RIGHT, .1);
        //drive.setSidePower(StandardRobotDrive.SIDE.LEFT, -.1);
        //while (colorSensors.colorTemp(port) > 5601) {
        //    telemetry.addData("Status", "Waiting..."); telemetry.update();
        //}
        //drive.setPowers(new String[]{"lf", "rf"}, new double[]{-.2, .2});
        drive.setSidePowers(
                new StandardRobotDrive.SIDE[]{StandardRobotDrive.SIDE.LEFT, StandardRobotDrive.SIDE.RIGHT},
                                 new double[]{                        -.15,                           .15});
        while (colorSensors.colorTemp(port) < WHITE_MIN) {
            if (!opModeIsActive()) break;

            if (colorSensors.colorTemp(auxPort) < 6275) {
                drive.setPowers(new String[]{"lf", "rf", "lb", "rb"}, new double[]{-.15, .15, -.08, .08});
            } else {
                drive.setSidePowers(
                        new StandardRobotDrive.SIDE[]{StandardRobotDrive.SIDE.LEFT, StandardRobotDrive.SIDE.RIGHT},
                                         new double[]{                        -.15,                           .15});
            }

            telemetry.addData("Status", "Straightening...");
            telemetry.addData("c0", "" + colorSensors.colorTemp(0));
            telemetry.addData("c1", "" + colorSensors.colorTemp(1));
            telemetry.addData("c2", "" + colorSensors.colorTemp(2));
            telemetry.update();
        }
        if (colorSensors.colorTemp(auxPort) < 6275) {
            drive.setPowers(new String[]{"lf", "rf", "lb", "rb"}, new double[]{-.15, .15, 0, 0});
            while (colorSensors.colorTemp(auxPort) < 6275) {
                if (!opModeIsActive()) break;
                telemetry.addData("Status", "Correcting turn...");
                telemetry.addData("c0", "" + colorSensors.colorTemp(0));
                telemetry.addData("c1", "" + colorSensors.colorTemp(1));
                telemetry.addData("c2", "" + colorSensors.colorTemp(2));
                telemetry.update();
        }}
        drive.stopAll();
    }
}
