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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robotHandlers.StandardRobotDrive;

// Created on 3/3/2017 at 8:44 AM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode

@Autonomous(name = "OldAutoPush2Test", group = "Linear Opmode")
@Disabled
public class AutoPush2Test extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private StandardRobotDrive drive;
    private Servo beacon_presser;
    private final double
            POSITION_UP = 0,
            POSITION_DOWN = 1;

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new StandardRobotDrive(hardwareMap);
        drive.setSideDirection(StandardRobotDrive.SIDE.LEFT, DcMotorSimple.Direction.FORWARD);
        drive.setSideDirection(StandardRobotDrive.SIDE.RIGHT, DcMotorSimple.Direction.REVERSE);

        beacon_presser = hardwareMap.servo.get("beacon");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        sleep(500);
        press(2);
        drive.stopAll();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Done pressing beacon.");
            telemetry.update();
        }
    }

    private final double
            BACK_UP_POWER = -.15,
            PRESS_POWER = .1;
    private final long
            PRESS_TIME = 1500,
            BACK_UP_TIME = 250,
            INTERVAL_TIME = 100;
    private void press(int times) throws InterruptedException {
        for (int i = 0; i < times; i++) {
            if (!opModeIsActive()) return;
            drive.setAllPowers(BACK_UP_POWER);
            sleep(BACK_UP_TIME);
            drive.stopAll();
            sleep(INTERVAL_TIME);
            beacon_presser.setPosition(POSITION_DOWN);
            drive.setAllPowers(PRESS_POWER);
            sleep(PRESS_TIME);
            drive.stopAll();
            sleep(INTERVAL_TIME);
            drive.setAllPowers(BACK_UP_POWER);
            sleep(BACK_UP_TIME);
            drive.stopAll();
            sleep(INTERVAL_TIME);
        }
        beacon_presser.setPosition(POSITION_UP);
    }
}
