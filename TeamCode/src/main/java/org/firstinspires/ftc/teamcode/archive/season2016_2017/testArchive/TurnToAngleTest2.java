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

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robots.jorge.Jorge;

// Created on 4/10/2017 at 9:46 PM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode

@Autonomous(name = "TurnToAngleTest2", group = "Linear Opmode")
@Disabled
public class TurnToAngleTest2 extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private Jorge jorge;
    private ModernRoboticsI2cGyro gyro;

    enum Direction {
        RIGHT, LEFT
    }

    @Override
    public void runOpMode() throws InterruptedException {

        jorge = new Jorge(this);

        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        gyro.calibrate();

        while (gyro.isCalibrating()) {
            telemetry.addData("Status", "Gyro is calibrating, don't move!"); telemetry.update();
        }

        jorge.drive.setAllModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData("Status", "Initialized"); telemetry.update();
        waitForStart();
        runtime.reset();

        jorge.drive.setAllModes(DcMotor.RunMode.RUN_TO_POSITION);
        // TODO: Modify EncodedRobotDrive to have setTargetPosition for sides
        jorge.drive.setTargetPositions(
                new String[]{ "lf", "lb",  "rf",  "rb" },
                new int[]   { 2000, 2000, -2000, -2000 },
                new double[]{   .8,   .8,   -.8,   -.8 }
        );

        //rotateToAngle(360, Direction.RIGHT);

        // Done.
        while (opModeIsActive()) {telemetry.addData("Status", "Done. Gyro: " + gyro.getHeading()); telemetry.update();}
    }

    private void rotateToAngle ( int angle, Direction dir ) {

        int encd = (int) Math.round(angle*((double)1000/47));

        if ( dir == Direction.RIGHT ) {
            jorge.drive.setTargetPositions(
                    new String[]{ "lf", "lb",  "rf",  "rb" },
                    new int[]   { encd, encd, -encd, -encd },
                    new double[]{   .8,   .8,   -.8,   -.8 }
            );
        } else {
            jorge.drive.setTargetPositions(
                    new String[]{  "lf",  "lb", "rf", "rb" },
                    new int[]   { -encd, -encd, encd, encd },
                    new double[]{   -.8,   -.8,   .8,   .8 }
            );
        }

        while ( !compareTarget(jorge.drive.getAllPositions(), jorge.drive.getAllTargetPositions()) ) { if (!opModeIsActive()) return; }

        jorge.drive.stopAll();

        //if (gyro.getHeading() != angle) {
        //    rotateToAngle(angle, dir, gyro.getHeading(), encd);
        //}

    }

    private void rotateToAngle ( int angle, Direction dir, int ratioAngle, int ratioEncoders ) {

        int encd = (int) Math.round(angle*((double)ratioEncoders/ratioAngle));

        if ( dir == Direction.RIGHT ) {
            jorge.drive.setTargetPositions(
                    new String[]{ "lf", "lb",  "rf",  "rb" },
                    new int[]   { encd, encd, -encd, -encd },
                    new double[]{   .8,   .8,   -.8,   -.8 }
            );
        } else {
            jorge.drive.setTargetPositions(
                    new String[]{  "lf",  "lb", "rf", "rb" },
                    new int[]   { -encd, -encd, encd, encd },
                    new double[]{   .8,   .8,   -.8,   -.8 }
            );
        }

        while ( !compareTarget(jorge.drive.getAllPositions(), jorge.drive.getAllTargetPositions()) ) { if (!opModeIsActive()) return; }

        jorge.drive.stopAll();

    }

    private boolean compareTarget ( int[] current, int[] target ) {
        if (current.length != target.length) return false;
        boolean Return = true;
        for ( int i = 0; i < current.length; i++ ) {
            if (current[i] > target[i]+2 || current[i] < target[i]-2) {
                Return = false; break;
            }
        }
        return Return;
    }

}
