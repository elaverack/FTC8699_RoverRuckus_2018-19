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

import org.firstinspires.ftc.teamcode.robots.jorge.AutonomousJorge;

// Created on 4/22/2017 at 3:58 PM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode

@Autonomous(name = "MoveToPointTest", group = "Linear Opmode")
@Disabled
public class MoveToPointTest extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private AutonomousJorge jorge;

    private final double
            IN_DISTANCE_PER_1680 = 19.125;

    private ModernRoboticsI2cGyro gyro;

    @Override
    public void runOpMode() throws InterruptedException {

        jorge = new AutonomousJorge(this, false);

        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        gyro.calibrate();

        while (gyro.isCalibrating()) {
            telemetry.addData("Status", "Gyro is calibrating, don't move!"); telemetry.update();
        }

        jorge.drive.setAllModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();

        goFeetWithEncoders(1, .8);
        rotateToAngle(65, TurnToAngleTest.Direction.RIGHT);
        goFeetWithEncoders((float)(34.25/12), .8);

        // Done.
        while (opModeIsActive()) { telemetry.addData("Status", "Done. Run Time: " + runtime.toString()); telemetry.update(); }
    }

    private int calcEncodersForFeet (float feet) {
        float distance = feet * 12;
        return (int) Math.round((distance / IN_DISTANCE_PER_1680) * 1680);
    }

    private void goFeetWithEncoders (float feet, double speed) {
        jorge.drive.setAllModes(DcMotor.RunMode.RUN_TO_POSITION);
        jorge.drive.setAllTargetPositions(calcEncodersForFeet(feet), speed);

        while (!compareTarget(jorge.drive.getAllPositions(), jorge.drive.getAllTargetPositions())) { if (!opModeIsActive()) return; }
        jorge.stop();

        jorge.drive.setAllModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void rotateToAngle ( int angle, TurnToAngleTest.Direction dir ) {
        jorge.drive.setAllModes(DcMotor.RunMode.RUN_TO_POSITION);

        int encd = (int) Math.round(angle*((double)1000/47.5));

        if ( dir == TurnToAngleTest.Direction.RIGHT ) {
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

        /*if (gyro.getHeading() != angle) {
            rotateToAngle(angle, dir, gyro.getHeading(), encd);
        }*/

        jorge.drive.setAllModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    private void rotateToAngle (int angle, TurnToAngleTest.Direction dir, int ratioAngle, int ratioEncoders ) {

        int encd = (int) Math.round(angle*((double)ratioEncoders/ratioAngle));

        if ( dir == TurnToAngleTest.Direction.RIGHT ) {
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

        jorge.drive.setAllModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    private boolean compareTarget ( int[] current, int[] target ) {
        if (current.length != target.length) return false;
        boolean Return = true;
        for ( int i = 0; i < current.length; i++ ) {
            if (current[i] > target[i]+4 || current[i] < target[i]-4) {
                Return = false; break;
            }
        }
        return Return;
    }
}
