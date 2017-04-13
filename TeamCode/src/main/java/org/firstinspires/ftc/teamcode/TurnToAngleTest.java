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
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robotHandlers.StandardRobotDrive;
import org.firstinspires.ftc.teamcode.robots.Jorge;

// Created on 4/10/2017 at 9:46 PM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode

@Autonomous(name = "TurnToAngleTest", group = "Linear Opmode")
//@Disabled
public class TurnToAngleTest extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private Jorge jorge;
    private final int
            DESIRED_ANGLE = 90;
    private final double
            MAX_SPEED = 1,
            MIN_SPEED = .2;
    private ModernRoboticsI2cGyro gyro;

    @Override
    public void runOpMode() throws InterruptedException {

        jorge = new Jorge(this);

        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        gyro.calibrate();

        while (gyro.isCalibrating()) {
            telemetry.addData("Status", "Gyro is calibrating, don't move!"); telemetry.update();
        }

        telemetry.addData("Status", "Initialized"); telemetry.update();
        waitForStart();
        runtime.reset();

        turnToAngle(DESIRED_ANGLE);

        jorge.stop();
        // Done.
        while (opModeIsActive()) {telemetry.addData("Status", "Done. Gyro: " + gyro.getHeading()); telemetry.update();}
    }

    private void turnToAngle (int angle) {
        if (!opModeIsActive()) return;
        int remainingAngle = angle - gyro.getHeading();
        int direction = 1;
        if (remainingAngle < 0) direction = -1;
        double motorSpeed = calcMotorSpeed(remainingAngle*(direction));
        jorge.drive.setSidePower(StandardRobotDrive.SIDE.LEFT, (motorSpeed)*(direction));
        jorge.drive.setSidePower(StandardRobotDrive.SIDE.RIGHT, (motorSpeed)*(-direction));
        if (direction == -1) {
            while (gyro.getHeading() != angle && gyro.getHeading() > angle) {
                if (!opModeIsActive()) return;
                remainingAngle = angle - gyro.getHeading(); motorSpeed = calcMotorSpeed(remainingAngle*(direction));
                jorge.drive.setSidePower(StandardRobotDrive.SIDE.LEFT, (motorSpeed)*(direction));
                jorge.drive.setSidePower(StandardRobotDrive.SIDE.RIGHT, (motorSpeed)*(-direction));
                telemetry.addData("Angle", gyro.getHeading()); telemetry.update();
            }
        } else {
            while (gyro.getHeading() != angle && gyro.getHeading() < angle) {
                if (!opModeIsActive()) return;
                remainingAngle = angle - gyro.getHeading(); motorSpeed = calcMotorSpeed(remainingAngle*(direction));
                jorge.drive.setSidePower(StandardRobotDrive.SIDE.LEFT, (motorSpeed)*(direction));
                jorge.drive.setSidePower(StandardRobotDrive.SIDE.RIGHT, (motorSpeed)*(-direction));
                telemetry.addData("Angle", gyro.getHeading()); telemetry.update();
            }
        }
        while (gyro.getHeading() > angle + 2) {
            if (!opModeIsActive()) return;
            jorge.drive.setSidePower(StandardRobotDrive.SIDE.LEFT, (-.2));
            jorge.drive.setSidePower(StandardRobotDrive.SIDE.RIGHT, (.2));
        }
        while (gyro.getHeading() < angle - 2) {
            if (!opModeIsActive()) return;
            jorge.drive.setSidePower(StandardRobotDrive.SIDE.LEFT, (.2));
            jorge.drive.setSidePower(StandardRobotDrive.SIDE.RIGHT, (-.2));
        }
        jorge.stop();
    }

    private double calcMotorSpeed (int remainingAngle) {
        if (remainingAngle == 0) return 0;
        double Return = ((3.8714911E-6)*(remainingAngle^3))+((-7.660761E-4)*(remainingAngle^2))+((.0550231775)*(remainingAngle))-(.5691622217);
        return Range.clip(Return, MIN_SPEED, MAX_SPEED);
    }

}
