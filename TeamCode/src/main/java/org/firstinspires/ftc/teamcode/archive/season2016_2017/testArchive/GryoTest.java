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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robotHandlers.RobotConfig;
import org.firstinspires.ftc.teamcode.robotHandlers.RobotHandler;
import org.firstinspires.ftc.teamcode.robotHandlers.StandardRobotDrive;

// Created on 3/9/2017 at 8:48 AM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode

@TeleOp(name = "GryoTest", group = "Iterative Opmode")
@Disabled
public class GryoTest extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private RobotHandler robot;
    private ModernRoboticsI2cGyro gyro;

    //Code to run ONCE when the driver hits INIT
    @Override
    public void init() {

        robot = new RobotHandler(new RobotConfig(new StandardRobotDrive(this.hardwareMap))) {
            @Override
            public void drive() {

                float straight = -gamepad1.right_stick_y;
                float strafe = gamepad1.right_stick_x;
                float rotate = gamepad1.left_stick_x;
                float slow = gamepad1.right_trigger;

                float powerRF = straight;
                float powerRB = straight;
                float powerLF = straight;
                float powerLB = straight;
                powerRF -= strafe;
                powerRB += strafe;
                powerLF += strafe;
                powerLB -= strafe;
                powerRF -= rotate;
                powerRB -= rotate;
                powerLF += rotate;
                powerLB += rotate;

                if (powerRF > 1) {powerRF = 1;} else if (powerRF < -1) {powerRF = -1;}
                if (powerRB > 1) {powerRB = 1;} else if (powerRB < -1) {powerRB = -1;}
                if (powerLF > 1) {powerLF = 1;} else if (powerLF < -1) {powerLF = -1;}
                if (powerLB > 1) {powerLB = 1;} else if (powerLB < -1) {powerLB = -1;}

                if (slow > 0.25) {
                    powerRF /= 4 * slow;
                    powerRB /= 4 * slow;
                    powerLF /= 4 * slow;
                    powerLB /= 4 * slow;
                }

                StandardRobotDrive drive = robot.getRobotDrive();
                drive.setPowers(new String[]{"rf", "rb", "lf", "lb"}, new double[]{powerRF, powerRB, powerLF, powerLB});

            }
        };

        robot.getRobotDrive().setSideDirections(
                new StandardRobotDrive.SIDE[]{StandardRobotDrive.SIDE.RIGHT, StandardRobotDrive.SIDE.LEFT},
                new DcMotorSimple.Direction[]{DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD}
        );

        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");

        gyro.calibrate();

        telemetry.addData("Status", "Initialized");

    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {runtime.reset();}

    @Override
    public void loop() {

        robot.drive();

        telemetry.addData("Status", "Running: " + runtime.toString());
        telemetry.addData("", "Gyro Headings");
        telemetry.addData("Heading", gyro.getHeading());
        telemetry.addData("Heading Mode", gyro.getHeadingMode());
        telemetry.addData("Integrating", gyro.getIntegratedZValue());

    }

    @Override
    public void stop() {robot.stop();}

}
