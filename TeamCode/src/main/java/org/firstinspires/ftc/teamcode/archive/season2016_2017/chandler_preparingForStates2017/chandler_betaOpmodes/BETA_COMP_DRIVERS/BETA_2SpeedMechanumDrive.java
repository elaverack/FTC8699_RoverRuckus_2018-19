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
package org.firstinspires.ftc.teamcode.archive.season2016_2017.chandler_preparingForStates2017.chandler_betaOpmodes.BETA_COMP_DRIVERS;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Competition-2Speed", group="Iterative Opmode")
@Disabled
public class BETA_2SpeedMechanumDrive extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();

    // Motors of the robot \/
    private DcMotor leftMotor1;
    private DcMotor leftMotor2;
    private DcMotor rightMotor1;
    private DcMotor rightMotor2;

    private Servo beacon_presser;

    // Joystick and motor power values to mess with during loop()
    private float rightStickY;
    private float rightStickX;
    private float powerRF;
    private float powerRB;
    private float powerLF;
    private float powerLB;
    private float leftStickX;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initiation of motors named in configuration file.
        // Basically gives the code the motor to look for and assigns it to a variable.
        leftMotor1  = hardwareMap.dcMotor.get("lf");
        leftMotor2  = hardwareMap.dcMotor.get("lb");
        rightMotor1 = hardwareMap.dcMotor.get("rf");
        rightMotor2 = hardwareMap.dcMotor.get("rb");
        beacon_presser = hardwareMap.servo.get("beacon");

        // A trial and error experience really.
        // This just sets the motor directions so that they all run correctly according to the code.
        leftMotor1.setDirection(DcMotor.Direction.FORWARD);
        leftMotor2.setDirection(DcMotor.Direction.FORWARD);
        rightMotor1.setDirection(DcMotor.Direction.REVERSE);
        rightMotor2.setDirection(DcMotor.Direction.REVERSE);
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

        // This is the calculation for the motor power values.
        // It first sets controller data to variables, and then respectively applies those
        //  variables to each motor power.
        rightStickY = -gamepad1.right_stick_y;
        rightStickX = gamepad1.right_stick_x;
        leftStickX = gamepad1.left_stick_x;
        powerRF = rightStickY;
        powerRB = rightStickY;
        powerLF = rightStickY;
        powerLB = rightStickY;
        powerRF -= rightStickX;
        powerRB += rightStickX;
        powerLF += rightStickX;
        powerLB -= rightStickX;
        powerRF -= leftStickX;
        powerRB -= leftStickX;
        powerLF += leftStickX;
        powerLB += leftStickX;

        // Could be simplified honestly.
        // Basically cuts the motor powers if they are above one after calculations
        //  -- since motor powers can't be greater than one or less than negative one.
        if (powerRF > 1) {powerRF = 1;} else if (powerRF < -1) {powerRF = -1;}
        if (powerRB > 1) {powerRB = 1;} else if (powerRB < -1) {powerRB = -1;}
        if (powerLF > 1) {powerLF = 1;} else if (powerLF < -1) {powerLF = -1;}
        if (powerLB > 1) {powerLB = 1;} else if (powerLB < -1) {powerLB = -1;}

        if (gamepad1.right_trigger > 0.25) {
            powerRF /= 2;
            powerRB /= 2;
            powerLF /= 2;
            powerLB /= 2;
        }

        // This sets the motors to the calculated powers and then outputs data to the
        //  driver station phone, mainly for troubleshooting purposes.
        leftMotor1.setPower(powerLF);
        leftMotor2.setPower(powerLB);
        rightMotor1.setPower(powerRF);
        rightMotor2.setPower(powerRB);
        beacon_presser.setPosition(gamepad1.left_trigger);
        telemetry.addData("Sticks", "X: " + gamepad1.right_stick_x);
        telemetry.addData("Sticks", "Y: " + -gamepad1.right_stick_y);
        telemetry.addData("Power", "Right Front: " + powerRF);
        telemetry.addData("Power", "Right Back: " + powerRB);
        telemetry.addData("Power", "Left Front: " + powerLF);
        telemetry.addData("Power", "Left Back: " + powerLB);
        telemetry.addData("Position", beacon_presser.getPosition());
    }

    @Override
    public void stop() {

        // To assure we don't get in trouble once the game ends, set the all the motors to zero.
        leftMotor1.setPower(0);
        leftMotor2.setPower(0);
        rightMotor1.setPower(0);
        rightMotor2.setPower(0);

    }

}
