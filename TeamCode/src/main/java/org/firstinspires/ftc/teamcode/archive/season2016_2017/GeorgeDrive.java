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
package org.firstinspires.ftc.teamcode.archive.season2016_2017;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

// Created on 2/15/2017 at 9:18 AM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode.archive.season2016_2017.chandler_preparingForStates2017

@TeleOp(name = "OldJorgeDrive", group = "COMP")
// @Autonomous(...) is the other common choice
@Disabled
public class GeorgeDrive extends OpMode {
    //Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftMotor1;
    private DcMotor leftMotor2;
    private DcMotor rightMotor1;
    private DcMotor rightMotor2;

    private DcMotor shooter;
    private final int SHOOTER_LOAD = 1150;
    private final int SHOOTER_FIRE = 1680;

    private DcMotor pickUp;
    private final double RUN_PICK_UP = 1; //TODO: Make sure this doesn't need to be negative
    private final double STOP_PICK_UP = 0;

    private Servo beacon_presser; //TODO: Test and set these values
        private final double BEACON_DOWN = 1;
        private final double BEACON_UP = 0;

    private Servo loader; //TODO: Test and set these values
        private final double LOAD = 0;
        private final double UNLOAD = 0.39;

    private float rightStickY;
    private float rightStickX;
    private float powerRF;
    private float powerRB;
    private float powerLF;
    private float powerLB;
    private float leftStickX;

    //Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        leftMotor1  = hardwareMap.dcMotor.get("lf");
        leftMotor2  = hardwareMap.dcMotor.get("lb");
        rightMotor1 = hardwareMap.dcMotor.get("rf");
        rightMotor2 = hardwareMap.dcMotor.get("rb");
        leftMotor1.setDirection(DcMotor.Direction.FORWARD);
        leftMotor2.setDirection(DcMotor.Direction.FORWARD);
        rightMotor1.setDirection(DcMotor.Direction.REVERSE);
        rightMotor2.setDirection(DcMotor.Direction.REVERSE);

        beacon_presser = hardwareMap.servo.get("beacon");

        shooter = hardwareMap.dcMotor.get("shoot");
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        pickUp = hardwareMap.dcMotor.get("pickup");

        loader = hardwareMap.servo.get("load");


    }

    //Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
    @Override
    public void init_loop() {}

    //Code to run ONCE when the driver hits PLAY
    @Override
    public void start() {runtime.reset();}

    //Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {
        telemetry.addData("Status", "Running: " + runtime.toString());

        rightStickY = -gamepad1.right_stick_y;
        rightStickX = gamepad1.right_stick_x;
        leftStickX = gamepad1.left_stick_x;
        calcDrivePowers(rightStickY, leftStickX, rightStickX, gamepad1.right_trigger);
        doBeaconPresser(gamepad1.a);
        leftMotor1.setPower(powerLF);
        leftMotor2.setPower(powerLB);
        rightMotor1.setPower(powerRF);
        rightMotor2.setPower(powerRB);

        doPickUp(gamepad2.b);
        if (gamepad2.y) {
            pickUp.setPower(-0.2);
        }
        load(gamepad2.x);
        doShooting(gamepad2.a);

        telemetry.addData("CAM", shooter.getCurrentPosition());
        telemetry.addData("CHECK", (shooter.getCurrentPosition() > SHOOTER_FIRE || shooter.getCurrentPosition() == SHOOTER_FIRE));
        telemetry.addData("SHOOTING N SET", shooting + ", " + set);

    }

    //Code to run ONCE after the driver hits STOP
    @Override
    public void stop() {
        leftMotor1.setPower(0);
        leftMotor2.setPower(0);
        rightMotor1.setPower(0);
        rightMotor2.setPower(0);
        shooter.setPower(0);
        pickUp.setPower(0);
    }

    private boolean shooting = false;
    private boolean set = false;
    private void doShooting (boolean trigger) {

        if (!shooting && trigger) {
            shooting = true;
        }

        if (shooting) {
            if (!set) {
                shooter.setTargetPosition(SHOOTER_FIRE);
                shooter.setPower(.2);
                set = true;
            }
            if ((shooter.getCurrentPosition() > SHOOTER_FIRE || shooter.getCurrentPosition() == SHOOTER_FIRE) && set) {
                shooter.setPower(0);
                shooter.setTargetPosition(0);
                shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                shooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                shooting = false;
                set = false;
            }
        }

    }

    private boolean loadToggled = false;
    private void load (boolean toggleButton) {
        if (!loadToggled && toggleButton) {
            if (loader.getPosition() == UNLOAD) {
                loader.setPosition(LOAD);
            } else {
                loader.setPosition(UNLOAD);
            }
            loadToggled = true;
        } else if (!toggleButton) {
            loadToggled = false;
        }
    }

    private boolean pickupToggled = false;
    private boolean pickupOn = false;
    private void doPickUp (boolean toggleButton) {
        if (!pickupToggled && toggleButton) {
            if (!pickupOn) {
                pickUp.setPower(RUN_PICK_UP);
                pickupOn = true;
                pickupToggled = true;
            } else {
                pickUp.setPower(STOP_PICK_UP);
                pickupOn = false;
                pickupToggled = true;
            }
        } else if (!toggleButton) {
            pickupToggled = false;
        }

    }

    private boolean beaconToggled = false;
    private void doBeaconPresser (boolean toggleButton) {
         if (!beaconToggled && toggleButton) {
             if (beacon_presser.getPosition() == BEACON_UP) {
                 beacon_presser.setPosition(BEACON_DOWN);
             } else {
                 beacon_presser.setPosition(BEACON_UP);
             }
             beaconToggled = true;
         } else if (!toggleButton) {
             beaconToggled = false;
         }
    }

    private void calcDrivePowers (float straight, float rotate, float strafe, float slow) {

        powerRF = straight;
        powerRB = straight;
        powerLF = straight;
        powerLB = straight;
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

    }

}
