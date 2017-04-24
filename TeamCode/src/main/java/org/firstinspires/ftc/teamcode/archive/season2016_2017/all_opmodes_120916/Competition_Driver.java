/* Copyright (c) 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

FLither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITFLSS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWFLR OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSIFLSS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING FLGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.teamcode.archive.season2016_2017.all_opmodes_120916;

import org.firstinspires.ftc.teamcode.archive.season2016_2017.robotCoreFunctions.MotorHandler;
import org.firstinspires.ftc.teamcode.archive.season2016_2017.robotCoreFunctions.ServoHandler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;

/**
 * Regular Tele Op Mode
 * Enables control of the robot via the gamepad.
 * Note: This is Chandler's main testing class
 * This has been modified to use the new robot action methods and classes with the motors and servos
 */
public class Competition_Driver extends OpMode {
    float speedLeft;
    float speedRight;
    float armLeft;
    float armRight;
    //ears starting position
    final double Leftarm_start = 0;
    final double Rightarm_start = 0;
    //to check tha the servo on horns is down
    boolean winchLock = false ;

    boolean dPadUp;
    boolean dPadDown;
    boolean aPressed;
    boolean bPressed;
    boolean xPressed;
    boolean yPressed;
    final String[] armNames = {"hookArm", "winchArm"};


    /*
    DcMotor motorRight1;
    DcMotor motorRight2;
    DcMotor motorLeft1;
    DcMotor motorLeft2;
    Servo LeftArm;
    Servo RightArm;
    */

    ServoHandler Arms;
    MotorHandler DriveTrain;
    MotorHandler arm;
    Servo winch;
    Servo bumper;
    int[] encoderPositions;


    @Override
    public void init() {
        //Arms refers to the flippers
        //arm refers to horns
        DriveTrain = new MotorHandler(hardwareMap, 4.5875f);
        Arms = new ServoHandler(hardwareMap);

        arm = new MotorHandler(hardwareMap, armNames, 1);
        winch = hardwareMap.servo.get("winchServo");

        bumper = hardwareMap.servo.get("bumper");

        DriveTrain.AddMotors(new String[]{"rf", "rb", "lf", "lb"});
        Arms.AddServo(new String[]{"leftArm", "rightArm"});

        DriveTrain.SetDirection(new String[]{"lf", "lb"}, Direction.REVERSE);
        DriveTrain.SetDirection(new String[]{"rf", "rb"}, Direction.FORWARD);

        Arms.SetServo(new String[]{"leftArm", "rightArm"}, new double[]{Leftarm_start, Rightarm_start});
        winch.setPosition(1);
        bumper.setPosition(0.6);

    }

    @Override
    public void loop() {
        speedRight = Range.clip(gamepad1.right_stick_y, -1, 1);
        speedLeft = Range.clip(gamepad1.left_stick_y, -1, 1);
        armLeft = gamepad1.right_trigger * 0.4f;
        armRight = 1 - (gamepad1.left_trigger * 0.4f);

        DriveTrain.setMotorsPower(new String[]{"rf", "rb"}, speedRight);
        DriveTrain.setMotorsPower(new String[]{"lf", "lb"}, speedLeft);

        dPadUp = gamepad2.dpad_up;
        dPadDown = gamepad2.dpad_down;
        aPressed = gamepad2.a;
        bPressed = gamepad2.b;
        xPressed = gamepad2.x;
        yPressed = gamepad2.y;



        Arms.SetServo(new String[]{"leftArm", "rightArm"}, new double[]{armLeft, armRight});

        encoderPositions = DriveTrain.readEncoders(new String[]{"rf", "rb", "lf", "lb"});

        if (dPadUp && !dPadDown) {
            arm.setMotor("hookArm", .50);
        } else if (dPadDown) {
            arm.setMotor("hookArm", -.40);
        } else {
            arm.setAllMotorsPower(0);
        }

        if (aPressed && !bPressed) {
            bumper.setPosition(1);
        } else if (bPressed) {
            bumper.setPosition(0.6);
        }

        if (!winchLock) {
            if (gamepad2.right_trigger >= .75) {
                arm.setMotor("winchArm", .30);
            } else if (gamepad2.left_trigger >= .75) {
                arm.setMotor("winchArm", -.30);
            }
        }


        if (xPressed && !yPressed) {
            winch.setPosition(0);
        } else if (yPressed) {
            winch.setPosition(1);
        }

        telemetry.addData("", encoderPositions[0]);
        telemetry.addData("", encoderPositions[1]);
        telemetry.addData("", encoderPositions[2]);
        telemetry.addData("", encoderPositions[3]);


    }


    @Override
    public void stop() {
        /*
        motorRight1.setPower(0);
        motorLeft1.setPower(0);
        motorRight2.setPower(0);
        motorLeft2.setPower(0);
        */

        DriveTrain.setAllMotorsPower(0);
        Arms.SetServo(new String[]{"leftArm", "rightArm"}, new double[]{Leftarm_start, Rightarm_start});

    }

}