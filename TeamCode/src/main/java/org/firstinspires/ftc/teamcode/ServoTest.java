package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

// Created on 12/15/2017 at 9:22 PM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode

@TeleOp(name = "ServoTest", group = "Iterative Opmode")
//@Disabled
public class ServoTest extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private Servo con;

    @Override
    public void init() {
        con = hardwareMap.servo.get("con");

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
        con.setPosition(.5);
    }

    boolean lbump = false;
    @Override
    public void loop() {
        if (!lbump && gamepad1.left_bumper) { // First press of right bumper
            con.setPosition(.784);
            lbump = true;
        } else if (lbump && !gamepad1.left_bumper) { // Release of right bumper
            con.setPosition(.5);
            lbump = false;
        }
        telemetry.addData("Status", "Running: " + runtime.toString());
    }

    @Override
    public void stop() {
    }

}
