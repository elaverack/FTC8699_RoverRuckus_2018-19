package org.firstinspires.ftc.teamcode.mecanlift.teleop.test;

import com.qualcomm.hardware.motors.NeveRest40Gearmotor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

// Created on 2/10/2018 at 11:19 AM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode.mecanlift.teleop.test

@TeleOp(name = "Relic Arm Test", group = "test")
//@Disabled
public class RArmTest extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor out, in;

    @Override
    public void init() {
        //armServo = new ContinuousServo(this, "arm");
        out = hardwareMap.dcMotor.get("out");
        in = hardwareMap.dcMotor.get("in");
        out.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        in.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        out.setDirection(DcMotorSimple.Direction.FORWARD);
        in.setDirection(DcMotorSimple.Direction.FORWARD);
        out.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        in.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
        //armServo.stop();
        hardwareMap.servo.get("jewel").setPosition(.863);
        hardwareMap.servo.get("bl").setPosition(0.078);
        hardwareMap.servo.get("br").setPosition(0.98);
        hardwareMap.servo.get("tl").setPosition(0.98);
        hardwareMap.servo.get("tr").setPosition(0.118);
    }

    @Override
    public void loop() {
        out.setPower(-gamepad1.right_stick_y);
        in.setPower(-gamepad1.left_stick_y);
        telemetry.addData("Status", "Running: " + runtime.toString());
    }

    @Override
    public void stop() {
        out.setPower(0);
        in.setPower(0);
    }

}