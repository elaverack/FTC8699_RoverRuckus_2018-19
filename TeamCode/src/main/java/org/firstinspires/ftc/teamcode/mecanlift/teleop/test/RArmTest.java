package org.firstinspires.ftc.teamcode.mecanlift.teleop.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

// Created on 2/10/2018 at 11:19 AM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode.mecanlift.teleop.test

@TeleOp(name = "Relic Arm Test", group = "test")
//@Disabled
public class RArmTest extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private ContinuousServo armServo;

    @Override
    public void init() {
        armServo = new ContinuousServo(this, "arm");

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
        armServo.stop();
        hardwareMap.servo.get("jewel").setPosition(1);
        hardwareMap.servo.get("bl").setPosition(0.078);
        hardwareMap.servo.get("br").setPosition(0.98);
        hardwareMap.servo.get("tl").setPosition(0.98);
        hardwareMap.servo.get("tr").setPosition(0.118);
    }

    @Override
    public void loop() {
        armServo.setPower(-gamepad1.right_stick_y);
        telemetry.addData("Status", "Running: " + runtime.toString());
    }

    @Override
    public void stop() {
        armServo.stop();
    }

    class ContinuousServo {

        private Servo s;

        public ContinuousServo (OpMode opMode, String config_name) {
            s = opMode.hardwareMap.servo.get(config_name);
        }

        public void stop() {
            s.setPosition(.5);
        }

        public void setPower(double power) {
            if (power > 1) {power = 1.0;} else if (power < -1) {power = -1.0;}
            power = (power + 1.0)/2.0;
            s.setPosition(power);
        }

        public double getPower() {
            double position = s.getPosition();
            return (position*2.0) - 1.0;
        }

    }

}