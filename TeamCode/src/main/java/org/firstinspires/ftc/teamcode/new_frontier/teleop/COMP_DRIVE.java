package org.firstinspires.ftc.teamcode.new_frontier.teleop;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.new_frontier.control.Arm;
import org.firstinspires.ftc.teamcode.new_frontier.control.SingleRoller;

// Created on 2/7/2019 at 8:00 AM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode.new_frontier.teleop

@TeleOp(name = "COMP DRIVE", group = "comp")
//@Disabled
public class COMP_DRIVE extends LinearOpMode {
    
    private DcMotor rf, rb, lf, lb;
    private SingleRoller r;
    private Arm          a;
    private double       maxPower = .6;
    
    @Override
    public void runOpMode() {
    
        rf = hardwareMap.dcMotor.get("rf");
        rb = hardwareMap.dcMotor.get("rb");
        lf = hardwareMap.dcMotor.get("lf");
        lb = hardwareMap.dcMotor.get("lb");
    
        rf.setDirection(DcMotorSimple.Direction.FORWARD);
        rb.setDirection(DcMotorSimple.Direction.FORWARD);
        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);
    
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    
        r = new SingleRoller(hardwareMap.crservo.get("roller"), (ModernRoboticsI2cColorSensor) hardwareMap.colorSensor.get("color"));
    
        a = new Arm(
                hardwareMap.dcMotor.get("shoulder"),
                hardwareMap.dcMotor.get("elbow"),
                (ModernRoboticsTouchSensor) hardwareMap.touchSensor.get("slim"),
                (ModernRoboticsTouchSensor) hardwareMap.touchSensor.get("elim"),
                this);
    
        telemetry.addData("Status", "Homing arm...");
        telemetry.update();
    
        a.init();
    
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        
        while (opModeIsActive()) {
    
            if (gamepad1.left_stick_button) maxPower = 1;
            else maxPower = .6;
    
            double powerRF, powerRB, powerLF, powerLB;
            double
                    straight    = -gamepad1.right_stick_y,
                    strafe      = gamepad1.right_stick_x,
                    rotate      = gamepad1.left_stick_x;
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
            if (powerRF > 1) {powerRF = 1f;} else if (powerRF < -1) {powerRF = -1f;}
            if (powerRB > 1) {powerRB = 1f;} else if (powerRB < -1) {powerRB = -1f;}
            if (powerLF > 1) {powerLF = 1f;} else if (powerLF < -1) {powerLF = -1f;}
            if (powerLB > 1) {powerLB = 1f;} else if (powerLB < -1) {powerLB = -1f;}
    
            if (gamepad1.right_trigger > 0.5) { powerLB /= 4f; powerLF /= 4f; powerRB /= 4f; powerRF /= 4f; }
    
            lf.setPower(powerLF*maxPower);
            lb.setPower(powerLB*maxPower);
            rf.setPower(powerRF*maxPower);
            rb.setPower(powerRB*maxPower);
    
            r.drive(gamepad1.dpad_up, gamepad1.dpad_down);
    
            a.drive(gamepad1.a, gamepad1.dpad_left, gamepad1.b, gamepad1.y, gamepad1.x,
                    gamepad1.left_bumper, gamepad1.right_bumper, gamepad1.right_stick_button,
                    gamepad1.left_trigger, gamepad1.right_trigger);
    
            telemetry.addData("grabbed", r.getGrabbed());
    
            a.telemetry();
    
            telemetry.update();
            
        }
    }
}