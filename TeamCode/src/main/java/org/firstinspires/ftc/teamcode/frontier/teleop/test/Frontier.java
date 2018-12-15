package org.firstinspires.ftc.teamcode.frontier.teleop.test;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.frontier.control.ToggleServo;

// Created on 12/9/2018 at 3:00 PM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode

@TeleOp(name = "Frontier Test", group = "test")
//@Disabled
public class Frontier extends OpMode {
    
    DcMotor rf, rb, lf, lb, lift;
    ToggleServo lock;
    BNO055IMU   imur, imul;
    Orientation rangles, langles;
    
    double unlocked = .5, locked = .4;
    
    @Override
    public void init() {
        
        rf = hardwareMap.dcMotor.get("rf");
        rb = hardwareMap.dcMotor.get("rb");
        lf = hardwareMap.dcMotor.get("lf");
        lb = hardwareMap.dcMotor.get("lb");
    
        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rb.setDirection(DcMotorSimple.Direction.REVERSE);
        lf.setDirection(DcMotorSimple.Direction.FORWARD);
        lb.setDirection(DcMotorSimple.Direction.FORWARD);
    
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        lift = hardwareMap.dcMotor.get("l");
        lift.setDirection(DcMotorSimple.Direction.FORWARD);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        lock = new ToggleServo(hardwareMap.servo.get("lock"), unlocked, locked);
    
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
    
        imur = hardwareMap.get(BNO055IMU.class, "imur");
        imul = hardwareMap.get(BNO055IMU.class, "imul");
        
        imur.initialize(parameters);
        imul.initialize(parameters);
        
        while (!imul.isGyroCalibrated() && !imur.isGyroCalibrated()) telemetry.addData("Status", "Calibrating gyros...");
        
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        lock.open();
    }

    @Override
    public void loop() {
        float powerRF, powerRB, powerLF, powerLB;
        float
                straight    = -gamepad1.left_stick_y,
                strafe      = gamepad1.left_stick_x,
                rotate      = gamepad1.right_stick_x;
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
    
        lf.setPower(powerLF);
        lb.setPower(powerLB);
        rf.setPower(powerRF);
        rb.setPower(powerRB);
        
        if (gamepad1.right_bumper && !gamepad1.left_bumper) lift.setPower(.5);
        else if (gamepad1.left_bumper && !gamepad1.right_bumper) lift.setPower(-.5);
        else lift.setPower(0);
        
        lock.tob(gamepad1.x);
        
        rangles = imur.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        langles = imul.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
    
        Acceleration racc = imur.getAcceleration(), lacc = imul.getAcceleration();
        telemetry.addData("lift", lift.getCurrentPosition());
        telemetry.addData("RX", rangles.firstAngle);
        telemetry.addData("RY", rangles.secondAngle);
        telemetry.addData("RZ", rangles.thirdAngle);
        telemetry.addData("LX", rangles.firstAngle);
        telemetry.addData("LY", rangles.secondAngle);
        telemetry.addData("LZ", rangles.thirdAngle);
        telemetry.addData("RXA", racc.xAccel);
        telemetry.addData("RYA", racc.yAccel);
        telemetry.addData("RZA", racc.zAccel);
        telemetry.addData("LXA", lacc.xAccel);
        telemetry.addData("LYA", lacc.yAccel);
        telemetry.addData("LZA", lacc.zAccel);
        
    }

    @Override
    public void stop() {
        
        rf.setPower(0);
        rb.setPower(0);
        lf.setPower(0);
        lb.setPower(0);
        
    }

}