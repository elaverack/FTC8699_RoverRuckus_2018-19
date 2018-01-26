package org.firstinspires.ftc.teamcode.comp1216;

// Created on 9/6/2017 at 8:23 PM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Dylen {

    /** Essential for all robot controllers: */
    private OpMode opmode;

    /** Drive setup: */
    // Layout is {{left front, left back}, {right front, right back}}. Think left first, front first.
    private DcMotor[][] drive;
    private final static String[] driveNames = {
            "lf",   // Left front   (NW)
            "lb",   // Left back    (SW)
            "rf",   // Right front  (NE)
            "rb"    // Right back   (SE)
    };

    /** Initializer: */
    public Dylen(OpMode om) {
        opmode = om;
        drive = new DcMotor[2][2];
        drive[0] = new DcMotor[]{
                opmode.hardwareMap.dcMotor.get("lf"),
                opmode.hardwareMap.dcMotor.get("lb")};
        drive[1] = new DcMotor[]{
                opmode.hardwareMap.dcMotor.get("rf"),
                opmode.hardwareMap.dcMotor.get("rb")};
        drive[0][0].setDirection(DcMotorSimple.Direction.FORWARD);
        drive[0][1].setDirection(DcMotorSimple.Direction.FORWARD);
        drive[1][0].setDirection(DcMotorSimple.Direction.REVERSE);
        drive[1][1].setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /** Drive modes: */
    public void drive() {
        float straight = -opmode.gamepad1.right_stick_y;
        float strafe = opmode.gamepad1.right_stick_x;
        float rotate = opmode.gamepad1.left_stick_x;
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
        drive[0][0].setPower(powerLF);
        drive[0][1].setPower(powerLB);
        drive[1][0].setPower(powerRF);
        drive[1][1].setPower(powerRB);
    }
    public void stop() {
        drive[0][0].setPower(0);
        drive[0][1].setPower(0);
        drive[1][0].setPower(0);
        drive[1][1].setPower(0);
    }
}
