package org.firstinspires.ftc.teamcode;

// Created on 9/6/2017 at 8:23 PM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Charles {

    /** Essential for all robot controllers: */
    private OpMode opmode;

    /** Drive setup: */
    // Layout is {{left front, left back}, {right front, right back}}. Think left first, front first.
    private DcMotor[][] drive;
    private final static String[] driveNames = {
            "lb",   // Left back    (SW)
            "rb"    // Right back   (SE)
    };

    /** Initializer: */
    public Charles(OpMode om) {
        opmode = om;
        drive = new DcMotor[2][2];
        drive[0] = new DcMotor[]{
                opmode.hardwareMap.dcMotor.get("lb")};
        drive[1] = new DcMotor[]{
                opmode.hardwareMap.dcMotor.get("rb")};
        drive[0][0].setDirection(DcMotorSimple.Direction.FORWARD);
        drive[1][0].setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /** Drive modes: */
    public void tankDrive() {
        drive[0][0].setPower(-opmode.gamepad1.left_stick_y);
        drive[1][0].setPower(-opmode.gamepad1.right_stick_y);
    }

    // Note: guessing which direction right is for left-stick-x (assuming positive)
    public void arcaneDrive() {
        double left = - opmode.gamepad1.right_stick_y + opmode.gamepad1.left_stick_x;
        double right = - opmode.gamepad1.right_stick_y - opmode.gamepad1.left_stick_x;
        drive[0][0].setPower(left);
        drive[1][0].setPower(right);
    }


    public void stop() {
        drive[0][0].setPower(0);
        drive[1][0].setPower(0);
    }

}
