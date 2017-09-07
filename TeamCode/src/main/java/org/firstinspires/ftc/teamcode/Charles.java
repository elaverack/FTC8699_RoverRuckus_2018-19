package org.firstinspires.ftc.teamcode;

// Created on 9/6/2017 at 8:23 PM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Charles {

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
    public Charles(OpMode om) {
        opmode = om;
        drive = new DcMotor[2][2];
        drive[0] = new DcMotor[]{
                opmode.hardwareMap.dcMotor.get("lf"),
                opmode.hardwareMap.dcMotor.get("lb")};
        drive[1] = new DcMotor[]{
                opmode.hardwareMap.dcMotor.get("rf"),
                opmode.hardwareMap.dcMotor.get("rb")};
    }

    /** Drive modes: */
    public void tankDrive() {
        drive[0][0].setPower(-opmode.gamepad1.left_stick_y);
        drive[0][1].setPower(-opmode.gamepad1.left_stick_y);
        drive[1][0].setPower(-opmode.gamepad1.right_stick_y);
        drive[1][1].setPower(-opmode.gamepad1.right_stick_y);
    }

    // Note: guessing which direction right is for left-stick-x (assuming positive)
    public void arcaneDrive() {
        double left = opmode.gamepad1.left_stick_x - opmode.gamepad1.right_stick_y;
        double right = - opmode.gamepad1.right_stick_y - opmode.gamepad1.left_stick_x;
        drive[0][0].setPower(left);
        drive[0][1].setPower(left);
        drive[1][0].setPower(right);
        drive[1][1].setPower(right);
    }

}
