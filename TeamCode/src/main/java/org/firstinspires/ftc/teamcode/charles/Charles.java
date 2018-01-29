package org.firstinspires.ftc.teamcode.charles;

// Created on 9/6/2017 at 8:23 PM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

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

    /** Upgrades: */
    private final static String
            liftMotorN  = "lift",       // Lift motor name

            tServoN     = "top",       // Top servo name
            bServoN     = "bom",       // Bottom servo name
            cServoN     = "con";       // Conveyor servo name

    private DcMotor liftMotor;

    private Servo
            tServo,
            bServo,
            cServo;

    private final double
            tsClose = .431,
            tsHold  = .49,
            tsOpen  = .686,

            bsClose = .431,
            bsHold  = .529,
            bsOpen  = .784,

            csStop  = .5,
            csFor   = .784,
            csBack  = .235,

            liftS   = .55,
            liftDS  = .25;

    private final int
            lift0 = 900,
            lift1 = 3000,
            lift2 = 5330,
            thres = 10;

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

        liftMotor = opmode.hardwareMap.dcMotor.get(liftMotorN);
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        tServo = opmode.hardwareMap.servo.get(tServoN);
        bServo = opmode.hardwareMap.servo.get(bServoN);
        cServo = opmode.hardwareMap.servo.get(cServoN);
    }

    public void start() {
        liftMotor.setTargetPosition(lift0);
        liftMotor.setPower(liftS);
        cServo.setPosition(csStop);
        tServo.setPosition(tsHold);
        while (!(liftMotor.getCurrentPosition() < liftMotor.getTargetPosition() + thres &&
                liftMotor.getCurrentPosition() > liftMotor.getTargetPosition() - thres)) {}
        bServo.setPosition(bsOpen);
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

        if (opmode.gamepad1.right_trigger > 0.5) { left /= 4; right /= 4; }

        drive[0][0].setPower(left);
        drive[1][0].setPower(right);

        doMechanisms(opmode.gamepad1, opmode.gamepad2);
    }

    private boolean
            up = false,
            down = false,
            y = false,
            a = false,
            x = false,
            b = false,
            rbump = false,
            lbump = false;
    private void doMechanisms (Gamepad g1, Gamepad g2) {

        /** Lift */
        if (!up && g1.dpad_up) {
            switch (liftMotor.getTargetPosition()) {
                case lift0:
                    liftMotor.setTargetPosition(lift1);
                    liftMotor.setPower(liftS);
                    break;
                case lift1:
                    liftMotor.setTargetPosition(lift2);
                    liftMotor.setPower(liftS);
                    break;
                case lift2:
                    liftMotor.setTargetPosition(lift1);
                    liftMotor.setPower(-liftS);
                    break;
            }
            up = true;
        } else if (up && !g1.dpad_up) up = false;

        if (!down && g1.dpad_down) {
            liftMotor.setTargetPosition(lift0);
            liftMotor.setPower(-liftS);
        } else if (!g1.dpad_down) down = false;

        if (liftMotor.getMode() == DcMotor.RunMode.RUN_TO_POSITION &&
                (liftMotor.getCurrentPosition() < liftMotor.getTargetPosition() + thres &&
                        liftMotor.getCurrentPosition() > liftMotor.getTargetPosition() - thres)) {
            liftMotor.setPower(0);
        }

        if (!y && g1.y) { // First press of y
            liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            liftMotor.setPower(liftDS);
            y = true;
        } else if (y && g1.y) { // Holding y
            liftMotor.setPower(liftDS);
        } else if (y) { // Release of y
            liftMotor.setPower(0);
            y = false;
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if (!a && g1.a) { // First press of a
            liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            liftMotor.setPower(-liftDS);
            a = true;
        } else if (a && g1.a) { // Holding a
            liftMotor.setPower(-liftDS);
        } else if (a) { // Release of a
            liftMotor.setPower(0);
            a = false;
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        /** Conveyor */
        if (!lbump && g1.left_bumper) { // First press of right bumper
            cServo.setPosition(csFor);
            lbump = true;
        } else if (lbump && !g1.left_bumper) { // Release of right bumper
            cServo.setPosition(csStop);
            lbump = false;
        }

        if (!x && g1.x) { // First press x
            cServo.setPosition(csBack);
            x = true;
        } else if (x && !g1.x) { // Release of x
            cServo.setPosition(csStop);
            x = false;
        }

        if (!rbump && g1.right_bumper) {
            if (bServo.getPosition() == bsOpen) {
                bServo.setPosition(bsClose);
            } else {
                bServo.setPosition(bsOpen);
            }
            rbump = true;
        } else if (!g1.right_bumper) rbump = false;

        if (g1.left_trigger > .5) {
            bServo.setPosition(bsOpen); tServo.setPosition(tsOpen);
        } else if (!b && g1.b) {
            if (tServo.getPosition() == tsHold) {
                tServo.setPosition(tsClose);
            } else {
                tServo.setPosition(tsHold);
            }
            b = true;
        } else if (!g1.b) {
            b = false;
        }
        if (tServo.getPosition() != tsHold && g1.left_bumper) {
            tServo.setPosition(tsHold);
        }
    }


    public void stop() {
        drive[0][0].setPower(0);
        drive[1][0].setPower(0);
        liftMotor.setPower(0);
        cServo.setPosition(csStop);
    }

}
