package org.firstinspires.ftc.teamcode.mecanlift.controller;

// Created on 1/28/2018 at 11:29 AM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode.mecanlift

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.visuals.VisualsHandler;

import static org.firstinspires.ftc.teamcode.visuals.Vector3.round;

public class Mecanlift {

    public enum Color {
        RED, BLUE, ERROR;

        public static Color readCS (ColorSensor s) {
            if (s.red() > s.blue()) return RED;
            if (s.blue() > s.red()) return BLUE;
            return ERROR;
        }
    }

    /** CONSTANTS */
    private static final double     // Grabber servo positions
            blo = 0.078,            // Bottom left open
            blc = 0.310,            // Bottom left close
            bro = 0.98,             // Bottom right open
            brc = 0.670,            // Bottom right close
            tlo = 0.98,             // Top left open
            tlc = 0.725,            // Top left close
            tro = 0.118,            // Top right open
            trc = 0.392,            // Top right close
            jewelArm_down = .18,   // The jewel servo position for lowering the arm
            jewelArm_up = 1;       // The jewel servo position for raising the arm
    protected static final String
            drive_rfN = "rf",   // The front right motor name
            drive_rbN = "rb",   // The back right motor name
            drive_lfN = "lf",   // The front left motor name
            drive_lbN = "lb",   // The back left motor name

            // Note: Looking at robot, not from
            grabber_blN = "bl", // The bottom left grabber servo name
            grabber_brN = "br", // The bottom right grabber servo name
            grabber_tlN = "tl", // The top left grabber servo name
            grabber_trN = "tr", // The top right grabber servo name

            liftN = "lift",     // The lift motor name

            jewelN = "jewel",   // The jewel arm servo motor name
            colorN = "color",   // The jewel arm color sensor name
            gyroN = "gyro";     // The gyro sensor name

    /** OPMODE */
    private OpMode opmode;

    /** DRIVE */
    private DcMotor rf, rb, lf, lb;
    private float
            powerRF,
            powerRB,
            powerLF,
            powerLB;

    /** GRABBERS */
    private ToggleServo tl, tr, bl, br;

    /** LIFT */
    private Lift lift;

    /** VISUALS */
    private VisualsHandler visuals;

    /** SENSORS */
    private GyroSensor gyro;
    private ModernRoboticsI2cColorSensor jewelSensor;

    /** JEWELS */
    private Servo jewelArm;

    /** FOR AUTONOMOUS USE */
    private Color allianceColor;

    /** INITIALIZERS */
    public Mecanlift (OpMode om) { init(om, false, Color.ERROR); } // USE IF TELEOP
    public Mecanlift (OpMode om, /*boolean auto,*/ Color allianceColor) { init(om, true, allianceColor); } // USE IF AUTO
    private void init(OpMode om, boolean auto, Color ac) {
        opmode = om;
        allianceColor = Color.ERROR;

        /** DRIVE */
        rf = opmode.hardwareMap.dcMotor.get(drive_rfN);
        rb = opmode.hardwareMap.dcMotor.get(drive_rbN);
        lf = opmode.hardwareMap.dcMotor.get(drive_lfN);
        lb = opmode.hardwareMap.dcMotor.get(drive_lbN);

        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rb.setDirection(DcMotorSimple.Direction.REVERSE);
        lf.setDirection(DcMotorSimple.Direction.FORWARD);
        lb.setDirection(DcMotorSimple.Direction.FORWARD);

        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /** LIFT */
        lift = new Lift(opmode.hardwareMap.dcMotor.get(liftN));

        /** GRABBER */
        bl = new ToggleServo(opmode.hardwareMap.servo.get(grabber_blN), blo, blc);
        br = new ToggleServo(opmode.hardwareMap.servo.get(grabber_brN), bro, brc);
        tl = new ToggleServo(opmode.hardwareMap.servo.get(grabber_tlN), tlo, tlc);
        tr = new ToggleServo(opmode.hardwareMap.servo.get(grabber_trN), tro, trc);

        /** JEWEL ARM */
        jewelArm = opmode.hardwareMap.servo.get(jewelN);

        /** AUTONOMOUS */
        if (auto) {
            /** VISUALS */
            visuals = new VisualsHandler(opmode, false);

            /** SENSORS */
            jewelSensor = (ModernRoboticsI2cColorSensor)opmode.hardwareMap.colorSensor.get(colorN);
            jewelSensor.enableLed(true); // NOTE: I don't know which one actually works, so this is just assurance.
            jewelSensor.enableLight(true);
            gyro = opmode.hardwareMap.gyroSensor.get(gyroN);
            gyro.calibrate();
            while (gyro.isCalibrating()) {
                opmode.telemetry.addData("Status", "Waiting on gyro...");
                opmode.telemetry.update();
            }

            allianceColor = ac;
        }
    }

    /** LOOP METHODS */
    public void drive() { // Assuming default controls as of 1/28

        /** LIFT */
        lift.run(opmode.gamepad1.dpad_up, opmode.gamepad1.dpad_down, opmode.gamepad1.y, opmode.gamepad1.x);

        /** GRABBER */
        boolean a = opmode.gamepad1.a, b = opmode.gamepad1.b;
        bl.tob(a);
        br.tob(a);
        tl.tob(b);
        tr.tob(b);

        /** MECANUM WHEELS */
        float
            straight    = -opmode.gamepad1.right_stick_y,
            strafe      = opmode.gamepad1.right_stick_x,
            rotate      = opmode.gamepad1.left_stick_x;
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
        if (powerRF > 1) {powerRF = 1;} else if (powerRF < -1) {powerRF = -1;}
        if (powerRB > 1) {powerRB = 1;} else if (powerRB < -1) {powerRB = -1;}
        if (powerLF > 1) {powerLF = 1;} else if (powerLF < -1) {powerLF = -1;}
        if (powerLB > 1) {powerLB = 1;} else if (powerLB < -1) {powerLB = -1;}

        if (opmode.gamepad1.right_trigger > 0.5) { powerLB /= 4; powerLF /= 4; powerRB /= 4; powerRF /= 4; }

        lf.setPower(powerLF);
        lb.setPower(powerLB);
        rf.setPower(powerRF);
        rb.setPower(powerRB);

    }
    public void drive(boolean debug_drive) {
        drive();
        if (debug_drive) {
            opmode.telemetry.addData("Joys", String.format("y: %1$s, x: %2$s, r: %3$s",
                    round(-opmode.gamepad1.right_stick_y),
                    round(opmode.gamepad1.right_stick_x),
                    round(opmode.gamepad1.left_stick_x)
            ));
            opmode.telemetry.addData("Pows", String.format("rf: %1$s, rb: %2$s, lf: %3$s, lb: %4$s",
                    round(powerRF), round(powerRB), round(powerLF), round(powerLB)
            ));
        }
    }
    public void drive(
            boolean liftUp, boolean liftDown, boolean liftDirectUp, boolean liftDirectDown,
            boolean bottomToggle, boolean topToggle,
            float straight, float strafe, float rotate, boolean slow) {
        /** LIFT */
        lift.run(liftUp, liftDown, liftDirectUp, liftDirectDown);

        /** GRABBER */
        bl.tob(bottomToggle);
        br.tob(bottomToggle);
        tl.tob(topToggle);
        tr.tob(topToggle);

        /** MECANUM WHEELS */
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
        if (powerRF > 1) {powerRF = 1;} else if (powerRF < -1) {powerRF = -1;}
        if (powerRB > 1) {powerRB = 1;} else if (powerRB < -1) {powerRB = -1;}
        if (powerLF > 1) {powerLF = 1;} else if (powerLF < -1) {powerLF = -1;}
        if (powerLB > 1) {powerLB = 1;} else if (powerLB < -1) {powerLB = -1;}
        if (slow) { powerLB /= 4; powerLF /= 4; powerRB /= 4; powerRF /= 4; }
        lf.setPower(powerLF);
        lb.setPower(powerLB);
        rf.setPower(powerRF);
        rb.setPower(powerRB);
    }
    public void drive(
            boolean liftUp, boolean liftDown, boolean liftDirectUp, boolean liftDirectDown,
            float brPos, float blPos, float trPos, float tlPos,
            float straight, float strafe, float rotate, boolean slow) {
        /** LIFT */
        lift.run(liftUp, liftDown, liftDirectUp, liftDirectDown);

        /** GRABBER */
        bl.setPos(blPos);
        br.setPos(brPos);
        tl.setPos(tlPos);
        tr.setPos(trPos);

        /** MECANUM WHEELS */
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
        if (powerRF > 1) {powerRF = 1;} else if (powerRF < -1) {powerRF = -1;}
        if (powerRB > 1) {powerRB = 1;} else if (powerRB < -1) {powerRB = -1;}
        if (powerLF > 1) {powerLF = 1;} else if (powerLF < -1) {powerLF = -1;}
        if (powerLB > 1) {powerLB = 1;} else if (powerLB < -1) {powerLB = -1;}
        if (slow) { powerLB /= 4; powerLF /= 4; powerRB /= 4; powerRF /= 4; }
        lf.setPower(powerLF);
        lb.setPower(powerLB);
        rf.setPower(powerRF);
        rb.setPower(powerRB);
    }

    /** STOP METHODS */
    public void stop() {

        /** DRIVE */
        rf.setPower(0);
        rb.setPower(0);
        lf.setPower(0);
        lb.setPower(0);

        /** LIFT */
        lift.stop();

    }

    /** JEWEL METHODS */
    public void raiseArm() { jewelArm.setPosition(jewelArm_up); }
    public void lowerArm() { jewelArm.setPosition(jewelArm_down); }


}
