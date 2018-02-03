package org.firstinspires.ftc.teamcode.mecanlift.controller;

// Created on 1/28/2018 at 11:29 AM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode.mecanlift

import android.graphics.Bitmap;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.visuals.VisualsHandler;

import java.security.Policy;

import static org.firstinspires.ftc.teamcode.visuals.Vector3.round;

public class Mecanlift {

    public enum Color {
        RED, BLUE, ERROR;

        public static Color readCS (ColorSensor s) {
            if (s.red() > s.blue()) return RED;
            if (s.blue() > s.red()) return BLUE;
            return ERROR;
        }

        public boolean turnCCW (Color jewelColor) {
            switch (this) {
                case RED:
                    return jewelColor == BLUE;
                case BLUE:
                    return jewelColor == RED;
                default:
                    return false;
            }
        }
    }

    /** CONSTANTS */
    private static final double     // Grabber servo positions
            blo = 0.078,            // Bottom left open
            blc = 0.310,            // Bottom left close
            bro = 0.98,             // Bottom right open
            brc = 0.670,            // Bottom right close
            tlo = 0.60,             // Top left open
            tlc = 0.425,            // Top left close
            tro = 0.118,            // Top right open
            trc = 0.392,            // Top right close
            jewelArm_down = .18,    // The jewel servo position for lowering the arm
            jewelArm_up = 1,        // The jewel servo position for raising the arm
            flip_power = .5;
    private static final int
            nflipped_pos = 0,       // Position
            flipped_pos = 5100,     // Position
            flip_position = 1000;

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
            grabber_rotN = "rot",

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

    /** GRABBER */
    private ToggleServo tl, tr, bl, br;
    private DcMotor rot;
    private boolean flipped = false;

    /** LIFT */
    private Lift lift;

    /** VISUALS */
    private VisualsHandler visuals;
    private ElapsedTime time;

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
        rot = opmode.hardwareMap.dcMotor.get(grabber_rotN);
        rot.setDirection(DcMotorSimple.Direction.REVERSE);
        rot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rot.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        /** JEWEL ARM */
        jewelArm = opmode.hardwareMap.servo.get(jewelN);

        /** AUTONOMOUS */
        gyro = opmode.hardwareMap.gyroSensor.get(gyroN);
        if (auto) {
            /** VISUALS */
            visuals = new VisualsHandler(opmode, false);
            visuals.vertx = x;
            visuals.hory = y;
            try {visuals.showAlignmentLines();} catch (Exception e) { /* meh */ }
            time = new ElapsedTime();

            /** SENSORS */
            jewelSensor = (ModernRoboticsI2cColorSensor)opmode.hardwareMap.colorSensor.get(colorN);
            jewelSensor.enableLed(true); // NOTE: I don't know which one actually works, so this is just assurance.
            jewelSensor.enableLight(true);
            gyro.calibrate();
            while (gyro.isCalibrating()) {
                opmode.telemetry.addData("Status", "Waiting on gyro...");
                opmode.telemetry.update();
            }

            allianceColor = ac;
        } else { opmode.hardwareMap.colorSensor.get(colorN).enableLed(false); }
    }

    private int x = 455, y = 1042;
    public void showAligning () {
        if (((int)(time.seconds()%2) == 1)) try {visuals.showAlignmentLines();} catch (Exception e) { /* meh */ }
    }

    public void start () {
        lift.start();
        raiseArm();
        bl.open();
        br.open();
        tl.open();
        tr.open();
    }

    /** LOOP METHODS */
    public void drive() { // Assuming default controls as of 1/28

        /** LIFT */
        lift.run(opmode.gamepad1.dpad_up, opmode.gamepad1.dpad_down, opmode.gamepad1.y, opmode.gamepad1.x);

        /** GRABBER */
        boolean a = opmode.gamepad1.a, b = opmode.gamepad1.b;
        if (!flipped) {
            bl.tob(b);
            br.tob(b);
            tl.tob(a);
            tr.tob(a);
        } else {
            bl.tob(a);
            br.tob(a);
            tl.tob(b);
            tr.tob(b);
        }
        doRotation(opmode.gamepad1.right_stick_button);
        doRotFix(opmode.gamepad1.right_bumper && opmode.gamepad1.left_bumper);

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
        doQuickTurn(opmode.gamepad1.left_stick_button);
        if (powerRF > 1) {powerRF = 1f;} else if (powerRF < -1) {powerRF = -1f;}
        if (powerRB > 1) {powerRB = 1f;} else if (powerRB < -1) {powerRB = -1f;}
        if (powerLF > 1) {powerLF = 1f;} else if (powerLF < -1) {powerLF = -1f;}
        if (powerLB > 1) {powerLB = 1f;} else if (powerLB < -1) {powerLB = -1f;}

        if (opmode.gamepad1.right_trigger > 0.5) { powerLB /= 4f; powerLF /= 4f; powerRB /= 4f; powerRF /= 4f; }

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
            boolean bottomToggle, boolean topToggle, boolean rot, boolean fixRot,
            float straight, float strafe, float rotate, boolean slow, boolean quickR) {
        /** LIFT */
        lift.run(liftUp, liftDown, liftDirectUp, liftDirectDown);

        /** GRABBER */
        if (flipped) {
            bl.tob(topToggle);
            br.tob(topToggle);
            tl.tob(bottomToggle);
            tr.tob(bottomToggle);
        } else {
            bl.tob(bottomToggle);
            br.tob(bottomToggle);
            tl.tob(topToggle);
            tr.tob(topToggle);
        }
        doRotation(rot);
        doRotFix(fixRot);

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
        doQuickTurn(quickR);
        if (powerRF > 1) {powerRF = 1f;} else if (powerRF < -1f) {powerRF = -1f;}
        if (powerRB > 1) {powerRB = 1f;} else if (powerRB < -1f) {powerRB = -1f;}
        if (powerLF > 1) {powerLF = 1f;} else if (powerLF < -1f) {powerLF = -1f;}
        if (powerLB > 1) {powerLB = 1f;} else if (powerLB < -1f) {powerLB = -1f;}
        if (slow) { powerLB /= 4f; powerLF /= 4f; powerRB /= 4f; powerRF /= 4f; }
        lf.setPower(powerLF);
        lb.setPower(powerLB);
        rf.setPower(powerRF);
        rb.setPower(powerRB);
    }
    public void drive(
            boolean liftUp, boolean liftDown, boolean liftDirectUp, boolean liftDirectDown,
            float brPos, float blPos, float trPos, float tlPos, boolean rot, boolean fixRot,
            float straight, float strafe, float rotate, boolean slow, boolean quickR) {
        /** LIFT */
        lift.run(liftUp, liftDown, liftDirectUp, liftDirectDown);

        // TODO: Make it so that when flipped, the servos keep each of their positions until change of the float
        /** GRABBER */
        bl.setPos(blPos);
        br.setPos(brPos);
        tl.setPos(tlPos);
        tr.setPos(trPos);
        doRotation(rot);
        doRotFix(fixRot);

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
        doQuickTurn(quickR);
        if (powerRF > 1) {powerRF = 1f;} else if (powerRF < -1) {powerRF = -1f;}
        if (powerRB > 1) {powerRB = 1f;} else if (powerRB < -1) {powerRB = -1f;}
        if (powerLF > 1) {powerLF = 1f;} else if (powerLF < -1) {powerLF = -1f;}
        if (powerLB > 1) {powerLB = 1f;} else if (powerLB < -1) {powerLB = -1f;}
        if (slow) { powerLB /= 4f; powerLF /= 4f; powerRB /= 4f; powerRF /= 4f; }
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

        /** GRABBER */
        rot.setPower(0);

    }

    public void closeVisuals () {
        visuals.close();
    }

    /** JEWEL METHODS */
    public void raiseArm() { jewelArm.setPosition(jewelArm_up); }
    public void lowerArm() { jewelArm.setPosition(jewelArm_down); }

    /** QUICK TURN */
    private boolean turned = false, turning = false, past = false;
    private int ang = 0;
    private void doQuickTurn (boolean b) {
        if (turning) {
            if (!past && gyro.getHeading() < ang) { past = true; }
            if (past && gyro.getHeading() > ang) {
                turning = false;
                past = false;
                return;
            }
            powerRF += 1f;
            powerRB += 1f;
            powerLF -= 1f;
            powerLB -= 1f;
            return;
        }
        if (turned && !b) { turned = false; return; }
        if (!b) return;
        if (!turned) {
            turning = true;
            ang = gyro.getHeading() + 180;
            if (ang > 359) ang -= 360;
            powerRF += 1f;
            powerRB += 1f;
            powerLF -= 1f;
            powerLB -= 1f;
        }
    }
    private boolean checkGyro () {
        return (gyro.getHeading() < ang + 2) && (gyro.getHeading() > ang - 2);
    }
    private boolean checkGyro (int goal) {
        return (gyro.getHeading() < goal + 2) && (gyro.getHeading() > goal - 2);
    }
    private void turnToAngle (int ang, boolean ccw, float speed) {
        if (ccw) {
            rf.setPower(speed);
            rb.setPower(speed);
            lf.setPower(-speed);
            lb.setPower(-speed);
            while (gyro.getHeading() > ang);
            while (gyro.getHeading() < ang);
        } else {
            rf.setPower(-speed);
            rb.setPower(-speed);
            lf.setPower(speed);
            lb.setPower(speed);
            while (gyro.getHeading() < ang);
            while (gyro.getHeading() > ang);
        }
        rf.setPower(0);
        rb.setPower(0);
        lf.setPower(0);
        lb.setPower(0);
    }

    /** ROTATION */
    private boolean rotated = false, rotating = false, fixing = false, fixed = false, lifted = false;
    //private ElapsedTime time;
    private void doRotation (boolean b) {
        if (rotating) {
            if (Lift.update_encoders(rot)) { // Done rotating
                rotating = false;
                flipped = !flipped;
                if (lifted) { lift.ground(); lifted = false; }
                return;
            } else return;
        }
        if (rotated && !b) { rotated = false; }
        if (!b) return;
        if (!rotated) { // Start rotating
            if (lift.grounded()) { lifted = true; lift.setPosition(flip_position); }
            if (flipped) {
                rot.setTargetPosition(nflipped_pos);
                rot.setPower(flip_power);
            } else {
                rot.setTargetPosition(flipped_pos);
                rot.setPower(flip_power);
            }
            rotating = true;
        }
    }
    private void doRotFix (boolean b) {
        if (fixing && Lift.update_encoders(rot)) {
            rot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if (lifted) { lift.ground(); lifted = false; }
            fixing = false;
            return;
        }
        if (fixed && !b) { fixed = false; return; }
        if (!b) return;
        if (!fixed) {
            if (lift.grounded()) { lifted = true; lift.setPosition(flip_position); }
            rot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rot.setTargetPosition(-flipped_pos);
            rot.setPower(-flip_power);
            fixing = true;
        }
    }
//    private void checkRotation () {
//        if (!checking) {
//            if (flipped) {
//                rot.setTargetPosition(nflipped_pos);
//                rot.setPower(-1);
//            } else {
//                rot.setTargetPosition(flipped_pos);
//                rot.setPower(1);
//            }
//            time = new ElapsedTime();
//            checking = true;
//            return;
//        }
//        if (time.seconds() >= 1) {
//            if (!flipped && rot.getCurrentPosition() < 10) {
//                flipped = true;
//            } else {
//
//            }
//        }
//    }

    public void driveDistance (double inches) {
        int encoder = 1120 * (int)(inches/(Math.PI * 4.0));
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rf.setTargetPosition(encoder);
        rb.setTargetPosition(encoder);
        lf.setTargetPosition(encoder);
        lb.setTargetPosition(encoder);
        rf.setPower(.5);
        rb.setPower(.5);
        lf.setPower(.5);
        lb.setPower(.5);
        while (!Lift.update_encoders(rb));
        rf.setPower(0);
        lf.setPower(0);
        lb.setPower(0);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void driveToBox (FIELDPOS pos) {
        lift.lift();
        lift.waitForEncoders();
        driveDistance(pos.inchesToBox);
        lift.groundground();
        lift.waitForEncoders();
    }

    private static final int ccwAngle = 10, cwAngle = 350; //NOTE: counter clockwise is positive for gyro
    public void doJewels () {
        br.close();
        bl.close();
        lowerArm();
        ElapsedTime time = new ElapsedTime();
        while (time.seconds() < 5);
        Color jewelC = Color.readCS(jewelSensor);
        if (jewelC == Color.ERROR) { raiseArm(); return; }
        br.close();
        bl.close();
        if (allianceColor.turnCCW(jewelC)) {
            turnToAngle(ccwAngle, true, .5f);
        } else {
            turnToAngle(cwAngle, false, .5f);
        }
        raiseArm();
        br.close();
        bl.close();
    }

    public enum FIELDPOS {
        BLUE_SIDE(70,true,42), BLUE_CORNER(64,true,32.8), RED_SIDE(290,false,42), RED_CORNER(296,false,32.8);
        public final int turnAngle;
        public final boolean ccw;
        public final double inchesToBox;
        FIELDPOS(int ta, boolean ccw, double in) { turnAngle = ta; this.ccw = ccw; inchesToBox = in; }
    }
    public void turnToBox (FIELDPOS pos) {
        turnToAngle(pos.turnAngle, pos.ccw, .5f);
//        int ang = pos.turnAngle;
//        if (pos.ccw) {
//            rf.setPower(.5);
//            rb.setPower(.5);
//            lf.setPower(-.5);
//            lb.setPower(-.5);
//        } else {
//            rf.setPower(-.5);
//            rb.setPower(-.5);
//            lf.setPower(.5);
//            lb.setPower(.5);
//        }
//        while (!checkGyro(ang));
//        rf.setPower(0);
//        rb.setPower(0);
//        lf.setPower(0);
//        lb.setPower(0);
    }

}
