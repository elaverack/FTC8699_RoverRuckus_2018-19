package org.firstinspires.ftc.teamcode.comp0120;

// Created on 1/12/2018 at 8:59 PM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class unnamed {

    protected static final int thres = 10; // Threshold for encoder checking

    protected static final String
            drive_rfN       = "rf",
            drive_rbN       = "rb",
            drive_lfN       = "lf",
            drive_lbN       = "lb",

            grabber_blN     = "bl",
            grabber_brN     = "br",
            grabber_tlN     = "tl",
            grabber_trN     = "tr",

            liftN           = "lift";

    protected OpMode opmode;
    protected Drive drive;
    protected G_Grabber grab;
    protected Lift lift;

    protected unnamed () {}
    public unnamed (OpMode om) { opmode = om; drive = new Drive(); grab = new G_Grabber(); lift = new Lift(); init(); }

    protected void init () { drive.init(); grab.init(); lift.init(); }

    public void start () { grab.start(); lift.start(); }

    public void drive () {
        drive.run(opmode.gamepad1);
        grab.run(opmode.gamepad1.a, opmode.gamepad1.b);
        lift.run(opmode.gamepad1.dpad_up, opmode.gamepad1.dpad_down, opmode.gamepad1.y, opmode.gamepad1.x);
    }

    public void stop () {
        drive.stop();
        lift.stop();
    }

    protected boolean update_encoders (DcMotor m) {
        if (m.getMode() != DcMotor.RunMode.RUN_TO_POSITION) return false;
        boolean ret = m.getCurrentPosition() < m.getTargetPosition() + thres &&
                m.getCurrentPosition() > m.getTargetPosition() - thres;
        if (ret) m.setPower(0);
        return ret;
    }

    protected class Drive { // Note: mecanum

        private DcMotor rf, rb, lf, lb;

        void init() {
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
        }

        void run (Gamepad g) { // Note: with slow controls on RT
            float straight = -g.left_stick_y;
            float strafe = -g.left_stick_x;
            float rotate = -g.right_stick_x;
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
            powerRF = clip(powerRF);
            powerRB = clip(powerRB);
            powerLF = clip(powerLF);
            powerLB = clip(powerLB);

            if (g.right_trigger > 0.5) { powerLB /= 4; powerLF /= 4; powerRB /= 4; powerRF /= 4; }

            lf.setPower(powerLF);
            lb.setPower(powerLB);
            rf.setPower(powerRF);
            rb.setPower(powerRB);
        }

        void stop() { setAll(0); }

        void setAll (double power) { lf.setPower(power); lb.setPower(power); rf.setPower(power); rb.setPower(power); }

    }

    protected class G_Grabber {

        private G_servo bl, br, tl, tr;

        // TODO: Add in safe guards in case grabber servos loosen again
        private static final double // Servo positions
                blo = 0.078,    // Bottom left open
                blc = 0.310,    // Bottom left close
                bro = 0.98,     // Bottom right open
                brc = 0.670,    // Bottom right close
                tlo = 0.98,     // Top left open
                tlc = 0.725,    // Top left close
                tro = 0.118,    // Top right open
                trc = 0.392;    // Top right close

        void init () {
            bl = new G_servo(grabber_blN, blo, blc);
            br = new G_servo(grabber_brN, bro, brc);
            tl = new G_servo(grabber_tlN, tlo, tlc);
            tr = new G_servo(grabber_trN, tro, trc);
        }

        void start () { open_b(); open_t(); }

        // TODO: Add code for rotation
        private boolean b_togd = false, t_togd = false;
        void run (boolean b_tog, boolean t_tog) {
            if (b_tog && !b_togd) {
                tog_b();
                b_togd = true;
            } else if (!b_tog) b_togd = false;
            if (t_tog && !t_togd) {
                tog_t();
                t_togd = true;
            } else if (!t_tog) t_togd = false;
        }

        void open_b  () { bl.open();    br.open();      }
        void open_t  () { tl.open();    tr.open();      }
        void close_b () { bl.close();   br.close();     }
        void close_t () { tl.close();   tr.close();     }
        void tog_b   () { bl.toggle();  br.toggle();    }
        void tog_t   () { tl.toggle();  tr.toggle();    }

        private class G_servo {

            private double c, o;
            private Servo s;

            G_servo (String config_name, double open_pos, double close_pos) {
                s = opmode.hardwareMap.servo.get(config_name);
                o = open_pos; c = close_pos;
            }

            void close () { s.setPosition(c); }
            void open () { s.setPosition(o); }
            void toggle () { if (s.getPosition() == o) { close(); } else open(); }

        }

    }

    protected class Lift {

        private DcMotor l;

        private static final int
                lift0 = 0,
                lift1 = 3000,
                lift2 = 5330;

        private static final double
                liftS   = .55,
                liftDS  = .25;

        void init () {
            l = opmode.hardwareMap.dcMotor.get(liftN);
            l.setDirection(DcMotorSimple.Direction.REVERSE);
            l.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            l.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        void start () {
            if (lift0 == 0) return;
            l.setTargetPosition(lift0);
            l.setPower(liftS);
            while (!update_encoders(l));
        }

        // TODO: Add grounding code (and method)
        private boolean upd = false, gd = false, ddupd = false, dddownd = false;
        void run (boolean up, boolean ground, boolean dd_up, boolean dd_down) {

            if (!upd && up) {
                switch (l.getTargetPosition()) {
                    case lift0:
                        l.setTargetPosition(lift1); l.setPower(liftS);
                        break;
                    case lift1:
                        l.setTargetPosition(lift2); l.setPower(liftS);
                        break;
                    case lift2:
                        l.setTargetPosition(lift1); l.setPower(-liftS);
                        break;
                }
                upd = true;
            } else if (!up) upd = false;

            if (!gd && ground) { l.setTargetPosition(lift0); l.setPower(-liftS); gd = true; }
            else if (!ground) gd = false;

            update_encoders(l);

            if (!ddupd && dd_up) {
                l.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                l.setPower(liftDS);
                ddupd = true;
            } else if (ddupd && !dd_up) {
                l.setPower(0);
                ddupd = false;
                l.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if (!dddownd && dd_down) {
                l.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                l.setPower(-liftDS);
                dddownd = true;
            } else if (dddownd && !dd_down) {
                l.setPower(0);
                dddownd = false;
                l.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

        }

        void stop() {
            l.setPower(0);
        }

    }

    private float clip (float v) {
        if (v > 1) return 1;
        if (v < -1) return -1;
        return v;
    }

}
