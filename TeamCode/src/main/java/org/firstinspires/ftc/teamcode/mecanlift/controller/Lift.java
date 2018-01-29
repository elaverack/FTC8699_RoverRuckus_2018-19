package org.firstinspires.ftc.teamcode.mecanlift.controller;

// Created on 1/26/2018 at 3:56 PM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Lift {

    private DcMotor l;

    private static final int
            thres = 10,     // Threshold for encoders

            lift0 = 0,      // Ground level
            lift1 = 3000,   // 1 glyph (6in) high
            lift2 = 5330;   // 2 glyphs (1ft) high

    private static final double
            liftS   = .55,  // Speed lift moves at when going to positions
            liftDS  = .25;  // Speed lift moves at when directly controlled

    private boolean
            upd = false,
            gd = false,
            ddupd = false,
            dddownd = false;

    public Lift(DcMotor lift) {
        l = lift;
        l.setDirection(DcMotorSimple.Direction.REVERSE);
        l.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        l.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void start () {
        if (lift0 == 0) return;
        l.setTargetPosition(lift0);
        l.setPower(liftS);
        while (!update_encoders(l));
    }

    // TODO: Add grounding code (and method)
    public void run (boolean up, boolean ground, boolean dd_up, boolean dd_down) {

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

    public void stop() { l.setPower(0); }

    public static boolean update_encoders (DcMotor m) {
        if (m.getMode() != DcMotor.RunMode.RUN_TO_POSITION) return false;
        boolean ret = m.getCurrentPosition() < m.getTargetPosition() + thres &&
                m.getCurrentPosition() > m.getTargetPosition() - thres;
        if (ret) m.setPower(0);
        return ret;
    }

}
