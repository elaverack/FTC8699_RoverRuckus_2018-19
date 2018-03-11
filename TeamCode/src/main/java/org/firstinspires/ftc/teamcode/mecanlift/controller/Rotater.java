package org.firstinspires.ftc.teamcode.mecanlift.controller;

// Created on 2/11/2018 at 8:56 PM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode.mecanlift.controller

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Rotater {

    private static final int
            not_flipped_pre = 80,
            not_flipped_pos = 30,
            flipped_pre = 1580,
            flipped_pos = 1640,
            flip_position = 1000;   // Position of lift when flipping from ground position
    private static final double
            pre_power = 1,
            pos_power = .3;

    private DcMotor r;
    private Lift lift;
    public boolean
            flipped = false;    // Boolean to determine whether the grabber is flipped or not
    private boolean
            rotated = false,    // Boolean for rotating button logic
            rotating = false,   // Boolean to determine whether or not we are currently rotating the grabber
            fixing = false,     // Boolean to determine whether or not we are currently fixing the grabber
            fixed = false,      // Boolean for fixing button logic
            lifted = false,     // Boolean to determine whether or not the lift has gone up a little bit for rotating
            pre = true;

    public Rotater (DcMotor rotater, Lift lift) {
        this.lift = lift;
        r = rotater;
        r.setDirection(DcMotorSimple.Direction.REVERSE);
        r.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        r.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void doRotation (boolean b) {
        if (rotating) {
            if (Lift.update_encoders(r)) { // Done rotating
                if (pre) {
                    if (flipped) {
                        r.setTargetPosition(not_flipped_pos);
                    } else r.setTargetPosition(flipped_pos);
                    r.setPower(pos_power);
                    if (lifted) { lift.ground(); lifted = false; }
                    pre = false;
                    return;
                }
                rotating = false;
                flipped = !flipped;
                pre = true;
                return;
            } else return;
        }

        if (rotated && !b) { rotated = false; }

        if (!b) return;

        if (!rotated) { // Start rotating
            if (lift.grounded()) { lifted = true; lift.setPosition(flip_position); }
            if (flipped) {
                r.setTargetPosition(not_flipped_pre);
            } else r.setTargetPosition(flipped_pre);
            r.setPower(pre_power);
            rotating = true;
        }
    }

    public void doRotFix (boolean b) {
        if (fixing && Lift.update_encoders(r)) {
            if (pre) {
                r.setTargetPosition(-flipped_pos);
                r.setPower(pos_power);
                pre = false;
                return;
            }
            r.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            r.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if (lifted) { lift.ground(); lifted = false; }
            fixing = false;
            pre = true;
            return;
        }
        if (fixed && !b) { fixed = false; return; }
        if (!b) return;
        if (!fixed) {
            if (lift.grounded()) { lifted = true; lift.setPosition(flip_position); }
            r.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            r.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            r.setTargetPosition(-flipped_pre);
            r.setPower(pre_power);
            fixing = true;
        }
    }

    public void stop () { r.setPower(0); }

}
