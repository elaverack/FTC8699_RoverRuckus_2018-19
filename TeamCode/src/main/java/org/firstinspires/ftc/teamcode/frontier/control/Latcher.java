package org.firstinspires.ftc.teamcode.frontier.control;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Latcher {
    
    public static final int
            thres = 10;     // Threshold for encoders
    
    private static final int
            LIFT_ALIGN      = 2770,     // align position
            LIFT_HOOK       = 5560,     // hook position
            LIFT_UNHOOK     = 7248;    // unhook position
    
    private static final double
            UP_S        = .7,    // Speed slide moves at when going to positions
            LOWER_S     = 1,   // Speed slide moves at when lowering robot
            DIR_S       = .6;   // Speed slide moves at when directly controlled
    
    private DcMotor l;
    private boolean
            upd     = false,    // up position
            downd   = false,    // down position
            ddupd   = false,    // direct drive up
            dddownd = false,    // direct drive down
            fixd    = false,    // reset encoders
            eGood   = false;    // encoder good
    
    public Latcher (DcMotor lift) {
    
        l = lift;
        l.setDirection(DcMotorSimple.Direction.FORWARD);
        l.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        l.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        l.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
    }
    
    public void run (boolean up, boolean down, boolean ddup, boolean dddown, boolean fix) {
        
        do_up(up);
        do_down(down);
        
        update();
        
        do_ddup(ddup);
        do_dddown(dddown);
        
        do_fix(fix);
    
    }
    
    public void stop () { l.setPower(0); }
    
    private void do_up (boolean b) {
        if (upd && !b) { upd = false; return; }
        if (!b) return;
        if (!upd) {
            int cur = l.getCurrentPosition();
            if (cur >= 0 - thres && cur < LIFT_ALIGN - thres) toAlign();
            else if (cur >= LIFT_ALIGN - thres && cur < LIFT_HOOK - thres) toHook();
            else if (cur >= LIFT_HOOK - thres && cur < LIFT_UNHOOK - thres) toUnhook();
            else if (cur < 0 - thres) toGround();
            upd = true;
        }
    }
    private void do_down (boolean b) {
        if (downd && !b) { downd = false; return; }
        if (!b) return;
        if (!downd) {
            int cur = l.getCurrentPosition();
            if (cur > thres && cur <= LIFT_ALIGN + thres) toGround();
            else if (cur > LIFT_ALIGN + thres && cur <= LIFT_HOOK + thres) toAlign();
            else if (cur > LIFT_HOOK + thres && cur <= LIFT_UNHOOK + thres) toHook();
            else if (cur > LIFT_UNHOOK + thres) toUnhook();
            downd = true;
        }
    }
    private void do_ddup (boolean b) {
        if (ddupd && !b) {
            l.setPower(0);
            ddupd = false;
            l.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            return;
        }
        if (!b) return;
        if (!ddupd) {
            l.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            l.setPower(DIR_S);
            ddupd = true;
        }
    }
    private void do_dddown (boolean b) {
        if (dddownd && !b) {
            l.setPower(0);
            dddownd = false;
            l.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            return;
        }
        if (!b) return;
        if (!dddownd) {
            l.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            l.setPower(-DIR_S);
            dddownd = true;
        }
    }
    private void do_fix (boolean b) {
        if (fixd && !b) { fixd = false; return; }
        if (!b) return;
        if (!fixd) {
            l.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            l.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            fixd = true;
        }
    }
    
    private void toPosition (int p) {
        l.setTargetPosition(p);
        l.setPower(UP_S);
        eGood = false;
    }
    private void toGround () { toPosition(0); }
    private void toAlign () { toPosition(LIFT_ALIGN); }
    private void toHook () { toPosition(LIFT_HOOK); }
    private void toUnhook () { toPosition(LIFT_UNHOOK); }
    public void raise () {
        l.setTargetPosition(LIFT_UNHOOK);
        l.setPower(LOWER_S);
        eGood = false;
    }
    public void lower () {
        l.setTargetPosition(0);
        l.setPower(UP_S);
        eGood = false;
    }
    public void unlock () {
        l.setTargetPosition(-20);
        l.setPower(LOWER_S);
        eGood = false;
    }
    
    private void update () {
        if (eGood || l.getMode() != DcMotor.RunMode.RUN_TO_POSITION) return;
        boolean ret = l.getCurrentPosition() < l.getTargetPosition() + thres &&
                l.getCurrentPosition() > l.getTargetPosition() - thres;
        if (ret) {
            l.setPower(0);
            eGood = true;
        }
    }
    public void waitFor (LinearOpMode opmode) {
        if (!opmode.opModeIsActive()) return;
        while (!eGood && opmode.opModeIsActive()) update();
    }
    
}
