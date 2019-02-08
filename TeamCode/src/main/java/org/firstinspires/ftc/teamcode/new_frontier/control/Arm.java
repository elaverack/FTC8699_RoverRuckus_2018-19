package org.firstinspires.ftc.teamcode.new_frontier.control;

import android.util.Log;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Arm {
    
    private static final double SHOULDER_SPEED = .35, ELBOW_SPEED = .25;
    private static final int[]
            OUT_POS = {-2952, 650},
            CLOSE_POS = {-1852, -450},
            SDEP_POS = {-253, 3121},
            GDEP_POS = {-1320, 1964};
    
    private DcMotor shoulder, elbow;
    private ModernRoboticsTouchSensor sl, el;
    private LinearOpMode opmode;
    
    private boolean
            doout = false,
            doclose = false,
            dosdep = false,
            dogdep = false,
            dodd = false,
            doshoulderdir = false,
            doelbowdir = false,
            doreset = false;
    
    private int shoulder_off = 0, elbow_off = 0;
    
    private int sho_dir = 1, elb_dir = 1;
    
    public Arm (DcMotor shoulder, DcMotor elbow, ModernRoboticsTouchSensor shoulder_limit, ModernRoboticsTouchSensor elbow_limit, LinearOpMode om) {
        this.shoulder = shoulder;
        this.elbow = elbow;
        this.shoulder.resetDeviceConfigurationForOpMode();
        this.shoulder.setDirection(DcMotorSimple.Direction.FORWARD);
        this.shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.elbow.resetDeviceConfigurationForOpMode();
        this.elbow.setDirection(DcMotorSimple.Direction.FORWARD);
        this.elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sl = shoulder_limit;
        el = elbow_limit;
        opmode = om;
    }
    
    public void init () {
        if (opmode.isStopRequested()) return;
    
        Log.d("ARM", Boolean.toString(sl.isPressed()));
        Log.d("ARM", shoulder.getMode().toString());
        Log.d("ARM", Boolean.toString(opmode.isStopRequested()));
        
        shoulder.setTargetPosition(0);
        shoulder.setPower(SHOULDER_SPEED);
        while (!sl.isPressed() && !opmode.isStopRequested()) shoulder.setTargetPosition(shoulder.getTargetPosition() + 5);
        //shoulder.setPower(0);
        //shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //shoulder.setTargetPosition(0);
        //shoulder.setPower(SHOULDER_SPEED);
        shoulder_off = shoulder.getCurrentPosition();
    
        elbow.setTargetPosition(0);
        elbow.setPower(ELBOW_SPEED);
        while (!el.isPressed() && !opmode.isStopRequested()) elbow.setTargetPosition(elbow.getTargetPosition() - 5);
        //elbow.setPower(0);
        //elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //elbow.setTargetPosition(0);
        //elbow.setPower(ELBOW_SPEED);
        
        elbow_off = elbow.getCurrentPosition();
        Log.d("ARM", "" + shoulder_off);
        Log.d("ARM", "" + elbow_off);
    }
    
    public void drive (boolean out, boolean close, boolean silverDep, boolean goldDep,
                       boolean directD, boolean shoulderDir, boolean elbowDir, boolean reset, double sho_pow, double elb_pow) {
    
        if (!dodd && directD) {
            shoulder.setPower(0);
            elbow.setPower(0);
            shoulder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            elbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            dodd = true;
        } else if (dodd && !directD) {
            shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            shoulder.setPower(0);
            elbow.setPower(0);
            dodd = false;
        }
    
        if (!doreset && reset) {
            shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            if (dodd) {
                shoulder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                elbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            } else {
                shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                shoulder.setPower(0);
                elbow.setPower(0);
            }
            doreset = true;
        } else if (doreset && !reset) doreset = false;
    
        if (dodd) {
            if (!doshoulderdir && shoulderDir) {
                sho_dir *= -1;
                doshoulderdir = true;
            } else if (doshoulderdir && !shoulderDir) doshoulderdir = false;
        
            if (!doelbowdir && elbowDir) {
                elb_dir *= -1;
                doelbowdir = true;
            } else if (doelbowdir && !elbowDir) doelbowdir = false;
            
            shoulder.setPower(sho_pow * sho_dir);
            elbow.setPower(elb_pow * elb_dir);
            
            return;
        }
        
        if (!doout && out) {
            shoulder.setTargetPosition(OUT_POS[0] + shoulder_off);
            elbow.setTargetPosition(OUT_POS[1]+ elbow_off);
            shoulder.setPower(SHOULDER_SPEED);
            elbow.setPower(ELBOW_SPEED);
            doout = true;
        } else if (doout && !out) doout = false;
    
        if (!doclose && close) {
            shoulder.setTargetPosition(CLOSE_POS[0] + shoulder_off);
            elbow.setTargetPosition(CLOSE_POS[1]+ elbow_off);
            shoulder.setPower(SHOULDER_SPEED);
            elbow.setPower(ELBOW_SPEED);
            doclose = true;
        } else if (doclose && !close) doclose = false;
    
        if (!dosdep && silverDep) {
            shoulder.setTargetPosition(SDEP_POS[0] + shoulder_off);
            elbow.setTargetPosition(SDEP_POS[1]+ elbow_off);
            shoulder.setPower(SHOULDER_SPEED);
            elbow.setPower(ELBOW_SPEED);
            dosdep = true;
        } else if (dosdep && !silverDep) dosdep = false;
    
        if (!dogdep && goldDep) {
            shoulder.setTargetPosition(GDEP_POS[0] + shoulder_off);
            elbow.setTargetPosition(GDEP_POS[1]+ elbow_off);
            shoulder.setPower(SHOULDER_SPEED);
            elbow.setPower(ELBOW_SPEED);
            dogdep = true;
        } else if (dogdep && !goldDep) dogdep = false;
        
    }
    
    public void telemetry () {
        opmode.telemetry.addData("shoulder", shoulder.getCurrentPosition() - shoulder_off);
        opmode.telemetry.addData("elbow", elbow.getCurrentPosition() - elbow_off);
        
        if (sl.isPressed()) opmode.telemetry.addData("shoulder limit", "pressed");
        else opmode.telemetry.addData("shoulder limit", "not pressed");
        if (el.isPressed()) opmode.telemetry.addData("elbow limit", "pressed");
        else opmode.telemetry.addData("elbow limit", "not pressed");
    }
    
    public void stop () {
        shoulder.setPower(0);
        elbow.setPower(0);
    }
    
}
