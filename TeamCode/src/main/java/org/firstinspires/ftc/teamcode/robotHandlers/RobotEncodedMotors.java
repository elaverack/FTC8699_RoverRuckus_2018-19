package org.firstinspires.ftc.teamcode.robotHandlers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Chandler on 3/8/2017.
 */

public class RobotEncodedMotors extends RobotMotors {

    RobotEncodedMotors(){}
    public RobotEncodedMotors (HardwareMap hm) {this.hardwareMap = hm;}
    @Deprecated public RobotEncodedMotors (HardwareMap hm, String motorName, DcMotor motor) {
        this.hardwareMap = hm; addMotor(motorName, motor);
    }
    @Deprecated public RobotEncodedMotors (HardwareMap hm, String[] motorNames, DcMotor[] motors) {
        this.hardwareMap = hm; addMotors(motorNames, motors);
    }
    public RobotEncodedMotors (HardwareMap hm, String motorName) {
        this.hardwareMap = hm; addMotor(motorName);
    }
    public RobotEncodedMotors (HardwareMap hm, String[] motorNames) {
        this.hardwareMap = hm; addMotors(motorNames);
    }

    public void setTargetPosition (String motorName, int position, double power) {
        motors.get(motorName).setTargetPosition(position);
        setPower(motorName, power);
    }
    public void setTargetPositions (String[] motorNames, int position, double power) {
        for (String motorName:motorNames) {
            motors.get(motorName).setTargetPosition(position);
        }
        for (String motorName:motorNames) {
            setPower(motorName, power);
        }
    }
    public void setTargetPositions (String[] motorNames, int[] positions, double[] powers) {
        if ((motorNames.length != positions.length || motorNames.length != powers.length)) return;
        /*for (int i = 0; i < motorNames.length; i++) {
            setTargetPosition(motorNames[i], positions[i], powers[i]);
        }*/
        for (int i = 0; i < motorNames.length; i++) {
            motors.get(motorNames[i]).setTargetPosition(positions[i]);
        }
        for (int i = 0; i < motorNames.length; i++) {
            setPower(motorNames[i], powers[i]);
        }
    }
    public void setTargetPositions (String[] motorNames, int[] positions, double power) {
        if (motorNames.length != positions.length) return;
        for (int i = 0; i < motorNames.length; i++) {
            setTargetPosition(motorNames[i], positions[i], power);
        }
    }
    public void setAllTargetPositions (int position, double power) {
        setTargetPositions(getAllMotorNames(), position, power);
    }

    public int getPosition (String motorName) {
        return motors.get(motorName).getCurrentPosition();
    }
    public int[] getPositions (String[] motorNames) {
        int[] Return = new int[motorNames.length];
        for (int i = 0; i < motorNames.length; i++) {
            Return[i] = getPosition(motorNames[i]);
        }
        return Return;
    }
    public int[] getAllPositions () {return getPositions(getAllMotorNames());}

    public int getTargetPosition (String motorName) {
        return motors.get(motorName).getTargetPosition();
    }
    public int[] getTargetPositions (String[] motorNames) {
        int[] Return = new int[motorNames.length];
        for (int i = 0; i < motorNames.length; i++) {
            Return[i] = getTargetPosition(motorNames[i]);
        }
        return Return;
    }
    public int[] getAllTargetPositions () {return getTargetPositions(getAllMotorNames());}

    public boolean updateMotor (String motorName) {
        DcMotor motor = getMotor(motorName);
        if (motor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) return false;
        int target = motor.getTargetPosition(); if (!inRange(motor.getCurrentPosition(), target-5, target+5)) return false;

        setMode(motorName, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(motorName, DcMotor.RunMode.RUN_TO_POSITION);
        return true;
    }
    public boolean updateMotors (String[] motorNames) {
        DcMotor[] motors = new DcMotor[motorNames.length];
        for (int i = 0; i < motorNames.length; i++) {motors[i] = getMotor(motorNames[i]);}
        for (DcMotor motor : motors) {
            if (motor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) return false;
            int target = motor.getTargetPosition(); if (!inRange(motor.getCurrentPosition(), target-5, target+5)) return false;
        }

        setPowers(motorNames, 0);
        setModes(motorNames, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setModes(motorNames, DcMotor.RunMode.RUN_TO_POSITION);
        return true;
    }
    public boolean updateAllMotors () {return updateMotors(getAllMotorNames());}

    private boolean inRange (int comp, int min, int max) {return min < comp && comp < max;}
    private boolean inRange (double comp, double min, double max) {return min < comp && comp < max;}

}
