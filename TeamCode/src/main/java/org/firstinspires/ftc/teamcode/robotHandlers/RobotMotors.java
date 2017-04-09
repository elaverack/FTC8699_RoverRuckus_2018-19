package org.firstinspires.ftc.teamcode.robotHandlers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robot.Robot;

import java.util.LinkedHashMap;

/**
 * Created by Chandler on 3/8/2017.
 */

public class RobotMotors {

    LinkedHashMap<String, DcMotor> motors = new LinkedHashMap<>();
    HardwareMap hardwareMap;

    RobotMotors(){}
    public RobotMotors (HardwareMap hm) {this.hardwareMap = hm;}
    @Deprecated public RobotMotors (HardwareMap hm, String motorName, DcMotor motor) {
        this.hardwareMap = hm; addMotor(motorName, motor);
    }
    @Deprecated public RobotMotors (HardwareMap hm, String[] motorNames, DcMotor[] motors) {
        this.hardwareMap = hm; addMotors(motorNames, motors);
    }
    public RobotMotors (HardwareMap hm, String motorName) {
        this.hardwareMap = hm; addMotor(motorName, hardwareMap.dcMotor.get(motorName));
    }
    public RobotMotors (HardwareMap hm, String[] motorNames) {
        this.hardwareMap = hm; addMotors(motorNames);
    }

    @Deprecated public void addMotor (String motorName, DcMotor motor) {this.motors.put(motorName, motor);}
    @Deprecated public void addMotors (String[] motorNames, DcMotor[] motors) {
        if (motorNames.length != motors.length) return;
        for (int i = 0; i < motorNames.length; i++) {addMotor(motorNames[i], motors[i]);}
    }
    public void addMotor (String motorName) {this.motors.put(motorName, hardwareMap.dcMotor.get(motorName));}
    public void addMotors (String[] motorNames) {
        for (String motorName : motorNames) {addMotor(motorName);}
    }

    public DcMotor getMotor (String motorName) {return motors.get(motorName);}
    public LinkedHashMap<String, DcMotor> getMotors (String[] motorNames) {
        LinkedHashMap<String, DcMotor> Return = new LinkedHashMap<>();
        for (String motorName:motorNames) {Return.put(motorName, getMotor(motorName));}
        return Return;
    }
    public LinkedHashMap<String, DcMotor> getAllMotors () {return motors;}

    public void setDirection (String motorName, DcMotorSimple.Direction direction) {
        motors.get(motorName).setDirection(direction);}
    public void setDirections (String[] motorNames, DcMotorSimple.Direction direction) {
        for (String motorName:motorNames) {setDirection(motorName, direction);}
    }
    public void setDirections (String[] motorNames, DcMotorSimple.Direction[] directions) {
        if (motorNames.length != directions.length) return;
        for (int i = 0; i < motorNames.length; i++) {setDirection(motorNames[i], directions[i]);}
    }
    public void setAllDirections (DcMotorSimple.Direction direction) {setDirections(getAllMotorNames(), direction);}

    public void setMode (String motorName, DcMotor.RunMode mode) {
        motors.get(motorName).setMode(mode);}
    public void setModes (String[] motorNames, DcMotor.RunMode mode) {
        for (String motorName:motorNames) {setMode(motorName, mode);}
    }
    public void setModes (String[] motorNames, DcMotor.RunMode[] modes) {
        if (motorNames.length != modes.length) return;
        for (int i = 0; i < motorNames.length; i++) {setMode(motorNames[i], modes[i]);}
    }
    public void setAllModes (DcMotor.RunMode mode) {setModes(getAllMotorNames(), mode);}

    public void setPower (String motorName, double power) {
        motors.get(motorName).setPower(power);}
    public void setPowers (String[] motorNames, double power) {
        for (String motorName:motorNames) {setPower(motorName, power);}
    }
    public void setPowers (String[] motorNames, double[] powers) {
        if (motorNames.length != powers.length) return;
        for (int i = 0; i < motorNames.length; i++) {setPower(motorNames[i], powers[i]);}
    }
    public void setAllPowers (double power) {setPowers(getAllMotorNames(), power);}

    public void stopMotor (String motorName) {setPower(motorName, 0);}
    public void stopMotors (String[] motorNames) {for (String motorName:motorNames) {stopMotor(motorName);}}
    public void stopAll () {setAllPowers(0);}


    String[] getAllMotorNames () {return this.motors.keySet().toArray(new String[]{});}


}
