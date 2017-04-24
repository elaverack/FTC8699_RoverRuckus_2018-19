package org.firstinspires.ftc.teamcode.archive.season2016_2017.robotCoreFunctions;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.text.MessageFormat;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.Set;

import com.qualcomm.ftccommon.DbgLog;

/**
 * Created by Rama on 12/22/2015.
 */
public class MotorHandler {
//Handles basic functions of the robot movement

    LinkedHashMap<String, DcMotor> motors = new LinkedHashMap<String, DcMotor>();
    HardwareMap hm;
    float wheelCircumference;
    public int rightGoal;
    public int leftGoal;


    public MotorHandler(HardwareMap hardwareMap, float wheelDiameter) {
        hm = hardwareMap;
        wheelCircumference = (float) Math.PI * wheelDiameter;
    }

    public MotorHandler(HardwareMap hardwareMap, String motor, float wheelDiameter) {
        hm = hardwareMap;
        wheelCircumference = (float) Math.PI * wheelDiameter;
        motors.put(motor, hm.dcMotor.get(motor));
    }

    public MotorHandler(HardwareMap hardwareMap, String[] motors, float wheelDiameter) {
        hm = hardwareMap;
        wheelCircumference = (float) Math.PI * wheelDiameter;
        for (int i = 0; i < motors.length; i++) {
            this.motors.put(motors[i], hm.dcMotor.get(motors[i]));
        }
    }

    public void AddMotor(String motorName) {
        motors.put(motorName, hm.dcMotor.get(motorName));
    }

    public void AddMotors(String[] motorNames) {
        for (int i = 0; i < motorNames.length; i++) {
            motors.put(motorNames[i], hm.dcMotor.get(motorNames[i]));
        }
    }

    public void SetDirection(String motorName, DcMotor.Direction direction) {
        motors.get(motorName).setDirection(direction);
    }

    public void SetDirection(String[] motorNames, DcMotor.Direction direction) {
        for (int i = 0; i < motorNames.length; i++) {
            motors.get(motorNames[i]).setDirection(direction);
        }
    }

    public void SetDirection(String[] motorNames, DcMotor.Direction[] directions) {
        for (int i = 0; i < motorNames.length; i++) {
            motors.get(motorNames[i]).setDirection(directions[i]);
        }
    }

    public void setMotor(String motorName, double power) {
        motors.get(motorName).setPower(power);
    }

    public void setMotorsPower(String[] motorNames, double power) {
        for (int i = 0; i < motorNames.length; i++) {
            motors.get(motorNames[i]).setPower(power);
        }
    }

    public void setMotorsPower(String[] motorNames, double[] motorPowers) {
        for (int i = 0; i < motorNames.length; i++) {
            motors.get(motorNames[i]).setPower(motorPowers[i]);
        }
    }

    public void setAllMotorsPower(double power) {
        Set keys = motors.keySet();
        for (Iterator<String> i = keys.iterator(); i.hasNext(); ) {
            motors.get(i.next()).setPower(power);
        }
    }

    public void setRunMode(String motorName, DcMotor.RunMode runMode) {
        motors.get(motorName).setMode(runMode);
    }

    public void setRunMode(String[] motorNames, DcMotor.RunMode runMode) {
        for (int i = 0; i < motorNames.length; i++) {
            DbgLog.msg("MotorHandler: setting motor " + motorNames[i] + " to RunMode " + runMode);
            motors.get(motorNames[i]).setMode(runMode);
        }
    }

    public void setRunMode(String[] motorNames, DcMotor.RunMode[] runModes) {
        for (int i = 0; i < motorNames.length; i++) {
            motors.get(motorNames[i]).setMode(runModes[i]);
        }
    }

    public void setAllRunModes(DcMotor.RunMode runMode) {
        Set keys = motors.keySet();
        for (Iterator<String> i = keys.iterator(); i.hasNext(); ) {
            motors.get(i.next()).setMode(runMode);
        }
    }

    public int readEncoder(String motorName) {
        return motors.get(motorName).getCurrentPosition();
    }

    public int[] readEncoders(String[] motorNames) {
        int[] values = new int[motorNames.length];
        for (int i = 0; i < motorNames.length; i++) {
            values[i] = motors.get(motorNames[i]).getCurrentPosition();
        }
        return values;
    }

    public long moveDistanceEstimation(float distance) {

        int rpm = 105;
        float InchesPerMinute = wheelCircumference * rpm;
        float millisecondsToMove = ((distance) / InchesPerMinute) * 60000f;

        return (long) millisecondsToMove;

    }

    public long moveDistanceEstimation(double feet) {

        int rpm = 105;
        float InchesPerMinute = wheelCircumference * rpm;
        float millisecondsToMove = ((float) (feet * 12) / InchesPerMinute) * 60000;

        return (long) millisecondsToMove;

    }

    public int moveDistanceEncoder(float distance, String[] drivetrain) {
        setRunMode(drivetrain, DcMotor.RunMode.RESET_ENCODERS);
        //DbgLog.msg(MessageFormat.format("moveDistanceEncoder: Set {0} to run mode {1}", Arrays.toString(drivetrain), getMotor(drivetrain[0]).getMode()));
        setRunMode(drivetrain, DcMotor.RunMode.RUN_TO_POSITION);
        //DbgLog.msg(MessageFormat.format("moveDistanceEncoder: Set {0} to run mode {1}", Arrays.toString(drivetrain), getMotor(drivetrain[0]).getMode()));

        int encoderUntilDistance = (int) ((distance / wheelCircumference) * 1680);
        DbgLog.msg(MessageFormat.format("moveDistanceEncoder: Encoder goal calculated to be {0}", encoderUntilDistance));

        for (int i = 0; i < drivetrain.length; i++) {
            DcMotor m = motors.get(drivetrain[i]);
            m.setTargetPosition(encoderUntilDistance);
            DbgLog.msg(MessageFormat.format("moveDistanceEncoder: Target position of {0} set to {1}", drivetrain[i], m.getTargetPosition()));
        }
        return encoderUntilDistance;
    }

   /*
   -- Nearly the same as above method - no need to have both.

   public int moveDisEncoder(float distance, String[] drivetrain) {

        setRunMode(drivetrain, DcMotor.RunMode.RESET_ENCODERS);
        DbgLog.msg(MessageFormat.format("moveDistanceEncoder: Set {0} to run mode {1}", drivetrain, getMotor(drivetrain[0]).getMode()));
        setRunMode(drivetrain, DcMotor.RunMode.RUN_TO_POSITION);
        DbgLog.msg(MessageFormat.format("moveDistanceEncoder: Set {0} to run mode {1}", drivetrain, getMotor(drivetrain[0]).getMode()));

        int encoderUntilDistance = (int) ((distance / wheelCircumference) * 1680);
        DbgLog.msg(MessageFormat.format("moveDistanceEncoder: Encoder goal calculated to be {0}", encoderUntilDistance));
        for (int i = 0; i < drivetrain.length; i++) {
            motors.get(drivetrain[i]).setTargetPosition(encoderUntilDistance);
            DbgLog.msg(MessageFormat.format("moveDistanceEncoder: Target position of {0} set to {1}", drivetrain[i], getMotor(drivetrain[i]).getTargetPosition()));
        }
        return encoderUntilDistance;
    }*/

    public DcMotor getMotor(String motorName) {
        return motors.get(motorName);
    }

    public DcMotor[] getMotor(String[] motorNames) {
        DcMotor[] motors = new DcMotor[motorNames.length];
        for (int i = 0; i < motorNames.length; i++) {
            motors[i] = this.motors.get(motorNames[i]);
        }
        return motors;
    }

    public boolean checkMotor(String motorName) {
        return (motors.get(motorName).getCurrentPosition() >= motors.get(motorName).getTargetPosition());
    }

    public boolean checkMotor(String motorName, int goal) {
        return (motors.get(motorName).getCurrentPosition() >= goal);
    }

    public boolean checkMotor(String motorName, int goal, int range) {
        return ((goal - range < motors.get(motorName).getCurrentPosition()) && (goal + range > motors.get(motorName).getCurrentPosition()));
    }

    public boolean checkAvg(String[] motorNames, int goal, int range) {
        double sum = 0;
        for (int i = 0; i < motorNames.length; i++) {
            sum += (double) motors.get(motorNames[i]).getCurrentPosition();
        }
        double avg = sum / motorNames.length;
        return ((goal - range < avg) && (goal + range > avg));
    }
    //4 inch wheel
}