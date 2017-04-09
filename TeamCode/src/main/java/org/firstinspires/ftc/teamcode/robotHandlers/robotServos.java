package org.firstinspires.ftc.teamcode.robotHandlers;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.LinkedHashMap;

/**
 * Created by Chandler on 2/19/2017.
 */

public class RobotServos {

    private LinkedHashMap<String, Servo> servos = new LinkedHashMap<>();
    private HardwareMap hardwareMap;

    protected RobotServos() {}
    //public RobotServos(String[] servoNames, Servo[] servos) {addServos(servoNames, servos);}
    //public RobotServos(LinkedHashMap<String, Servo> servos) {addServos(servos);}
    public RobotServos (HardwareMap hm, String servoName) {this.hardwareMap = hm; addServo(servoName);}
    public RobotServos (HardwareMap hm, String[] servoNames) {this.hardwareMap = hm; addServos(servoNames);}

    @Deprecated public void addServo (String servoName, Servo servo) {servos.put(servoName, servo);}
    @Deprecated public void addServos (String[] servoNames, Servo[] servos) {
        if (servoNames.length != servos.length) return;
        for (int i = 0; i < servoNames.length; i++) {addServo(servoNames[i], servos[i]);}
    }
    public void addServo (String servoName) {servos.put(servoName, hardwareMap.servo.get(servoName));}
    public void addServos (String[] servoNames) {
        for (String servoName : servoNames) {addServo(servoName);}
    }
    public void addServos (LinkedHashMap<String, Servo> servos) {this.servos.putAll(servos);}

    public Servo getServo (String servoName) {return servos.get(servoName);}
    public LinkedHashMap<String, Servo> getServos (String[] servoNames) {
        LinkedHashMap<String, Servo> Return = new LinkedHashMap<>();
        for (String servoName:servoNames) {Return.put(servoName, getServo(servoName));}
        return Return;
    }
    public LinkedHashMap<String, Servo> getAllServos () {return servos;}

    public void setPosition (String servoName, double position) {servos.get(servoName).setPosition(position);}
    public void setPositions (String[] servoNames, double position) {
        for (String servoName:servoNames) {setPosition(servoName, position);}
    }
    public void setPositions (String[] servoNames, double[] positions) {
        if (servoNames.length != positions.length) return;
        for (int i = 0; i < servoNames.length; i++) {setPosition(servoNames[i], positions[i]);}
    }
    public void setAllPositions (double position) {setPositions(getAllNames(), position);}

    public double getPosition (String servoName) {return servos.get(servoName).getPosition();}
    public double[] getPositions (String[] servoNames) {
        double[] Return = new double[servoNames.length];
        for (int i = 0; i < servoNames.length; i++) {Return[i] = getPosition(servoNames[i]);}
        return Return;
    }
    public double[] getAllPositions () {return getPositions(getAllNames());}

    private String[] getAllNames () {
        return servos.keySet().toArray(new String[]{});
    }

}
