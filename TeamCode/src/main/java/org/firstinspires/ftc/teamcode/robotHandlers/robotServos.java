package org.firstinspires.ftc.teamcode.robotHandlers;

import com.qualcomm.robotcore.hardware.Servo;

import java.util.LinkedHashMap;

/**
 * Created by Chandler on 2/19/2017.
 */

public class RobotServos {

    private LinkedHashMap<String, Servo> servos = new LinkedHashMap<>();

    public RobotServos() {}
    public RobotServos(String[] servoNames, Servo[] servos) {addServos(servoNames, servos);}
    public RobotServos(LinkedHashMap<String, Servo> servos) {addServos(servos);}

    public void addServo (String servoName, Servo servo) {servos.put(servoName, servo);}
    public void addServos (String[] servoNames, Servo[] servos) {
        if (servoNames.length != servos.length) return;
        for (int i = 0; i < servoNames.length; i++) {addServo(servoNames[i], servos[i]);}
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
    public void setAllPositions (double position) {setPositions(servos.keySet().toArray(new String[0]), position);}

}
