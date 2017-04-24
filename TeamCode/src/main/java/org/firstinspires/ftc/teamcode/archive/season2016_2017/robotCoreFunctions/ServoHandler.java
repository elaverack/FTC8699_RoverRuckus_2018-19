package org.firstinspires.ftc.teamcode.archive.season2016_2017.robotCoreFunctions;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Servo.Direction;

import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.Set;

/**
 * Created by Chandler on 1/10/2016.
 */
public class ServoHandler {

    LinkedHashMap<String, Servo> servos  = new LinkedHashMap<String, Servo>();
    HardwareMap hm;

    public ServoHandler(HardwareMap hm) {
        this.hm = hm;
    }

    public ServoHandler(HardwareMap hm, String ServoName) {
        this.hm = hm;
        servos.put(ServoName, hm.servo.get(ServoName));
    }

    public ServoHandler(HardwareMap hm, String[] ServoNames) {
        this.hm = hm;
        for (int i = 0; i < ServoNames.length; i++) {
            servos.put(ServoNames[i], hm.servo.get(ServoNames[i]));
        }
    }

    public void AddServo(String servoName) {
        servos.put(servoName, hm.servo.get(servoName));
    }

    public void AddServo(String[] ServoNames) {
        for (int i = 0; i < ServoNames.length; i++) {
            servos.put(ServoNames[i], hm.servo.get(ServoNames[i]));
        }
    }

    public void SetDirection(String servoName, Direction direction) {
        servos.get(servoName).setDirection(direction);
    }

    public void SetDirection(String[] servoNames, Direction direction) {
        for (int i = 0; i < servoNames.length; i++) {
            servos.get(servoNames[i]).setDirection(direction);
        }
    }

    public void SetDirection(String[] servoNames, Direction[] directions) {
        for (int i = 0; i < servoNames.length; i++) {
            servos.get(servoNames[i]).setDirection(directions[i]);
        }
    }

    public void SetServo(String servoName, double position) {
        servos.get(servoName).setPosition(position);
    }

    public void SetServo(String[] ServoNames, double postition) {
        for (int i = 0; i < ServoNames.length; i++) {
            servos.get(ServoNames[i]).setPosition(postition);
        }
    }

    public void SetServo(String[] ServoNames, double[] positions) {
        for (int i = 0; i < ServoNames.length; i++) {
            servos.get(ServoNames[i]).setPosition(positions[i]);
        }
    }

    public void SetAllServos(double position) {
        Set keys = servos.keySet();
        for (Iterator<String> i = keys.iterator(); i.hasNext(); ) {
            servos.get(i.next()).setPosition(position);
        }
    }



}
