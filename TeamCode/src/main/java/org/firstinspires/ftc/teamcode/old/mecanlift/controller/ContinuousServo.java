package org.firstinspires.ftc.teamcode.old.mecanlift.controller;

// Created on 2/16/2018 at 9:23 PM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode.old.mecanlift.controller

import com.qualcomm.robotcore.hardware.Servo;

public class ContinuousServo {

    private Servo s;

    public ContinuousServo (Servo crServo) {
        s = crServo;
    }

    public void stop() { s.setPosition(.5); }

    public void setPower(double power) {
        if (power > 1) {power = 1.0;} else if (power < -1) {power = -1.0;}
        power = (power + 1.0)/2.0;
        s.setPosition(power);
    }

    public double getPower() {
        double position = s.getPosition();
        return (position*2.0) - 1.0;
    }

}
