package org.firstinspires.ftc.teamcode;

// Created on 1/26/2018 at 3:41 PM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.Servo;

public class ToggleServo {

    private double c, o;
    private Servo s;
    private boolean
            buttoned = false,
            opened = false,
            closed = false;

    public ToggleServo (Servo servo, double open_pos, double close_pos)
        { s = servo; o = open_pos; c = close_pos; }

    public void close () { s.setPosition(c); }
    public void open () { s.setPosition(o); }
    public void toggle () { if (s.getPosition() == o) { close(); } }

    /** tob = toggle on button */
    public void tob (boolean button) {
        if (button && !buttoned) { toggle(); buttoned = true; }
        else if (!button && buttoned) buttoned = false;
    }

    /** owb = open with button */
    public void owb (boolean button) {
        if (button && !opened) { open(); opened = true; return; }
        if (!button && opened) { close(); opened = false; }
    }

    /** cwb = close with button */
    public void cwb (boolean button) {
        if (button && !closed) { close(); closed = true; return; }
        if (!button && closed) { open(); closed = false; }
    }

}
