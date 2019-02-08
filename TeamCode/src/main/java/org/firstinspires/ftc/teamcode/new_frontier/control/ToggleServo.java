package org.firstinspires.ftc.teamcode.new_frontier.control;

// Created on 1/26/2018 at 3:41 PM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.Servo;

public class ToggleServo {

    private double c, o;
    private Servo s;
    private boolean
            buttoned = false,
            opened = false,
            closed = false;
    private Runnable runOnClose = new Runnable() {
        @Override
        public void run() {}
    }, runOnOpen = new Runnable() {
        @Override
        public void run() {}
    };

    public ToggleServo (Servo servo, double open_pos, double close_pos)
        { s = servo; o = open_pos; c = close_pos; }
    public ToggleServo (Servo servo, double open_pos, double close_pos, Runnable runOnClose, Runnable runOnOpen)
        { s = servo; o = open_pos; c = close_pos; this.runOnClose = runOnClose; this.runOnOpen = runOnOpen; }

    public void close () { s.setPosition(c); runOnClose.run(); }
    public void open () { s.setPosition(o); runOnOpen.run(); }
    public void toggle () { if (isOpened()) { close(); } else open(); }

    public boolean isOpened () { return s.getPosition() == o; }
    public void setOpen (boolean doopen) { if (doopen) {open();} else close(); }

    /** tob = toggle on button */
    public void tob (boolean button) {
        if (!button && buttoned) { buttoned = false; return; }
        if (!button) return;
        if (!buttoned) { toggle(); buttoned = true; }
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

    /**
     * Set the position of the servo between open and close.
     * @param pos Value between 0 and 1, 0 being open and 1 being closed.
     */
    public void setPos (double pos) { s.setPosition(o + pos*(c - o)); }



}
