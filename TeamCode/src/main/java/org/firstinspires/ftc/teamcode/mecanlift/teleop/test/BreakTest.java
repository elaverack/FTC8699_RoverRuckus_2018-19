package org.firstinspires.ftc.teamcode.mecanlift.teleop.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

// Created on 4/18/2018 at 9:17 PM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode.mecanlift.teleop.test

@TeleOp(name = "BreakTest", group = "test")
//@Disabled
public class BreakTest extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DeviceInterfaceModule dim;
    private TouchSensor beam;
    private boolean ledOn = false;

    @Override
    public void init() {

        dim = hardwareMap.deviceInterfaceModule.get("dim");
        beam = hardwareMap.touchSensor.get("beam");

        dim.setDigitalChannelMode(0, DigitalChannel.Mode.OUTPUT);
        dim.setDigitalChannelState(0, false);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() { runtime.reset(); }

    @Override
    public void loop() {

        if (ledOn && beam.isPressed()) {
            dim.setDigitalChannelState(0, false);
            ledOn = false;
        } else if (!ledOn && !beam.isPressed()) {
            dim.setDigitalChannelState(0, true);
            ledOn = true;
        }

        telemetry.addData("Status", "Running: " + runtime.toString());
    }

    @Override
    public void stop() {
    }

}