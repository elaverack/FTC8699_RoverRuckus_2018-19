package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceReader;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.util.ElapsedTime;

// Created on 1/20/2018 at 2:07 AM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode

@TeleOp(name = "color_test", group = "Iterative Opmode")
@Disabled
public class color_test extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private ModernRoboticsI2cColorSensor color;
    private int red = -1;
    //private final I2cAddr color_id = new I2cAddr(0x04);


    @Override
    public void init() {
        color = (ModernRoboticsI2cColorSensor) hardwareMap.colorSensor.get("color");

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();


    }

    @Override
    public void loop() {
        if (color.red() > color.blue()) {
            red = 1;
        } else if (color.blue() > color.red()) {red = 0;} else red = -1;
        telemetry.addData("color", red);

        telemetry.addData("Status", "Running: " + runtime.toString());
    }

    @Override
    public void stop() {
    }

}
