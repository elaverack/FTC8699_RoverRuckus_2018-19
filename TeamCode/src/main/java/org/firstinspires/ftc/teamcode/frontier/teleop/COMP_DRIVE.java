package org.firstinspires.ftc.teamcode.frontier.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Enums;
import org.firstinspires.ftc.teamcode.frontier.control.Frontier;

// Created on 12/14/2018 at 1:26 PM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode.frontier.teleop

@TeleOp(name = "COMP DRIVE", group = "comp")
@Disabled
public class COMP_DRIVE extends OpMode {
    
    Frontier robot;
    
    @Override
    public void init() {
        robot = new Frontier(this, Enums.FieldPosition.ERROR);
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() { robot.start(); }

    @Override
    public void loop() { robot.drive(); }

    @Override
    public void stop() { robot.stop(); }

}