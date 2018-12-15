package org.firstinspires.ftc.teamcode.frontier.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Enums;
import org.firstinspires.ftc.teamcode.frontier.control.Frontier;

// Created on 12/14/2018 at 1:36 PM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode.frontier.autonomous

@Autonomous(name = "DEPOT_AUTO", group = "comp")
//@Disabled
public class DEPOT_AUTO extends LinearOpMode {
    
    private static final Enums.FieldPosition POS = Enums.FieldPosition.DEPOT;
    
    private Frontier robot;
    
    @Override
    public void runOpMode() throws InterruptedException {
        
        robot = new Frontier(this, POS);
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        robot.waitForStart();
        
        robot.start();
        
        robot.auto();
        
        while (opModeIsActive()) {
            
            telemetry.addData("Status", "Done.");
            telemetry.update();
            
        }
    }
}