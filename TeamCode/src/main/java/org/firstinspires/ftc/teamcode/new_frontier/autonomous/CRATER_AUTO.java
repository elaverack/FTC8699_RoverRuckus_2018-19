package org.firstinspires.ftc.teamcode.new_frontier.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Enums;
import org.firstinspires.ftc.teamcode.new_frontier.control.NewFrontier;

// Created on 1/28/2019 at 9:26 AM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode.new_frontier.autonomous

@Autonomous(name = "CRATER_AUTO", group = "comp")
//@Disabled
public class CRATER_AUTO extends LinearOpMode {
    
    NewFrontier robot;
    
    @Override
    public void runOpMode() {
        
        robot = new NewFrontier(this, Enums.FieldPosition.CRATER);
        
        robot.waitForStart();
        
        robot.start();
        
        robot.auto();
        
        while (opModeIsActive()) {
            
            telemetry.addData("Status", "Done.");
            telemetry.update();
            
        }
        
        robot.autoStop();
    }
}