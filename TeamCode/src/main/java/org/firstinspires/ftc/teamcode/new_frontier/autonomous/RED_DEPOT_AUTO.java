package org.firstinspires.ftc.teamcode.new_frontier.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

// Created on 1/28/2019 at 9:26 AM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode.new_frontier.autonomous

@Autonomous(name = "RED_DEPOT_AUTO", group = "comp")
//@Disabled
public class RED_DEPOT_AUTO extends LinearOpMode {
    
    
    
    @Override
    public void runOpMode() {
        
        
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        
        
        
        while (opModeIsActive()) {
            
            
            
        }
    }
}