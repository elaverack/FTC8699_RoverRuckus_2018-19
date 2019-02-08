package org.firstinspires.ftc.teamcode.new_frontier.teleop.test;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Enums.Mineral;

// Created on 1/6/2019 at 12:24 PM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode.frontier.teleop.test

@TeleOp(name = "Roller Test 2", group = "test")
//@Disabled
public class RollerTest2 extends OpMode {
    
    private ModernRoboticsI2cColorSensor c;
    private CRServo r;
    //private boolean grabbed = false, doa = false;
    private boolean pulling = false, pushing = false, done = false, doa = false, donepass = false;
    private static final double INTAKE = .5, OUTTAKE = -.5;
    private Mineral grabbed = Mineral.ERROR;
    private ElapsedTime runtime;
    private double doneTime = 0;
    
    @Override
    public void init() {
        
        runtime = new ElapsedTime();
        
        c = (ModernRoboticsI2cColorSensor) hardwareMap.colorSensor.get("color");
        c.enableLed(true);
        
        r = hardwareMap.crservo.get("roller");
        r.setPower(0);
        
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        r.setPower(INTAKE);
        runtime.reset();
    }

    @Override
    public void loop() {
        
//        if (c.read8(ModernRoboticsI2cColorSensor.Register.COLOR_INDEX) != 0 && !grabbed) {
//            s.setPosition(.5);
//            grabbed = true;
//        }
//        if (grabbed && gamepad1.a) {
//            s.setPosition(.2);
//            doa = true;
//        }
//        if (grabbed && !gamepad1.a && doa) {
//            s.setPosition(.7);
//            doa = false;
//            grabbed = false;
//        }
        byte read = c.read8(ModernRoboticsI2cColorSensor.Register.COLOR_INDEX);
        if (gamepad1.a && !doa) {
            r.setPower(OUTTAKE);
            doa = true;
        } else if (doa && !gamepad1.a) {
            r.setPower(INTAKE);
            grabbed = Mineral.ERROR;
            pulling = false;
            pushing = false;
            done = false;
            doa = false;
        } else if (!doa && !done) {
            if (read != 0 && grabbed == Mineral.ERROR) { // if we are seeing something and don't already have something
                pulling = true;
                grabbed = Mineral.identify(read);
            } else if (pulling && read == 0) { // if we are pulling one through and don't see it anymore
                pulling = false;
            } else if (!pulling && !pushing && read != 0) { // if we aren't pulling or pushing and we are seeing something
                if (Mineral.identify(read) != grabbed) {
                    r.setPower(OUTTAKE);
                    pushing = true;
                } else {
                    done = true;
                }
            } else if (pushing && Mineral.identify(read) == grabbed) { // if we are pushing one away and we are seeing the one we were holding
                pulling = true;
                pushing = false;
                r.setPower(INTAKE);
            }
        } else if (done) {
            if (read == 0 && !donepass) {
                r.setPower(OUTTAKE / 2);
                donepass = true;
            } else if (donepass) {
                r.setPower(0);
            }
        }
        
        telemetry.addData("color", (int)read);
        telemetry.addData("red", c.red());
        telemetry.addData("green", c.green());
        telemetry.addData("blue", c.blue());
        telemetry.addData("alpha", c.alpha());
        //telemetry.addData("grabbed")
//        if (grabbed) {
//            if (c.read8(ModernRoboticsI2cColorSensor.Register.COLOR_INDEX) > 50) telemetry.addData("type", "gold");
//            else telemetry.addData("type", "silver");
//        }
        telemetry.addData("grabbed", grabbed);
        telemetry.addData("pulling", pulling);
        telemetry.addData("pushing", pushing);
        telemetry.addData("done", done);
        telemetry.addData("doa", doa);
        
    }

    @Override
    public void stop() {
        
        c.enableLed(false);
        //s.setPosition(.5);
        r.setPower(0);
        
    }

}