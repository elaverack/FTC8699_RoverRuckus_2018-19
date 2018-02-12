package org.firstinspires.ftc.teamcode.mecanlift.autonomous.test.movement;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.teamcode.visuals.VuforiaHandler;

// Created on 1/27/2018 at 3:22 PM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode.autonomous

@Autonomous(name = "Navigation Test", group = "test")
//@Disabled
public class NavTest extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private VuforiaHandler vuforia;

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");
        telemetry.update();
        vuforia = new VuforiaHandler(this);
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() { }

    @Override
    public void start() { runtime.reset(); vuforia.start(); CameraDevice.getInstance().setFlashTorchMode(true); }

    @Override
    public void loop() {
        VuforiaHandler.PosRot pr = vuforia.getRelativePosition();
        double newX = pr.position.x + Math.abs(pr.position.z)*Math.tan(Math.toRadians(pr.rotation.y));
        if (pr.rotation.y > 0) newX *= -1;
        pr.position.teleout(this, "pos");
        pr.rotation.teleout(this, "rot");
        telemetry.addData("new x", newX);
        telemetry.addData("Status", "Running: " + runtime.toString());
    }

    @Override
    public void stop() { CameraDevice.getInstance().setFlashTorchMode(false); }

}