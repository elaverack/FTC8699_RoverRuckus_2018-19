package org.firstinspires.ftc.teamcode.mecanlift.autonomous.test.alignment;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Enums;
import org.firstinspires.ftc.teamcode.visuals.VisualsHandler;

// Created on 2/24/2018 at 10:18 PM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode.mecanlift.autonomous.test.alignment

@Autonomous(name = "Crypto Realtime Blue", group = "test")
//@Disabled
public class CryptoRTBlue extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private VisualsHandler visuals;
    private int numDividers = 0;

    @Override
    public void init() {
        visuals = new VisualsHandler(this, false);
        VisualsHandler.phoneLightOn();

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {  }

    @Override
    public void start() { runtime.reset(); }

    @Override
    public void loop() {
        if (((int)(runtime.seconds()%3) == 0)) numDividers = visuals.previewCryptobox(Enums.Color.BLUE).size();
        telemetry.addData("Status", "Running: " + runtime.toString());
        telemetry.addData("Div", numDividers);
    }

    @Override
    public void stop() { visuals.close(); }

}