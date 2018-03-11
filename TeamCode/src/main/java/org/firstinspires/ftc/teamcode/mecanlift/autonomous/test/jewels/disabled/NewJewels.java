package org.firstinspires.ftc.teamcode.mecanlift.autonomous.test.jewels.disabled;

import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.visuals.VisualsHandler;
import org.opencv.core.Point;

// Created on 2/23/2018 at 10:23 PM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode.mecanlift.autonomous.test.jewels

@Autonomous(name = "Jewel Test", group = "test")
@Disabled
public class NewJewels extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private VisualsHandler visuals;
    private int picID = 0;

    @Override
    public void init() {
        visuals = new VisualsHandler(this, false);

        VisualsHandler.phoneLightOn();

        visuals.right.center = new Point(621, 1095);
        visuals.right.radius = 80;
        visuals.left.center = new Point(378, 1107);
        visuals.left.radius = 80;
        visuals.showAlignmentCircles();

        visuals.getPreview().setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) { visuals.takeAndSavePic("new_jewel_test.png"); picID++; }
        });

        telemetry.addData("Status", "Initialized");
        //runtime.reset();
    }

    @Override
    public void init_loop() {
//        tele("Seeing?", checkColumn());
//        if (gettingem) {
//            tele("Gotem", gotem);
//            VuforiaHandler.PosRot pr = visuals.vuforia.getRelativePosition();
//            if (pr.position.z != 0 && lastKnownPos == null) {
//                lastKnownPos = pr;
//                gotem = true;
//            } else if (pr.position.z != 0) {
//                lastKnownPos.doAverage(pr);
//                lastKnownPos.position.teleout(opmode, "Pos");
//                lastKnownPos.rotation.teleout(opmode, "Rot");
//            } else if (gotem) {
//                lastKnownPos = null;
//                gotem = false;
//            }
//        } else showAligning();
//        teleup();
//        public void showAligning () { if (((int)(time.seconds()%2) == 1)) visuals.showAlignmentCircles(); }
    }

    @Override
    public void start() { runtime.reset(); }

    @Override
    public void loop() {
        if (((int)(runtime.seconds()%2) == 1)) visuals.showAlignmentCircles();

        telemetry.addData("Status", "Saved " + picID);
    }

    @Override
    public void stop() { visuals.close(); }

}