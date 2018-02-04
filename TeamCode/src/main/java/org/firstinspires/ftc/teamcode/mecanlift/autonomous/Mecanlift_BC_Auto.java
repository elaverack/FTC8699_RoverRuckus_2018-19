package org.firstinspires.ftc.teamcode.mecanlift.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mecanlift.controller.Mecanlift;

// Created on 2/3/2018 at 12:41 AM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode.mecanlift.autonomous

@Autonomous(name = "QUAL_BC_AUTO", group = "Iterative Opmode")
//@Disabled
public class Mecanlift_BC_Auto extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private Mecanlift robot;
    //private final Mecanlift.FIELDPOS pos = Mecanlift.FIELDPOS.BLUE_CORNER;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Mecanlift(this, Mecanlift.Color.BLUE, Mecanlift.Position.CORNER);

        telemetry.addData("Status", "Align vert. line with edge of jewel and hor. line with bottom of vumark.");
        telemetry.update();
        while (!isStarted()) { robot.showAligning(); }
        runtime.reset();

        robot.doParkAutonomous(this);

        while (opModeIsActive()) {
            telemetry.addData("Status", "Done. Run Time: " + runtime.toString());
            telemetry.update();
        }

        //robot.closeVisuals();

    }
}
