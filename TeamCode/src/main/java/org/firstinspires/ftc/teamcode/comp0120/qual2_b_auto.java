package org.firstinspires.ftc.teamcode.comp0120;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.util.ElapsedTime;

// Created on 1/20/2018 at 2:49 AM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode

@Autonomous(name = "QUAL2_B_AUTO", group = "Linear Opmode")
@Disabled
public class qual2_b_auto extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    unnamed_v2 robot;
    DeviceInterfaceModule dim;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new unnamed_v2(this);
        robot.auto_init();
        dim = hardwareMap.deviceInterfaceModule.get("dim");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();

        robot.j_down();
        robot.c.enableLed(true);

        while (runtime.seconds() < 5) {
            //robot.c.enableLed(((int)(runtime.seconds()%2) == 1));
            dim.setLED(0, ((int)(runtime.seconds()%2) == 1)); //blue
            dim.setLED(1, ((int)(runtime.seconds()%2) == 0)); //red
        }

        if (robot.c.red() > robot.c.blue()) {
            while (robot.gyro.getHeading() < 30) {
                robot.rf.setPower(.5);
                robot.rb.setPower(.5);
                robot.lf.setPower(-.5);
                robot.lb.setPower(-.5);
            }
            robot.stop_drive();
        } else if (robot.c.blue() > robot.c.red()) {
            while (robot.gyro.getHeading() < 5 || robot.gyro.getHeading() > 330 ) {
                robot.rf.setPower(-.5);
                robot.rb.setPower(-.5);
                robot.lf.setPower(.5);
                robot.lb.setPower(.5);
            }
            robot.stop_drive();
        } else {
            robot.j_up();
        }

        while (opModeIsActive()) {


            telemetry.addData("Status", "Done. Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
