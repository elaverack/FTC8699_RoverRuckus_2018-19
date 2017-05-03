/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode.archive.season2016_2017.testArchive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robots.jorge.Jorge;

// Created on 4/11/2017 at 11:43 AM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode

@Autonomous(name = "TestEncodedDistance", group = "Linear Opmode")
@Disabled
public class TestEncodedDistance extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private Jorge jorge;
    private final float IN_PER_REV = 19.125f;

    @Override
    public void runOpMode() throws InterruptedException {

        jorge = new Jorge(this);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();

        goFeetWithEncoders(2, 1);

        //Done.
        while (opModeIsActive()) {

            //jorge.drive();

            //int[] positions = jorge.drive.getAllPositions();
            telemetry.addData("Status", "Done. Run Time: " + runtime.toString());
            //int i = 1;
            //for (int position : positions) {
            //    telemetry.addData("" + i, "" + position); i++;
            //}
            telemetry.update();
        }
    }

    private int calcEncodersForFeet (float feet) {
        float distance = feet * 12;
        return Math.round((distance / IN_PER_REV) * 1680);
    }

    private void goFeetWithEncoders (float feet, double speed) {
        jorge.drive.setAllModes(DcMotor.RunMode.RUN_TO_POSITION);
        jorge.drive.setAllTargetPositions(calcEncodersForFeet(feet), speed);

        boolean Break = false;
        while (!Break) {Break = (jorge.drive.updateAllMotors() || !opModeIsActive());}
        jorge.stop();
    }
}
