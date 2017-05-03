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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robots.jorge.autonomous.JorgeAutonomousFunctions;
import org.firstinspires.ftc.teamcode.robotHandlers.Vuforia2017Manager;
import org.firstinspires.ftc.teamcode.robots.jorge.AutonomousJorge;

// Created on 4/13/2017 at 8:08 PM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode

@Autonomous(name = "OldAutoFullBeaconTest", group = "Linear Opmode")
@Disabled
public class AutoFullBeaconTest extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private AutonomousJorge jorge;

    @Override
    public void runOpMode() throws InterruptedException {

        jorge = new AutonomousJorge(this);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();

        JorgeAutonomousFunctions.GO_TO_WHITE_LINE (jorge, AutonomousJorge.COLOR_SENSOR.MIDDLE);

        JorgeAutonomousFunctions.STRAIGHTEN_ON_WHITE_LINE (jorge, AutonomousJorge.COLOR_SENSOR.BACK, AutonomousJorge.COLOR_SENSOR.MIDDLE);

        JorgeAutonomousFunctions.PRESS_BEACON (jorge, 1);

        runtime.reset();

        jorge.vuforia.checkOnBeacons(AutonomousJorge.BEACON.B1.index);

        if (jorge.vuforia.getBeaconConfig(AutonomousJorge.BEACON.B1.index) == Vuforia2017Manager.BLUE_BLUE) {
            while (runtime.seconds() < 4) {
                telemetry.addData("Status", "Wrong color. Waiting to press...");
                telemetry.update();
            }
            JorgeAutonomousFunctions.PRESS_BEACON(jorge, 1);
        }

        while (opModeIsActive()) {telemetry.addData("Status", "Done. Run Time: " + runtime.toString()); telemetry.update();}
    }
}
