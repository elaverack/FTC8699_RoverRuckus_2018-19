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
package org.firstinspires.ftc.teamcode.archive.season2016_2017.chandler_preparingForStates2017.chandler_CameraTesting;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.archive.season2016_2017.robotCoreFunctions.ImageAnalyst;

import java.io.File;
import java.io.FileInputStream;


// Created on 2/3/2017 at 9:59 AM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode.archive.season2016_2017.chandler_preparingForStates2017.chandler_betaOpmodes

@TeleOp(name = "PhotoTest2", group = "Linear Opmode")
// @Autonomous(...) is the other common choice
@Disabled
public class BETA_Photo2 extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    private final File PHOTO_DIRECTORY = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOWNLOADS);
    private final int RED_TOLERANCE = 70;
    private final int BLUE_TOLERANCE = 55;
    private final int[] photoNames = {1, 2, 3, 4};
    private int[][] values = new int[4][2];


    @Override
    public void runOpMode() {
        /*File[] Photos = new File[4];
        Bitmap[] photos = new Bitmap[4];
        ImageAnalyst[] imageAnalysts = new ImageAnalyst[4];

        for (int i = 0; i < photoNames.length; i++) {
            Photos[i] = new File(PHOTO_DIRECTORY, photoNames[i] + ".jpg");
            photos[i] = BitmapFactory.decodeFile(Photos[i].getAbsolutePath());
            imageAnalysts[i] = new ImageAnalyst(photos[i]);
            values[i][0] = imageAnalysts[i].analyzeDistribution(imageAnalysts[i].getVerticalDistribution('R', RED_TOLERANCE));
            values[i][1] = imageAnalysts[i].analyzeDistribution(imageAnalysts[i].getVerticalDistribution('B', BLUE_TOLERANCE));
        }*/

        int one = 6969;
        int two = 6969;

        runtime.reset();
        File photoFile = new File(PHOTO_DIRECTORY, "1.jpg");
        if (photoFile.exists()) {
            //Bitmap photo = BitmapFactory.decodeFile(photoFile.getPath());
            FileInputStream f;
            try {
                f = new FileInputStream(photoFile);
            } catch (Exception e) {
                f = null;
            }
            Bitmap photo1 = BitmapFactory.decodeStream(f);
            try {
                f.close();
            } catch (Exception e) {

            }
            photo1 = Bitmap.createScaledBitmap(photo1, (int) photo1.getWidth()/8, (int) photo1.getHeight()/8, true);
            telemetry.addData("Status", "Decode done. Processing...");
            telemetry.update();
            ImageAnalyst imageAnalyst = new ImageAnalyst(photo1);
            one = imageAnalyst.analyzeDistribution(imageAnalyst.getVerticalDistribution('R', RED_TOLERANCE));
            two = imageAnalyst.analyzeDistribution(imageAnalyst.getVerticalDistribution('B', BLUE_TOLERANCE));
        }
        double processTime = runtime.seconds();

        telemetry.addData("Status", processTime);
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Photo 1", "RED: " + one + ", BLUE: " + two);
            //telemetry.addData("Photo 2", "RED: " + values[1][0] + ", BLUE: " + values[1][0]);
            //telemetry.addData("Photo 3", "RED: " + values[2][0] + ", BLUE: " + values[2][0]);
            //telemetry.addData("Photo 4", "RED: " + values[3][0] + ", BLUE: " + values[3][0]);
            telemetry.update();
        }
    }

}
