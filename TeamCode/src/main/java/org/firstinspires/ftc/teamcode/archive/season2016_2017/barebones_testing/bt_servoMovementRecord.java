package org.firstinspires.ftc.teamcode.archive.season2016_2017.barebones_testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.io.File;
import android.os.Environment;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.Enumeration;
import java.util.Vector;

// Created on 12/9/2016 at 9:13 PM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode.archive.season2016_2017.barebones_testing

@TeleOp(name = "bt_servoMovementRecord", group = "Iterative Opmode")
// @Autonomous(...) is the other common choice
@Disabled
public class bt_servoMovementRecord extends OpMode {
    //Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    //The servo and touch sensor that is used to record.
    Servo servo;
    TouchSensor sensor;

    //The list of values that are being saved and the file they are being saved to.
    Vector save = new Vector();
    File saveFile;

    //Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        //Initiation of servo and sensor. These are actually mapped in the configuration file.
        servo = hardwareMap.servo.get("servo");
        sensor = hardwareMap.touchSensor.get("sensor");

        /* Working file creation example.

        !! No need to read this if you're trying to understand how the code works.
        !! This is just for me to reference for future work.

        android.content.Context context = hardwareMap.appContext;

        file = new File(Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOWNLOADS) + "/Servo Test Saves");
        if (!file.mkdir()) {
            telemetry.addData("Error", "It didn't work, try again...");
        }
        file = new File(Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOWNLOADS) + "/Servo Test Saves", "congrats.txt");
        try {
            file.createNewFile();
        } catch (IOException e) {
            e.printStackTrace();
            telemetry.addData("Error", "Maybe not congrats...");
        }
        try {
            FileOutputStream f = new FileOutputStream(file);
            PrintWriter pw = new PrintWriter(f);
            pw.println("YAY! IT WORKED!");
            pw.flush();
            pw.close();
            f.close();
        } catch (FileNotFoundException e) {
            e.printStackTrace();
            telemetry.addData("Error", "Maybe not congrats...");
        } catch (IOException e) {
            e.printStackTrace();
            telemetry.addData("Error", "Maybe not congrats...");
        }
        */

        /* Initiation of save file. First it declares where the file will go (a folder I created in Downloads) and what the
        * file will be named (save2.txt -- save1.txt was my first success, but I had to change how it wrote to the file. I keep it for reference.).
        * It then creates the file ("saveFile.createNewFile();"). If there's an issue (catch IOException), it will notify the driver station phone
        * through telemetry ("telemetry.addData("Error", "Couldn't create file for some reason.");")
        */
        saveFile = new File(Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOWNLOADS) + "/Servo Test Saves", "save2.txt");
        try {
            saveFile.createNewFile();
        } catch (IOException e) {
            e.printStackTrace();
            telemetry.addData("Error", "Couldn't create file for some reason.");
        }
    }

    //Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
    @Override
    public void init_loop() {


    }

    //Code to run ONCE when the driver hits PLAY
    @Override
    public void start() {

        runtime.reset();

        //Get the servo ready for recording
        servo.setPosition(0);

        //Just for reference in the save file, it makes the first line read "START"
        save.addElement("START");

    }

    //Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {
        telemetry.addData("Status", "Running: " + runtime.toString());

        //If I press the touch sensor, move the servo. This is so that I don't have to bother with a controller
        if (sensor.isPressed()) {
            servo.setPosition(1);
        } else {
            servo.setPosition(0);
        }

        //Just keeping track of servo position
        telemetry.addData("Servo Position", servo.getPosition());

        //Add both the current runtime and the servo position to the list of values
        save.addElement("{" + runtime.seconds() + "}, {" + servo.getPosition() + "}");

    }

    //Code to run ONCE after the driver hits STOP
    @Override
    public void stop() {

        //Just another reference in the save file.
        save.addElement("STOP");

        try {
            //Open up file to write to
            FileOutputStream f = new FileOutputStream(saveFile);
            PrintWriter pw = new PrintWriter(f);
            Enumeration e = save.elements();

            /* Using enumeration (I don't even know exactly what that is), write all the values to
            * the text file line by line.
            */
            while (e.hasMoreElements()) {
                pw.println(e.nextElement().toString());
            }
            pw.flush();
            pw.close();
            f.close();
        } catch (FileNotFoundException e) {
            //If the file doesn't exist, do something that I don't think works.
            e.printStackTrace();
            telemetry.addData("Error", "I wonder if this shows up");
        } catch (IOException e) {
            //Literally the same as before
            e.printStackTrace();
            telemetry.addData("Error", "I wonder if this shows up");
        }

    }

}
