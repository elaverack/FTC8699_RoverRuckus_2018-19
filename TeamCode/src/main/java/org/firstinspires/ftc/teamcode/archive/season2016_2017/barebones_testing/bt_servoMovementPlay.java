package org.firstinspires.ftc.teamcode.archive.season2016_2017.barebones_testing;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.Arrays;
import java.util.Vector;

// Created on 12/10/2016 at 10:02 AM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode.archive.season2016_2017.barebones_testing

@Autonomous(name = "bt_servoMovementPlay", group = "Iterative Opmode")
// @Autonomous(...) is the other common choice
@Disabled
public class bt_servoMovementPlay extends OpMode {
    //Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    //The servo and touch sensor that is used to record.
    Servo servo;
    TouchSensor sensor;

    //The file to load from, the array to store the values in and the counter that will aid in reading that array
    File loadFile;
    double[][] values;
    int counter = 0;


    //Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        //Initiation of servo and sensor. These are actually mapped in the configuration file.
        servo = hardwareMap.servo.get("servo");
        sensor = hardwareMap.touchSensor.get("sensor");

        //Initiation of load file with the same parameters as are in the record opmode.
        loadFile= new File(Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOWNLOADS) + "/Servo Test Saves", "save2.txt");
        if (!loadFile.exists()) {
            //Double check if the file exists in order to avoid errors later.
            telemetry.addData("Error", "Couldn't find load file");
        }

    }

    //Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
    @Override
    public void init_loop() {


    }

    //Code to run ONCE when the driver hits PLAY
    @Override
    public void start() {

        servo.setPosition(0);

        //Temporary lists to store lines from load file
        Vector load = new Vector();
        String[] e;
        try {
            FileInputStream f = new FileInputStream(loadFile);
            BufferedReader reader = new BufferedReader(new InputStreamReader(f));

            String line = reader.readLine();
            while(line != null){
                load.addElement(line);
                line = reader.readLine();
            }

        } catch (FileNotFoundException er) {

        } catch (IOException er) {

        } finally {
            e = Arrays.copyOf(load.toArray(), load.toArray().length, String[].class);
        }

        /* Initiation of values array. The first dimension's length is 2 less than the length of the
        * save file (because the values don't include START and STOP. Keep in mind for every value in
        * the first dimension, it returns another set of values.
        */
        values = new double[e.length-2][2];

        for (int i = 1; i < e.length-1; i++) {  //This is a for loop that will loop only so many times,
                                                // specifically the length of the save file minus two

            /* Simply what the latter lines do is take the data set that the loop is currently on,
            * slice up the string, and convert the values into doubles. This is necessary because
            * the data is read as text, but this opmode needs it as a number. An example of a set of
            * values would be "{1.084037}, {1.0}". The first part is the time in seconds that the program
            * was at when recording the second part -- the power of the servo. In order to get these
            * back to numbers, it first removes specifically "1.084037" and "1.0" using substring
            * ("e[i].substring(e[i].indexOf('{')+1, e[i].indexOf('}'))"). Then it converts those strings
            * to doubles using parseDouble() and assigns the values to variables.
            */
            double TIME = Double.parseDouble(e[i].substring(e[i].indexOf('{')+1, e[i].indexOf('}')));
            double POWER = Double.parseDouble(e[i].substring(e[i].lastIndexOf('{')+1, e[i].lastIndexOf('}')));

            values[i-1] = new double[]{TIME, POWER}; //This then commits those variables to the values array
        }

        runtime.reset(); //NOTE: this normally appears at the beginning of the function, but it's moved
                         // to the end here because there's a lot of processing that's going on during
                         // the function, so it's moved here to keep the time as accurate as possible.

    }

    //Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {
        telemetry.addData("Status", "Running: " + runtime.toString());

        //Code to read and set values
        if (counter < values.length) { //As long as the value we're looking for exists, do the following
            if (runtime.seconds() > values[counter][0]) { //If the runtime matches, do the following
                servo.setPosition(values[counter][1]); //Set the servo to the value corresponding to the time
                counter++; //Look for the next value
            }

            //Show the driver how many lines it needs to read, the current line it's reading, and the position of the servo
            telemetry.addData("Values Length", values.length);
            telemetry.addData("Counter", counter);
            telemetry.addData("Servo Position", servo.getPosition());
            //NOTE: the opmode runs through all the data so quickly that counter is increasing really fast.
            // Not the most useful information to the driver, but good to know in case something goes wrong.

        } else { //If there is no more to read, let the driver know that its finished
            telemetry.addData("counter>values.length", "Finished.");
        }

    }

    //Code to run ONCE after the driver hits STOP
    @Override
    public void stop() {

        //There will be code here when using this opmode with motors, but right now there doesn't have to be

    }

}
