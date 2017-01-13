package org.firstinspires.ftc.teamcode.barebones_testing;

import java.io.File;
import android.os.Environment;
import android.text.AndroidCharacter;

import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.PrintWriter;
import java.text.SimpleDateFormat;
import java.util.Calendar;

/**
 * Created by Chandler on 12/14/2016.
 */
public class DebugLogger {

    private File log;
    private PrintWriter pw;
    FileOutputStream f;

    public DebugLogger() {

        File directory = new File(Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOWNLOADS) + "/Debug Logs");
        if (!directory.exists()) {
            directory.mkdir();
        }

        log = new File(directory, "log_" +
                (Calendar.getInstance().get(Calendar.MONTH)+1) + "-" +
                (Calendar.getInstance().get(Calendar.DAY_OF_MONTH)) + "_" +
                (Calendar.getInstance().get(Calendar.HOUR_OF_DAY)) + ":" +
                (Calendar.getInstance().get(Calendar.MINUTE)) + ".txt");
        try {
            log.createNewFile();
            f = new FileOutputStream(log);
            pw = new PrintWriter(f);
        } catch (Exception e) {
            //It doesn't really help to log the error here if the log creation is the error.
        }

    }

    public void log(String line) {
        pw.println(line);
    }

    public void close_log() {
        pw.flush();
        pw.close();
        try {
            f.close();
        } catch (Exception e) {

        }
    }

}
