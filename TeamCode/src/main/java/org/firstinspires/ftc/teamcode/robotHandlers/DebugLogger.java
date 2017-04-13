package org.firstinspires.ftc.teamcode.robotHandlers;

import java.io.File;
import android.os.Environment;

import java.io.FileOutputStream;
import java.io.PrintWriter;
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

        Calendar calendar = Calendar.getInstance();
        log = new File(directory, "log_" +
                (calendar.get(Calendar.MONTH)+1) + "-" +
                (calendar.get(Calendar.DAY_OF_MONTH)) + "_" +
                (calendar.get(Calendar.HOUR_OF_DAY)) + ":" +
                (calendar.get(Calendar.MINUTE)) + ".txt");
        try {
            log.createNewFile();
            f = new FileOutputStream(log);
            pw = new PrintWriter(f);
        } catch (Exception e) {
            //It doesn't really help to log the error here if the log creation is the error.
        }

    }

    public void log(String line) {
        Calendar calendar = Calendar.getInstance();
        pw.println((calendar.get(Calendar.HOUR_OF_DAY)) + "." + (calendar.get(Calendar.MINUTE)) + "." + (calendar.get(Calendar.SECOND)) + ":\t" + line);
        pw.flush();
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
