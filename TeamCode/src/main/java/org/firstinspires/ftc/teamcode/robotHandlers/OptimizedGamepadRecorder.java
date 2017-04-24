package org.firstinspires.ftc.teamcode.robotHandlers;

import android.os.Environment;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.Arrays;
import java.util.Enumeration;
import java.util.Vector;

/**
 * Created by Chandler on 4/14/2017.
 */

public class OptimizedGamepadRecorder {

    public static final String RECORD_DIR = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOWNLOADS) + "/Recorded Movements";

    private final File FILE;

    private float[] currentValues = new float[4];
    private float[] previousValues = new float[]{0, 0, 0, 0};
    private Vector save = new Vector();
    private ElapsedTime runtime = new ElapsedTime();

    public OptimizedGamepadRecorder (String fileName) throws Error {

        File dir = new File(RECORD_DIR);
        if (!dir.exists()) dir.mkdirs();

        FILE = new File(RECORD_DIR, fileName);

        if (FILE.exists() && !FILE.delete()) throw new Error("Welp, file exists and I couldn't delete it...");

        try {
            if (!FILE.createNewFile()) throw new Error("Welp, couldn't make file.");
        } catch (IOException e) { throw new Error("Welp, couldn't make file -- IOException."); }

        save.addElement("START");
        runtime.reset();

    }

    public void resetRuntime () { runtime.reset(); }

    public void doRecord (float straight, float rotate, float strafe, float slow) {

        currentValues = new float[]{straight, rotate, strafe, slow};

        if (!compareValues(previousValues, currentValues)) {
            save.addElement("{" + runtime.seconds() + "}, {" + straight + "}, {" + rotate + "}, {" + strafe + "}, {" + slow + "}");
        }

        previousValues = currentValues;

    }

    public void closeFile () {

        save.addElement("STOP");

        try {
            FileOutputStream f = new FileOutputStream(FILE);
            PrintWriter pw = new PrintWriter(f);
            Enumeration e = save.elements();

            while (e.hasMoreElements()) {
                pw.println(e.nextElement().toString());
            }
            pw.flush();
            pw.close();
            f.close();
        } catch (FileNotFoundException e) {
            throw new Error("Welp, couldn't make file -- FileNotFoundException.");
        } catch (IOException e) {
            throw new Error("Welp, couldn't make file -- IOException.");
        }

    }

    private boolean compareValues(float[] prior, float[] present) { return Arrays.equals(prior, present); }

}
