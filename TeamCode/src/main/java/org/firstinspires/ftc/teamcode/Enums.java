package org.firstinspires.ftc.teamcode;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class Enums {
    
    /** SAMPLE ARRANGEMENT ENUM FOR SAMPLING */
    public enum SampleArrangement {
        
        LEFT(0, "left"), MIDDLE(1, "middle"), RIGHT(2, "right"), ERROR(-1, "error");
        
        public static Scalar
                silverLow   = new Scalar(200,200,200),
                silverHigh  = new Scalar(255,255,255),
                goldLow     = new Scalar(0,120,200),
                goldHigh    = new Scalar(150,255,255);
        
        private final int toint;
        private final String tostring;
        
        SampleArrangement(int i, String s) { toint = i; tostring = s; }
        
        public String toString () { return tostring; }
        public int toInt () { return toint; }
        
        public static SampleArrangement fromInts (int gold, int silver1, int silver2) {
            if (gold < silver1 && gold < silver2) return LEFT;
            else if ((gold > silver1 && gold < silver2) || (gold < silver1 && gold > silver2)) return MIDDLE;
            else if (gold > silver1 && gold > silver2) return RIGHT;
            else return ERROR;
        }
        
        public static List<MatOfPoint> circularContours (Mat src, double min, double max) {
            List<MatOfPoint> ret = new ArrayList<>();
            
            Imgproc.findContours(src, ret, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            
            for (MatOfPoint m : ret.toArray(new MatOfPoint[0])) {
                
                Point   center = new Point();
                float[] radius = new float[1];
                
                Imgproc.minEnclosingCircle(new MatOfPoint2f(m.toArray()), center, radius);
                
                //System.out.println(center.toString() + "\n" + radius[0]);
                
                double ratio = Imgproc.contourArea(m) / (Math.PI * Math.pow(radius[0], 2));
                
                if (ratio < min || ratio > max) ret.remove(m);
                
            }
            
            return ret;
            
        }
        public static List<MatOfPoint> rectangularContours (Mat src, double min, double max) {
            List<MatOfPoint> ret = new ArrayList<>();
            
            Imgproc.findContours(src, ret, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            
            for (MatOfPoint m : ret.toArray(new MatOfPoint[0])) {
                
                double ratio = Imgproc.contourArea(m) / Imgproc.boundingRect(m).area();
                
                if (ratio < min || ratio > max) ret.remove(m);
                
            }
            
            return ret;
            
        }
    }
    
    /** FIELD POSITION ENUM FOR AUTONOMOUS MOVEMENT */
    public enum FieldPosition {
        
        CRATER, DEPOT, ERROR
        
    }
    
}
