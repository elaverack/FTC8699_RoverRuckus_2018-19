package org.firstinspires.ftc.teamcode.visuals;

import android.graphics.PointF;
import android.util.Log;

import org.opencv.android.OpenCVLoader;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class Graph {

    private static final int margin = 4; // pixels
    private static final double def_title_scale = 1.5, def_axis_scale = 1, def_label_scale = .5;
    private static final Scalar def_background = new Scalar(255, 255, 255), def_foreground = new Scalar(0, 0, 0);
    private static final int def_font = Core.FONT_HERSHEY_SIMPLEX;
    
    private String title, x_title, y_title;
    private int width, height;
    private double x_start, x_stop, x_step, y_start, y_stop, y_step;
    
    Mat  graph;
    Rect workingArea;
    
    public Graph (String title, String xtitle, String ytitle, int width, int height, double xstart, double xstop, double xstep, double ystart, double ystop, double ystep) {
        this.title = title;
        x_title = xtitle;
        y_title = ytitle;
        this.width = width;
        this.height = height;
        x_start = xstart;
        x_stop = xstop;
        x_step = xstep;
        y_start = ystart;
        y_stop = ystop;
        y_step = ystep;
    
    
        OpenCVLoader.initDebug();
        
        graph = new Mat(this.height, this.width, CvType.CV_8UC3);
        graph.setTo(def_background);
        
        workingArea = new Rect(margin, margin, this.width - margin * 2, this.height - margin * 2);
        
        GraphText t = new GraphText(this.title, def_font, def_title_scale, def_foreground);
        t.resizeToFit(workingArea.width, workingArea.height);
        t.putThis(graph, GraphText.Pivot.TOP_CENTER, new Point(this.width / 2, margin));
        
        workingArea.y += t.getBox().height + margin;
        workingArea.height -= t.getBox().height + margin;
        
        GraphText x = new GraphText(x_title, def_font, def_axis_scale, def_foreground);
        x.resizeToFit(workingArea.width, workingArea.height);
        x.putThis(graph, GraphText.Pivot.BOTTOM_CENTER, new Point(this.width / 2, this.height - margin));
        
        workingArea.height -= x.getBox().height + margin;
        
        Scalar ci = new Scalar(def_foreground.val[0] + def_background.val[0], def_foreground.val[1] + def_background.val[1], def_foreground.val[2] + def_background.val[2]);
        GraphText y = new GraphText(y_title, def_font, def_axis_scale, ci);
        y.resizeToFit(workingArea.height, workingArea.width);
        Mat temp = new Mat(this.height, this.width, CvType.CV_8UC3);
        temp.setTo(new Scalar(0,0,0));
        y.putThis(temp, GraphText.Pivot.TOP_LEFT, new Point(margin, workingArea.y + workingArea.height / 2 + y.getBox().width / 2));
        Mat trans = Imgproc.getRotationMatrix2D(new Point(margin, workingArea.y + workingArea.height / 2 + y.getBox().width / 2), 90, 1);
        Imgproc.warpAffine(temp, temp, trans, temp.size());
        //Log.wtf("DEBUG", "Graph: " + graph.cols() + ", " + graph.rows());
        //Log.wtf("DEBUG", "Temp: " + temp.cols() + ", " + temp.rows());
        Core.subtract(graph, temp, graph);
        
        workingArea.x += y.getBox().height + margin;
        workingArea.width -= y.getBox().height + margin;
        
        ArrayList<GraphText> xLabels = new ArrayList<>(), yLabels = new ArrayList<>();
        
        for (double i = x_start; i <= x_stop; i += Math.abs(x_step)) xLabels.add(new GraphText(Double.toString(i), def_font, def_label_scale, def_foreground));
        for (double i = y_stop; i >= y_start; i -= Math.abs(y_step)) yLabels.add(new GraphText(Double.toString(i), def_font, def_label_scale, def_foreground));
        
        int ylabelmar = 0; for (GraphText ylabel : yLabels) if (ylabel.getBox().width > ylabelmar) ylabelmar = (int)ylabel.getBox().width; // get widest y label
        
        workingArea.width -= ylabelmar + (int) ( 1.5 * margin );
        double dx = (workingArea.width - margin - .5*xLabels.get(0).getBox().width) / ( xLabels.size() - 1 );
        
        int aj = 0, ak = 0; // widest x label's width and widest label's width next to j
        for (int i = 0; i < xLabels.size(); i++) {
            int w;
            if ((w = (int)xLabels.get(i).getBox().width) > aj) {
                aj = w;
                if (i - 1 < 0) ak = (int)xLabels.get(i + 1).getBox().width;
                else if (i + 1 > xLabels.size() - 1) ak = (int)xLabels.get(i - 1).getBox().width;
                else {
                    int w0 = (int)xLabels.get(i - 1).getBox().width, w1 = (int)xLabels.get(i + 1).getBox().width;
                    ak = w0 > w1 ? w0 : w1;
                }
            }
        }
        if (( dx - .5 * ( aj + ak ) ) < margin) {
            double s = (double)( 2 * ( workingArea.width - 2 * margin - xLabels.size() * margin ) ) / ( ( xLabels.size() - 1 ) * ( aj + ak ) + xLabels.get(xLabels.size()-1).getBox().width );
            for (GraphText text : xLabels) text.scale *= s;
            dx = ( workingArea.width - margin - .5 * xLabels.get(0).getBox().width ) / ( xLabels.size() - 1 );
        }
        
        for (int i = 0; i < xLabels.size(); i++) {
            Point p = new Point(workingArea.x + ylabelmar + (int) ( 1.5 * margin ) + (int)(dx * i), workingArea.br().y);
            xLabels.get(i).putThis(graph, GraphText.Pivot.BOTTOM_CENTER, p);
            if (i == 0) Imgproc.line(
                    graph,
                    new Point(p.x, p.y - xLabels.get(i).getBox().height - (int)(.5*margin)),
                    new Point(p.x, workingArea.y),
                    def_foreground);
            else Imgproc.line(
                    graph,
                    new Point(p.x, p.y - xLabels.get(i).getBox().height - (int) ( .5 * margin )),
                    new Point(p.x, p.y - xLabels.get(i).getBox().height - (int) ( 2.5 * margin )),
                    def_foreground);
        }
        
        workingArea.height -= xLabels.get(0).getBox().height + (int) ( 1.5 * margin );
        
        double dy = ( workingArea.height - .5*yLabels.get(0).getBox().height ) / (yLabels.size()-1);
        
        for (int i = 0; i < yLabels.size(); i++) {
            Point p = new Point(workingArea.x + ylabelmar, workingArea.y + (int)(dy * i));
            yLabels.get(i).putThis(graph, GraphText.Pivot.TOP_RIGHT, p);
            if (i == yLabels.size() - 1) Imgproc.line(
                    graph,
                    new Point(p.x + (int) ( .5 * margin ), p.y + (int) ( .5 * yLabels.get(i).getBox().height )),
                    new Point(this.width - margin, p.y + (int) ( .5 * yLabels.get(i).getBox().height )),
                    def_foreground);
            else Imgproc.line(
                    graph,
                    new Point(p.x + (int) ( .5 * margin ), p.y + (int) ( .5 * yLabels.get(i).getBox().height )),
                    new Point(p.x + (int) ( 2.5 * margin ), p.y + (int) ( .5 * yLabels.get(i).getBox().height )),
                    def_foreground);
        }
        
        workingArea.x += ylabelmar + (int) ( 1.5 * margin );
        workingArea.y += (int) ( .5 * yLabels.get(0).getBox().height );
        workingArea.width -= workingArea.br().x - (workingArea.x + (int) ( dx * (xLabels.size() - 1) ));
        workingArea.height -= (int)(.5*yLabels.get(0).getBox().height);
        
        
        //Rectangle(graph, workingArea, new MCvScalar(0, 0, 255));
    }
    
    public Mat getGraph () { return graph.clone(); }
    public void saveGraph (String filename) { VisualsHandler.savePhoto(getGraph(), filename); }
    
    public void plot (PointF point, int dotRadius, Scalar color) {
        if (point.x > x_stop || point.y > y_stop) return;
        Imgproc.circle(
                graph,
                new Point(
                        (int) ( workingArea.x + ( point.x - x_start ) / ( ( x_stop - x_start ) / ( workingArea.br().x - workingArea.x ) ) ),
                        (int) ( workingArea.br().y - ( point.y - y_start ) / ( ( y_stop - y_start ) / ( workingArea.br().y - workingArea.y ) ) )),
                dotRadius,
                color,
                -1);
    }
    public void plot (List<PointF> points, int dotRadius, Scalar color) { for (PointF p : points) plot(p, dotRadius, color); }
    
    public void referenceLine (PointF point1, PointF point2, int thickness, Scalar color) {
        if (point1.x < x_start || point1.x < y_start || point2.x > x_stop || point2.x > y_stop) return;
        
        Point 
                p1 = new Point(
                        (int) ( workingArea.x + ( point1.x - x_start ) / ( ( x_stop - x_start ) / ( workingArea.br().x - workingArea.x ) ) ),
                        (int) ( workingArea.br().y - ( point1.y - y_start ) / ( ( y_stop - y_start ) / ( workingArea.br().y - workingArea.y ) ) )), 
                p2 = new Point(
                        (int) ( workingArea.x + ( point2.x - x_start ) / ( ( x_stop - x_start ) / ( workingArea.br().x - workingArea.x ) ) ),
                        (int) ( workingArea.br().y - ( point2.y - y_start ) / ( ( y_stop - y_start ) / ( workingArea.br().y - workingArea.y ) ) ));
        
        Imgproc.line(graph, p1, p2, color, thickness);
    }
    
    static class GraphText {
        
        public enum Pivot{
            
            TOP_LEFT(       new PointF(0,   1)),
            TOP_CENTER(     new PointF(-.5f,1)),
            TOP_RIGHT(      new PointF(-1,  1)),
            CENTER_LEFT(    new PointF(0,   .5f)),
            CENTER(         new PointF(-.5f,.5f)),
            CENTER_RIGHT(   new PointF(-1,  .5f)),
            BOTTOM_LEFT(    new PointF(0,   0)),
            BOTTOM_CENTER(  new PointF(-.5f,0)),
            BOTTOM_RIGHT(   new PointF(-1,  0));
            
            private final PointF pivotPointRatio;
            
            Pivot (PointF ppr) { pivotPointRatio = ppr; }
            
            public PointF getRatio () { return pivotPointRatio; }
            
        }
        
        public String text;
        //public Point origin;
        public int font;
        public double scale;
        public Scalar color;
        
        public GraphText (String text, /*Point origin,*/ int font, double scale, Scalar color) {
            this.text = text; /*this.origin = origin;*/ this.font = font; this.scale = scale; this.color = color;
        }
        
        public void putThis (Mat src, Point origin) { Imgproc.putText(src, text, origin, font, scale, color); }
        public void putThis (Mat src, Pivot p, Point o) {
            PointF s     = p.getRatio();
            Size   b     = getBox();
            Point  point = new Point(o.x + (int)(s.x*b.width), o.y + (int)(s.y*b.height));
            Imgproc.putText(src, text, point, font, scale, color);
        }
        
        public Size getBox () { return Imgproc.getTextSize(text, font, scale, 1, new int[1]); }
        public boolean fits (int width, int height) {
            Size s = getBox();
            return s.width <= width && s.height <= height;
        }
        public void resizeToFit (int width, int height) {
            Size b = getBox();
            if (b.width > width) scale *= (double) width / b.width;
            b = getBox();
            if (b.height > height) scale *= (double) height / b.height;
        }
        
    }

}
