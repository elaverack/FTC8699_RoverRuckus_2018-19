package org.firstinspires.ftc.teamcode.visuals;

// Created on 12/1/2017 at 12:16 PM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode.visuals

import android.app.Activity;
import android.view.View;
import android.widget.LinearLayout;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.ArrayList;

public class LayoutInterfacer {

    private OpMode opmode;
    private ArrayList<Integer> ids = new ArrayList<>();
    private final int parent;

    public LayoutInterfacer(OpMode om, LinearLayout parent) {opmode = om; this.parent = parent.getId();}
    public LayoutInterfacer(OpMode om, LinearLayout parent, View... views) {opmode = om; this.parent = parent.getId(); createViews(views);}

    public void createViews(final View... views) {
        run(new Runnable() {
            @Override
            public void run() {
                LinearLayout p = getParent();
                for (View v:views) {
                    int id = v.getId();
                    View view = getView(id);
                    if (view != null) p.removeView(view);
                    p.addView(v);
                    if (ids.contains(id)) continue;
                    ids.add(id);
                }
            }
        });
    }

    public View getView (int id) { return getActivity().findViewById(id); }
    public LinearLayout getParent() { return (LinearLayout)getView(parent); }

    public void removeViews (final int... ids) {
        run(new Runnable() {
            @Override
            public void run() { LinearLayout p = getParent(); for (int id:ids) { p.removeView(getActivity().findViewById(id)); } }
        });
    }

    public void close() {
        int[] ids = new int[this.ids.size()];
        int c = 0;
        for (Integer i:this.ids) { ids[c] = i; c++; }
        removeViews(ids);
    }

    public void run(Runnable runnable) { getActivity().runOnUiThread(runnable); }

    private Activity getActivity() { return (Activity)opmode.hardwareMap.appContext; }

}
