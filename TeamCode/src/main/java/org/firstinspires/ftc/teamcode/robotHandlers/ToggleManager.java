package org.firstinspires.ftc.teamcode.robotHandlers;

/**
 * Created by Chandler on 3/7/2017.
 */

public abstract class ToggleManager {

    private boolean toggling = false;

    public abstract void onToggle();

    public void doToggle (boolean toggle) {
        if (toggling) return;
        if (!toggle) {toggling = false; return;}
        onToggle();
    }

}
