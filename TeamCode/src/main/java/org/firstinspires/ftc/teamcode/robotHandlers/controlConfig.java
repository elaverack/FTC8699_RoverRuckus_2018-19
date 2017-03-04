package org.firstinspires.ftc.teamcode.robotHandlers;

/**
 * Created by Chandler on 2/19/2017.
 */

public class ControlConfig {

    public static final int TANK_DEFAULT = 0, MECH_DEFAULT = 1, MECH_REVERSED = 2, RIGHT_POWER = 0, LEFT_POWER = 1, POSITION = 2, ROTATION = 3;

    private final int[][] config = {{LEFT_POWER, RIGHT_POWER}, {POSITION, ROTATION}, {ROTATION, POSITION}};

    public final int LEFT, RIGHT;

    public static ControlConfig TANK = new ControlConfig(TANK_DEFAULT);
    public static ControlConfig MECHANUM = new ControlConfig(MECH_DEFAULT);

    public ControlConfig(int config) {
        LEFT = this.config[config][0];
        RIGHT = this.config[config][1];
    }

    public ControlConfig(int l, int r) {
        LEFT = l;
        RIGHT = r;
    }

}
