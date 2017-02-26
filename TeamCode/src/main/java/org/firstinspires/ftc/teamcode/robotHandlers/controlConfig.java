package org.firstinspires.ftc.teamcode.robotHandlers;

/**
 * Created by Chandler on 2/19/2017.
 */

public class controlConfig {

    public static final int TANK_DEFAULT = 0, MECH_DEFAULT = 1, MECH_REVERSED = 2, RIGHT_POWER = 0, LEFT_POWER = 1, POSITION = 2, ROTATION = 3;

    private final int[][] config = {{LEFT_POWER, RIGHT_POWER}, {POSITION, ROTATION}, {ROTATION, POSITION}};

    public final int LEFT, RIGHT;

    public static controlConfig TANK = new controlConfig(TANK_DEFAULT);
    public static controlConfig MECHANUM = new controlConfig(MECH_DEFAULT);

    public controlConfig (int config) {
        LEFT = this.config[config][0];
        RIGHT = this.config[config][1];
    }

    public controlConfig (int l, int r) {
        LEFT = l;
        RIGHT = r;
    }

}
