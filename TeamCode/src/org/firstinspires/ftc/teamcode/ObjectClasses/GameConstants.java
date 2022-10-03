package org.firstinspires.ftc.teamcode.ObjectClasses;

public final class GameConstants {

    private GameConstants() {
        // No need to instantiate the class, we can hide its constructor
    }

    public static final int FULL_TILE_DISTANCE = 50;
    public static final int HALF_TILE_DISTANCE = FULL_TILE_DISTANCE /2;
    public static final int QUARTER_TILE_DISTANCE = HALF_TILE_DISTANCE /2;
    public static final int EIGHTH_TILE_DISTANCE = QUARTER_TILE_DISTANCE /2;
    public static final int SIXTEENTH_TILE_DISTANCE = EIGHTH_TILE_DISTANCE /2;

    public static final double HIGH_CONE_JUNCTION_SCORE_HEIGHT = 1.0;
    public static final double MEDIUM_CONE_JUNCTION_SCORE_HEIGHT = .9;
    public static final double LOW_CONE_JUNCTION_SCORE_HEIGHT = .8;
    public static final double GROUND_CONE_JUNCTION_SCORE_HEIGHT = .2;
    public static final double TERMINAL_HEIGHT = .1;

    public static final double CONE_INTAKE_HEIGHT_CHANGE = .2;

    public static final double FIVE_CONE_STACK_INTAKE_HEIGHT = .7;
    public static final double FOUR_CONE_STACK_INTAKE_HEIGHT = .6;
    public static final double THREE_CONE_STACK_INTAKE_HEIGHT = .5;
    public static final double TWO_CONE_STACK_INTAKE_HEIGHT = .4;
    public static final double ONE_CONE_INTAKE_HEIGHT = .3;

}
