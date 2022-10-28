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

    public static final double CONE_HEIGHT_MM = 152;
    public static final double CONE_INTAKE_HEIGHT_CHANGE_MM = 60;

    public static final double HIGH_CONE_JUNCTION_SCORE_HEIGHT_MM = 851 + CONE_HEIGHT_MM + CONE_INTAKE_HEIGHT_CHANGE_MM;
    public static final double MEDIUM_CONE_JUNCTION_SCORE_HEIGHT_MM = 597 + CONE_HEIGHT_MM + CONE_INTAKE_HEIGHT_CHANGE_MM;
    public static final double LOW_CONE_JUNCTION_SCORE_HEIGHT_MM = 343 + CONE_HEIGHT_MM + CONE_INTAKE_HEIGHT_CHANGE_MM;
    public static final double GROUND_CONE_JUNCTION_SCORE_HEIGHT_MM = 127 + CONE_HEIGHT_MM + CONE_INTAKE_HEIGHT_CHANGE_MM;
    public static final double TERMINAL_SCORE_HEIGHT_MM = 0 + CONE_HEIGHT_MM + CONE_INTAKE_HEIGHT_CHANGE_MM;

    public static final double ONE_CONE_INTAKE_HEIGHT_MM = CONE_HEIGHT_MM + CONE_INTAKE_HEIGHT_CHANGE_MM;
    public static final double TWO_CONE_STACK_INTAKE_HEIGHT_MM = ONE_CONE_INTAKE_HEIGHT_MM + CONE_INTAKE_HEIGHT_CHANGE_MM;
    public static final double THREE_CONE_STACK_INTAKE_HEIGHT_MM = TWO_CONE_STACK_INTAKE_HEIGHT_MM + CONE_INTAKE_HEIGHT_CHANGE_MM;
    public static final double FOUR_CONE_STACK_INTAKE_HEIGHT_MM = THREE_CONE_STACK_INTAKE_HEIGHT_MM + CONE_INTAKE_HEIGHT_CHANGE_MM;
    public static final double FIVE_CONE_STACK_INTAKE_HEIGHT_MM = FOUR_CONE_STACK_INTAKE_HEIGHT_MM  + CONE_INTAKE_HEIGHT_CHANGE_MM;

    public static final int W_3_JUNCTION = 1;
    public static final int Y_3_JUNCTION = -1;
    public static final int X_2_JUNCTION = 2;
    public static final int X_4_JUNCTION = -2;

    public static enum Signal {LEFT, MIDDLE, RIGHT}
    public static Signal currentSignal = Signal.MIDDLE;

}
