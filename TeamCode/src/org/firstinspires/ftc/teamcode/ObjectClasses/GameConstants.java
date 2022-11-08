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

    public static final double CONE_HEIGHT_ENC_VAL = 150;
    public static final double CONE_CLEARANCE_HEIGHT_ENC_VAL = CONE_HEIGHT_ENC_VAL /2;

    public static final double CONE_GRIP_HEIGHT_ENC_VAL = 50;



    public static final double HIGH_CONE_JUNCTION_SCORE_HEIGHT_ENC_VAL = 1480;
    public static final double MEDIUM_CONE_JUNCTION_SCORE_HEIGHT_ENC_VAL = 1100 + CONE_HEIGHT_ENC_VAL + CONE_CLEARANCE_HEIGHT_ENC_VAL;
    public static final double LOW_CONE_JUNCTION_SCORE_HEIGHT_ENC_VAL = 800 + CONE_HEIGHT_ENC_VAL + CONE_CLEARANCE_HEIGHT_ENC_VAL;
    public static final double GROUND_CONE_JUNCTION_SCORE_HEIGHT_ENC_VAL = 300 + CONE_HEIGHT_ENC_VAL + CONE_CLEARANCE_HEIGHT_ENC_VAL;
    public static final double TERMINAL_SCORE_HEIGHT_ENC_VAL = 0 + CONE_HEIGHT_ENC_VAL + CONE_CLEARANCE_HEIGHT_ENC_VAL;

    public static final double ONE_CONE_INTAKE_HEIGHT_ENC_VAL = 0;
    public static final double TWO_CONE_STACK_INTAKE_HEIGHT_ENC_VAL = ONE_CONE_INTAKE_HEIGHT_ENC_VAL + CONE_GRIP_HEIGHT_ENC_VAL;
    public static final double THREE_CONE_STACK_INTAKE_HEIGHT_ENC_VAL = TWO_CONE_STACK_INTAKE_HEIGHT_ENC_VAL + CONE_GRIP_HEIGHT_ENC_VAL;
    public static final double FOUR_CONE_STACK_INTAKE_HEIGHT_ENC_VAL = THREE_CONE_STACK_INTAKE_HEIGHT_ENC_VAL + CONE_GRIP_HEIGHT_ENC_VAL;
    public static final double FIVE_CONE_STACK_INTAKE_HEIGHT_ENC_VAL = FOUR_CONE_STACK_INTAKE_HEIGHT_ENC_VAL + CONE_GRIP_HEIGHT_ENC_VAL;

    public enum Signal {LEFT, MIDDLE, RIGHT}
    public static Signal currentSignal = Signal.MIDDLE;

}
