package org.firstinspires.ftc.teamcode.ObjectClasses;

public final class GameConstants {

    private GameConstants() {
        // No need to instantiate the class, we can hide its constructor
    }

    public static final double FULL_TILE_DISTANCE_DRIVE= 23.5*2; //multiplied by 2 for virtual
    public static final double HALF_TILE_DISTANCE_DRIVE = FULL_TILE_DISTANCE_DRIVE /2;
    public static final double QUARTER_TILE_DISTANCE_DRIVE = HALF_TILE_DISTANCE_DRIVE /2;
    public static final double EIGHTH_TILE_DISTANCE_DRIVE = QUARTER_TILE_DISTANCE_DRIVE /2;
    public static final double SIXTEENTH_TILE_DISTANCE_DRIVE = EIGHTH_TILE_DISTANCE_DRIVE /2;
    public static final double THIRTYSECOND_TILE_DISTANCE_DRIVE = SIXTEENTH_TILE_DISTANCE_DRIVE /2;
    public static final double SIXTYFOURTH_TILE_DISTANCE_DRIVE = THIRTYSECOND_TILE_DISTANCE_DRIVE /2;

    public static final double FULL_TILE_DISTANCE_STRAFE = 27; //multiplied by 2 for virtual
    public static final double HALF_TILE_DISTANCE_STRAFE = FULL_TILE_DISTANCE_STRAFE /2;
    public static final double QUARTER_TILE_DISTANCE_STRAFE = HALF_TILE_DISTANCE_STRAFE /2;
    public static final double EIGHTH_TILE_DISTANCE_STRAFE = QUARTER_TILE_DISTANCE_STRAFE /2;
    public static final double SIXTEENTH_TILE_DISTANCE_STRAFE = EIGHTH_TILE_DISTANCE_STRAFE  /2;
    public static final double THIRTYSECOND_TILE_DISTANCE_STRAFE  = SIXTEENTH_TILE_DISTANCE_STRAFE  /2;
    public static final double SIXTYFOURTH_TILE_DISTANCE_STRAFE  = THIRTYSECOND_TILE_DISTANCE_STRAFE  /2;

    public static final double CONE_HEIGHT_ENC_VAL = 500;
    public static final double HIGH_CONE_JUNCTION_SCORE_HEIGHT_ENC_VAL = 2150;
    public static final double MEDIUM_CONE_JUNCTION_SCORE_HEIGHT_ENC_VAL = 1550;
    public static final double LOW_CONE_JUNCTION_SCORE_HEIGHT_ENC_VAL = 980;
    //public static final double GROUND_CONE_JUNCTION_SCORE_HEIGHT_ENC_VAL = 250;

    public static final double ONE_CONE_INTAKE_HEIGHT_ENC_VAL = 0;
    public static final double TWO_CONE_STACK_INTAKE_HEIGHT_ENC_VAL = 110;
    public static final double THREE_CONE_STACK_INTAKE_HEIGHT_ENC_VAL = 170;
    public static final double FOUR_CONE_STACK_INTAKE_HEIGHT_ENC_VAL = 230;
    public static final double FIVE_CONE_STACK_INTAKE_HEIGHT_ENC_VAL = 320;
}
