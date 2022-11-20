package org.firstinspires.ftc.teamcode.ObjectClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.openftc.easyopencv.OpenCvCamera;

public class AprilTagVision {

    public OpenCvCamera camera;

    public enum Signal {LEFT, MIDDLE, RIGHT}

    //Set default to MIDDLE in case something goes wrong with vision
    public Signal currentSignal = Signal.MIDDLE;
    public int currentSignalNumber = 2;

    // NOTE: this calibration is for the C920 webcam at 800x448.
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int ID_TAG_FOR_LEFT = 0;
    int ID_TAG_FOR_MIDDLE = 1;
    int ID_TAG_FOR_RIGHT = 2;

    int cameraMonitorViewId;

    public void init(HardwareMap ahwMap) {
        currentSignal = Signal.MIDDLE;
    }

    public void CheckForAprilTags(LinearOpMode activeOpMode) {
        //deleted because vision doesn't work in virtual
    }
}
