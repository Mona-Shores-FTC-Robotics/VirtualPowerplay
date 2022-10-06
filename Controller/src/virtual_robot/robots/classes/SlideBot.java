package virtual_robot.robots.classes;

import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.configuration.MotorType;

import javafx.fxml.FXML;
import javafx.geometry.Pos;
import javafx.scene.control.Label;
import javafx.scene.shape.Rectangle;
import javafx.scene.transform.Rotate;
import javafx.scene.transform.Translate;
import virtual_robot.controller.BotConfig;

/**
 * For internal use only. Represents a robot with four mechanum wheels, color sensor, four distance sensors,
 * a BNO055IMU, and a slide that slides and elevates.
 * <p>
 * The easiest way to create a new robot configuration is to copy and paste the Java class and the FXML file
 * of an existing configuration, then make make modifications. The ArmBot config is a modification of
 * the MechanumBot config.
 * <p>
 * The @BotConfig annotation is required. The name will be displayed to the user in the Configuration
 * combo box. The filename refers to the fxml file that contains the markup for the graphical UI.
 * Note: the fxml file must be located in the virtual_robot.robots.classes.fxml folder.
 */
@BotConfig(name = "Slide Bot", filename = "slide_bot")
public class SlideBot extends XDrivePhysicsBase {
    private static double SLIDE_LENGTH = 10;
    private static double SLIDE_WIDTH = 10;
    private static double SLIDE_X = 35;
    private static double SLIDE_Y = 0;

    //Servo to control the hand at the end of the arm. Note use of ServoImpl class rather than Servo interface.
    private ServoImpl elevationServo = null;
    private ServoImpl slideServo = null;

    private Translate slideTranslate = new Translate(SLIDE_X, SLIDE_Y);


    /*
    Variables representing graphical UI nodes that we will need to manipulate. The @FXML annotation will
    cause these variables to be instantiated automatically during loading of the slide_bot.fxml file.
     */
    @FXML
    private Rectangle slide;            //The slide
    @FXML
    private Label elevation;           //Where to put the elevation

    /**
     * No-parameter constructor. This will be used if SlideBot is selected from the Config menu. It will use
     * the default motor type in MechanumBase.
     */
    public SlideBot() {
        super();
    }

    /**
     * Slidebot constructor. This can only be used by subclasses.
     * @param driveMotorType
     */
    public SlideBot(MotorType driveMotorType){
        super(driveMotorType);
    }

    /**
     * The initialize() method is called automatically when the robot's graphical UI is loaded from the
     * arm_bot.fxml markup file. It should be used to set up parts of the graphical UI that will change
     * as the robot operates
     */
    public void initialize() {
        super.initialize();

        //Temporarily activate the hardware map to allow calls to "get"
        hardwareMap.setActive(true);

        //Instantiate the slide servos. Note the cast to ServoImpl.
        elevationServo = (ServoImpl) hardwareMap.servo.get("elevation_servo");
        slideServo = (ServoImpl) hardwareMap.servo.get("slide_servo");

        //Deactivate the hardwaremap to prevent users from accessing hardware until after INIT is pressed
        hardwareMap.setActive(false);

        slide.getTransforms().add(slideTranslate);
    }

    /**
     * Create the HardwareMap object
     */
    protected void createHardwareMap() {
        super.createHardwareMap();

        hardwareMap.put("elevation_servo", new ServoImpl());
        hardwareMap.put("slide_servo", new ServoImpl());
    }

    /**
     * Update robot position on field and update the robot sensors
     *
     * @param millis milliseconds since the previous update
     */
    public synchronized void updateStateAndSensors(double millis) {
        super.updateStateAndSensors(millis);
    }

    private double getSlideAngle() {
            // Using GoBilda servos programmed to 300 degrees of rotation
            double servoAngle = 360 * (slideServo.getInternalPosition() - 0.5);
            // 5:1 gear reduction
            double slideAngle = servoAngle;
            return slideAngle;
    }

    private double getSlideTranslate() {
        double servoSlideTranslateX = 28 * (slideServo.getInternalPosition() - .5);
        return servoSlideTranslateX;
    }


    private double getElevationAngle() {
        // Using GoBilda servos programmed to 300 degrees of rotation
        double servoAngle = 360 * elevationServo.getInternalPosition();
        // 9:1 gear reduction
        double elevationAngle = servoAngle;
        return elevationAngle;

        /* This is the original getElevationAngle and getTurretAngle code.

         private double getTurretAngle() {
            // Using GoBilda servos programmed to 300 degrees of rotation
            double servoAngle = 300 * (turretServo.getInternalPosition() - 0.5);
            // 5:1 gear reduction
            double turretAngle = servoAngle / 5.0;
            return turretAngle;
        }

         private double getElevationAngle() {
            // Using GoBilda servos programmed to 300 degrees of rotation
            double servoAngle = 300 * elevationServo.getInternalPosition();
            // 9:1 gear reduction
            double elevationAngle = 15 + (servoAngle / 9.0);
            return elevationAngle;
         */
    }

    /**
     * Update the display of the robot UI. This method will be called from the UI Thread via a call to
     * Platform.runLater().
     */
    @Override
    public synchronized void updateDisplay() {
        super.updateDisplay();

        // Update the slide part of the display
        slide.setX(getSlideTranslate());

        elevation.setText(String.format("%.1f", getElevationAngle()));
        elevation.setAlignment(Pos.CENTER);
    }

    /**
     * Stop all motors and close the BNO055IMU
     */
    public void powerDownAndReset() {
        super.powerDownAndReset();
    }

}
