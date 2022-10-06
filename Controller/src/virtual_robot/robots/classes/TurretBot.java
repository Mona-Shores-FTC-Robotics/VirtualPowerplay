package virtual_robot.robots.classes;

import com.qualcomm.robotcore.hardware.DcMotorExImpl;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.configuration.MotorType;

import org.dyn4j.collision.CategoryFilter;
import org.dyn4j.dynamics.Body;
import org.dyn4j.dynamics.joint.RevoluteJoint;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import javafx.fxml.FXML;
import javafx.geometry.Pos;
import javafx.scene.Group;
import javafx.scene.control.Label;
import javafx.scene.shape.Circle;
import javafx.scene.shape.Rectangle;
import javafx.scene.transform.Rotate;
import virtual_robot.controller.BotConfig;
import virtual_robot.dyn4j.Dyn4jUtil;
import virtual_robot.dyn4j.FixtureData;
import virtual_robot.game_elements.classes.Carousel;

/**
 * For internal use only. Represents a robot with four mechanum wheels, color sensor, four distance sensors,
 * a BNO055IMU, and a turret that rotates and elevates.
 * <p>
 * The easiest way to create a new robot configuration is to copy and paste the Java class and the FXML file
 * of an existing configuration, then make make modifications. The ArmBot config is a modification of
 * the MechanumBot config.
 * <p>
 * The @BotConfig annotation is required. The name will be displayed to the user in the Configuration
 * combo box. The filename refers to the fxml file that contains the markup for the graphical UI.
 * Note: the fxml file must be located in the virtual_robot.robots.classes.fxml folder.
 */
@BotConfig(name = "Turret Bot", filename = "turret_bot")
public class TurretBot extends MechanumBase {
    private static double TURRET_LENGTH = 35;
    private static double TURRET_WIDTH = 10;
    private static double TURRET_PIVOT_X = 37;
    private static double TURRET_PIVOT_Y = 37;

    private static double CLAW_PIVOT_X = 37;
    private static double CLAW_PIVOT_Y = 37;


    //Servo to control the hand at the end of the arm. Note use of ServoImpl class rather than Servo interface.
    //private ServoImpl elevationServo = null;
    private DcMotorExImpl liftMotor = null;

    private ServoImpl turretServo = null;
    private ServoImpl intake1Servo = null;
    private ServoImpl intake2Servo = null;
    private ServoImpl clawServo = null;

    Rotate turretRotate;
    Rotate clawRotate;
    Rotate intake1GroupRotateTransform;
    Rotate intake2GroupRotateTransform;
    private Body intake1Body;
    private Body intake2Body;

    /*
    Variables representing graphical UI nodes that we will need to manipulate. The @FXML annotation will
    cause these variables to be instantiated automatically during loading of the turret_bot.fxml file.
     */
    @FXML
    private Rectangle turret;            //The turret

    @FXML
    private Rectangle claw;            //The claw

    @FXML
    private Label lift;           //Where to put the elevation

    @FXML private Circle intake1Circle;
    @FXML private Group intake1Group;

    @FXML private Circle intake2Circle;
    @FXML private Group intake2Group;


    private CategoryFilter INTAKE1_FILTER = new CategoryFilter(Carousel.CAROUSEL_SPINNER_CATEGORY, Carousel.CAROUSEL_CATEGORY);
    private CategoryFilter INTAKE2_FILTER = new CategoryFilter(Carousel.CAROUSEL_SPINNER_CATEGORY, Carousel.CAROUSEL_CATEGORY);

    /**
     * No-parameter constructor. This will be used if TurretBot is selected from the Config menu. It will use
     * the default motor type in MechanumBase.
     */
    public TurretBot() {
        super();
    }

    /**
     * TurretBot constructor. This can only be used by subclasses.
     * @param driveMotorType
     */
    public TurretBot(MotorType driveMotorType){
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

        //Instantiate the turret servos. Note the cast to ServoImpl.
        //elevationServo = (ServoImpl) hardwareMap.servo.get("elevation_servo");

        liftMotor = hardwareMap.get(DcMotorExImpl.class, "lift_motor");

        turretServo = (ServoImpl) hardwareMap.servo.get("turret_servo");
        intake1Servo = (ServoImpl) hardwareMap.servo.get("intake1_servo");
        intake2Servo = (ServoImpl) hardwareMap.servo.get("intake2_servo");
        clawServo = (ServoImpl) hardwareMap.servo.get("claw_servo");

        //Deactivate the hardwaremap to prevent users from accessing hardware until after INIT is pressed
        hardwareMap.setActive(false);

        turretRotate = new Rotate(0, TURRET_PIVOT_X, TURRET_PIVOT_Y);
        turret.getTransforms().add(turretRotate);

        intake1GroupRotateTransform = new Rotate(0, 22, 12);
        intake1Group.getTransforms().add(intake1GroupRotateTransform);

        intake2GroupRotateTransform = new Rotate(0, 53, 12);
        intake2Group.getTransforms().add(intake2GroupRotateTransform);

        clawRotate = new Rotate(0, CLAW_PIVOT_X, CLAW_PIVOT_Y);
        claw.getTransforms().add(clawRotate);
    }

    /**
     * Create the HardwareMap object
     */
    protected void createHardwareMap() {
        super.createHardwareMap();


        hardwareMap.put("lift_motor", new DcMotorExImpl(MotorType.Neverest40));

        hardwareMap.put("turret_servo", new ServoImpl());
        hardwareMap.put("intake1_servo", new ServoImpl());
        hardwareMap.put("intake2_servo", new ServoImpl());
        hardwareMap.put("claw_servo", new ServoImpl());
    }

    /**
     * Update robot position on field and update the robot sensors
     *
     * @param millis milliseconds since the previous update
     */
    public synchronized void updateStateAndSensors(double millis) {
        super.updateStateAndSensors(millis);

        liftMotor.update(millis);
        double liftMotorVelocity = liftMotor.getVelocity(AngleUnit.RADIANS);

    }

    private double getIntake1Angle() {
        // Using GoBilda servos programmed to 300 degrees of rotation
        double servoAngle = 180 * (intake1Servo.getInternalPosition());
        // 5:1 gear reduction
        double intake1Angle = servoAngle;
        return intake1Angle;
    }

    private double getIntake2Angle() {
        // Using GoBilda servos programmed to 300 degrees of rotation
        double servoAngle = 180 * (intake2Servo.getInternalPosition());
        // 5:1 gear reduction
        double intake2Angle = servoAngle;
        return intake2Angle;
    }

    private double getTurretAngle() {
            // Using GoBilda servos programmed to 300 degrees of rotation
            double servoAngle = 180 * (turretServo.getInternalPosition() - 0.5);
            // 5:1 gear reduction
            double turretAngle = servoAngle;
            return turretAngle;
    }

    private double getClawAngle() {
        // Using GoBilda servos programmed to 300 degrees of rotation
        double servoAngle = 180 * (clawServo.getInternalPosition() - 0.5);
        // 5:1 gear reduction
        double clawAngle = servoAngle;
        return clawAngle;
    }


     private double getLiftAngle() {
         double liftMotorCurrentPosition = liftMotor.getCurrentPosition();
         return liftMotorCurrentPosition;
     }



    /**
    private double getElevationAngle() {
        // Using GoBilda servos programmed to 300 degrees of rotation
        double servoAngle = 360 * elevationServo.getInternalPosition();
        // 9:1 gear reduction
        double elevationAngle = servoAngle;
        return elevationAngle;
    }
    **/

    /**
     * Update the display of the robot UI. This method will be called from the UI Thread via a call to
     * Platform.runLater().
     */
    @Override
    public synchronized void updateDisplay() {
        super.updateDisplay();

        // Update the turret part of the display
        turretRotate.setAngle(getTurretAngle());
        lift.setText(String.format("%.1f", getLiftAngle()));
        lift.setAlignment(Pos.CENTER);
        intake1GroupRotateTransform.setAngle(getIntake1Angle());
        intake2GroupRotateTransform.setAngle(getIntake2Angle());
        clawRotate.setAngle(getClawAngle()+ getTurretAngle());
    }

    /**
     * Stop all motors and close the BNO055IMU
     */
    public void powerDownAndReset() {
        super.powerDownAndReset();
    }

}

