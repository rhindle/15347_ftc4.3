package Boneyard;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class is used to define all the specific hardware for the robot.
 */

public class HardwareMSI {
    /* Public OpMode members. */

    /* Motors */
    public DcMotor leftMotor;
    public DcMotor rightMotor;
    public DcMotor lifter;
    public DcMotor shoulderServo;

    /* Servos */
    public Servo elbowServo;
    public Servo clawServo;
    public Servo wristServo;
    public CRServo clawServo2;
    //public CRServo shoulderServo;

    /* Sensors */
    public ColorSensor sensorColor;
    public DistanceSensor sensorColorRange;
    public BNO055IMU sensorIMU;
    //public DigitalChannel digitalTouch;


//    public static final double MID_SERVO = 0.5;
//    public static final double ARM_UP_POWER = 0.45;
//    public static final double ARM_DOWN_POWER = -0.45;
    public static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // 1120 for Neverest 40
    public static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    public static final double     WHEEL_DIAMETER_INCHES   = 4 ;       // For figuring circumference
    public static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public HardwareMSI() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        /* Define and Initialize Motors */
        leftMotor = hwMap.get(DcMotor.class, "leftmotor");
        rightMotor = hwMap.get(DcMotor.class, "rightmotor");
        lifter = hwMap.get(DcMotor.class, "lifter");
        shoulderServo = hwMap.get(DcMotor.class, "sholderServo");

        leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        lifter.setDirection(DcMotor.Direction.FORWARD);
        shoulderServo.setDirection(DcMotor.Direction.FORWARD);

        leftMotor.setPower(0);
        rightMotor.setPower(0);
        lifter.setPower(0);
        shoulderServo.setPower(0);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoulderServo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /* Define Servos */
        wristServo = hwMap.get(Servo.class, "wristServo");
        elbowServo = hwMap.get(Servo.class, "elbowservo");
        clawServo = hwMap.get(Servo.class, "clawservo");
        clawServo2 = hwMap.get(CRServo.class, "clawServo2");
        //shoulderServo = hwMap.get(CRServo.class, "shoulderservo");

        /* Define Sensors */
        sensorColor = hwMap.get(ColorSensor.class, "sensor_color_distance");
        sensorColorRange = hwMap.get(DistanceSensor.class, "sensor_color_distance");
        //digitalTouch = hwMap.get(DigitalChannel.class, "digitalTouch");
        //digitalTouch.setMode(DigitalChannel.Mode.INPUT);

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "sensorIMU".
        sensorIMU = hwMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        sensorIMU.initialize(parameters);

    }
}


