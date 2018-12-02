package Boneyard;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Hardware_LeighBot;

import java.util.Locale;


@TeleOp (name="Leighbot: Testing (A)", group="Test")
//@Disabled
public class LeighBot_Tester2 extends LinearOpMode {
//    private Gyroscope sensorIMU;
//    private DcMotor motorTest;
//    private DigitalChannel digitalTouch;
//    private DistanceSensor sensorColorRange;
//    private Servo servoTest;

    Hardware_LeighBot robot   = new Hardware_LeighBot();
    Orientation angles;
    float powerLeft, powerRight, xValue, yValue;

    @Override
    public void runOpMode() {
//        sensorIMU = hardwareMap.get(Gyroscope.class, "sensorIMU");
//        motorTest = hardwareMap.get(DcMotor.class, "motorTest");
//        digitalTouch = hardwareMap.get(DigitalChannel.class, "digitalTouch");
//        sensorColorRange = hardwareMap.get(DistanceSensor.class, "sensorColorRange");
//        servoTest = hardwareMap.get(Servo.class, "servoTest");
//
//        // set digital channel to input mode
//        digitalTouch.setMode(DigitalChannel.Mode.INPUT);

        robot.init(hardwareMap);

        // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
        robot.motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.motorBoom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorStick.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Send telemetry message to alert driver that we are calibrating;
        telemetry.addData(">", "Calibrating Gyro");    //
        telemetry.update();

        // make sure the gyro is calibrated before continuing
        while (!isStopRequested() && !robot.sensorIMU.isGyroCalibrated())  {
            sleep(50);
            idle();
        }

        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();

        robot.motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBoom.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.motorStick.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.motorStick.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // LK: The following help with inaccuracy in my test robot; not present in original gyro code
//        robot.motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        robot.motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.motorBoom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.motorStick.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (Display Gyro value), and reset gyro before we move..
        while (!isStarted()) {
            angles = robot.sensorIMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData(">", "Robot Heading = %.1f", angles.firstAngle);
            telemetry.update();
            sleep(100);
        }

//        // Wait for the game to start (driver presses PLAY)
//        waitForStart();

        double tgtPowerLeft = 0;
        double tgtPowerRight = 0;
        double tgtPowerBoom = 0;
        double tgtPowerStick = 0;

        double tgtSampler = 0.5;

        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F, 0F, 0F};
        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;
        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            /* Color Sensor Section */

            Color.RGBToHSV((int) (robot.sensorColor.red() * SCALE_FACTOR),
                    (int) (robot.sensorColor.green() * SCALE_FACTOR),
                    (int) (robot.sensorColor.blue() * SCALE_FACTOR),
                    hsvValues);

            telemetry.addData("Distance (cm)",
                    String.format(Locale.US, "%.02f", robot.sensorDistance.getDistance(DistanceUnit.CM)));
//            telemetry.addData("Alpha", robot.sensorColor.alpha());
//            telemetry.addData("Red  ", robot.sensorColor.red());
//            telemetry.addData("Green", robot.sensorColor.green());
//            telemetry.addData("Blue ", robot.sensorColor.blue());
            telemetry.addData("Hue", hsvValues[0]);

            if (robot.sensorDistance.getDistance(DistanceUnit.CM) < 10 && hsvValues[0] < 50) {
                telemetry.addData(">", "Candidate Found!");
            } else {
                telemetry.addData(">", "no match");
            }

            /* Drive Section */

            yValue = -gamepad1.left_stick_y;   // throttle
            xValue = -gamepad1.right_stick_x;  // steer

            if (!gamepad1.right_bumper) {
                if (yValue < 0) {              // mimic car steering wrt reversing
                    xValue = -xValue;
                }
                xValue = xValue * Math.abs(yValue);  // make turn proportional to speed
            }

            powerLeft =  yValue - xValue;
            powerRight = yValue + xValue;

            powerLeft = Range.clip(powerLeft,-1.0f,1.0f);
            powerRight = Range.clip(powerRight, -1.0f, 1.0f);

            if (!gamepad1.left_bumper) {   // left bumper is full speed, otherwise half.
                powerLeft =  powerLeft / 2;
                powerRight = powerRight / 2;
            }

            robot.motorLeft.setPower(powerLeft);
            robot.motorRight.setPower(powerRight);

            /*  Boom Section */
            //tgtPowerBoom = this.gamepad1.right_trigger - this.gamepad1.left_trigger;
            tgtPowerBoom = -this.gamepad2.left_stick_y;
            tgtPowerStick = -this.gamepad2.right_stick_y;

            robot.motorStick.setPower(tgtPowerStick);

            if (robot.sensorBoomLimit.getState() == true && tgtPowerBoom < 0) {
                robot.motorBoom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                robot.motorBoom.setTargetPosition(60);
                robot.motorBoom.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.motorBoom.setPower(Math.abs(tgtPowerBoom));
            } else if (robot.sensorBoomLimit.getState() == true && tgtPowerBoom > 0) {
                robot.motorBoom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                robot.motorBoom.setTargetPosition(1150);
                robot.motorBoom.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.motorBoom.setPower(Math.abs(tgtPowerBoom));
            } else {
                //robot.motorBoom.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                if (robot.motorBoom.getCurrentPosition() < 100 || robot.motorBoom.getCurrentPosition() > 1100) {
                    robot.motorBoom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }
                robot.motorBoom.setPower(0);
            }
            if (robot.sensorBoomLimit.getState() == false) {
                //robot.motorBoom.setPower(0);
                initBoom();
            }



            telemetry.addData("stick", "  y=" + yValue + "  x=" + xValue);
            telemetry.addData("power", "  left=" + powerLeft + "  right=" + powerRight);
            telemetry.addData("Boom power", tgtPowerBoom);
            telemetry.addData("Boom encoder",robot.motorBoom.getCurrentPosition());
            telemetry.addData("Stick power", tgtPowerStick);
            telemetry.addData("Stick encoder",robot.motorStick.getCurrentPosition());

            // check to see if we need to move the servo.
            if (gamepad1.y) {
                // move to min pos
                tgtSampler += .005;
                sleep(50);
            }
            if (gamepad1.a) {
                // move to middle
                tgtSampler -= .005;
                sleep(50);
            }
            if (gamepad1.b) {
                // move to target position
                robot.servoSampler.setPosition(tgtSampler);
            }

            if (gamepad2.x) {
                initBoom();
            }

            if (gamepad2.b) {
                initStick();
            }

            // is switch pressed?
//            if (digitalTouch.getState() == false) {
//                // button is pressed
//                telemetry.addData("Button", "PRESSED");
//            } else {
//                // button not pressed
//                telemetry.addData("Button", "NOT PRESSED");
//            }

            telemetry.addData("Target Servo Position", "%.3f", tgtSampler);
            //telemetry.addData(">", "Robot Heading = %.1f", angles.firstAngle);
//            telemetry.addData("Servo Position", servoTest.getPosition());
//            telemetry.addData("Target Power", tgtPower);
//            telemetry.addData("Motor Power", motorTest.getPower());
//            telemetry.addData("Distance (cm)", sensorColorRange.getDistance(DistanceUnit.CM));
//            telemetry.addData("Status", "Running");

            telemetry.update();

        }

    }
    public void initBoom() {
        robot.motorBoom.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBoom.setPower(-0.2);
        while (robot.sensorBoomLimit.getState() == true) {
            // limit sensor not triggered, so keep moving boom
            sleep(10);
        }
        robot.motorBoom.setPower(0);
        robot.motorBoom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBoom.setTargetPosition(60);
        robot.motorBoom.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBoom.setPower(0.6);
        sleep(500);
        robot.motorBoom.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void initStick() {
        robot.motorStick.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorStick.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}