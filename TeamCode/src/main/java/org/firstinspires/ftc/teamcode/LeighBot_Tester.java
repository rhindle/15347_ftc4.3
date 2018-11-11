package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;


@TeleOp (name="Leighbot: Testing", group="Test")
//@Disabled
public class LeighBot_Tester extends LinearOpMode {
//    private Gyroscope imu;
//    private DcMotor motorTest;
//    private DigitalChannel digitalTouch;
//    private DistanceSensor sensorColorRange;
//    private Servo servoTest;

    HardwareLeighBot         robot   = new HardwareLeighBot();
    Orientation angles;

    @Override
    public void runOpMode() {
//        imu = hardwareMap.get(Gyroscope.class, "imu");
//        motorTest = hardwareMap.get(DcMotor.class, "motorTest");
//        digitalTouch = hardwareMap.get(DigitalChannel.class, "digitalTouch");
//        sensorColorRange = hardwareMap.get(DistanceSensor.class, "sensorColorRange");
//        servoTest = hardwareMap.get(Servo.class, "servoTest");
//
//        // set digital channel to input mode
//        digitalTouch.setMode(DigitalChannel.Mode.INPUT);

        robot.init(hardwareMap);

        // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.boomDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Send telemetry message to alert driver that we are calibrating;
        telemetry.addData(">", "Calibrating Gyro");    //
        telemetry.update();

        // make sure the gyro is calibrated before continuing
        while (!isStopRequested() && !robot.imu.isGyroCalibrated())  {
            sleep(50);
            idle();
        }

        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.boomDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // LK: The following help with inaccuracy in my test robot; not present in original gyro code
//        robot.leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        robot.rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.boomDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (Display Gyro value), and reset gyro before we move..
        while (!isStarted()) {
            angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData(">", "Robot Heading = %.1f", angles.firstAngle);
            telemetry.update();
            sleep(100);
        }

//        // Wait for the game to start (driver presses PLAY)
//        waitForStart();

        double tgtPowerLeft = 0;
        double tgtPowerRight = 0;
        double tgtPowerBoom = 0;

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


            tgtPowerLeft = -this.gamepad1.left_stick_y;
            tgtPowerRight = -this.gamepad1.right_stick_y;
            tgtPowerBoom = this.gamepad1.right_trigger - this.gamepad1.left_trigger;


            robot.leftDrive.setPower(tgtPowerLeft);
            robot.rightDrive.setPower(tgtPowerRight);
//
//            if (robot.sensorBoom.getState() == true && robot.boomDrive.getCurrentPosition()>20 && tgtPowerBoom < 0) {
//                robot.boomDrive.setPower(tgtPowerBoom);
//            } else if (robot.sensorBoom.getState() == true && robot.boomDrive.getCurrentPosition()<1150 && tgtPowerBoom > 0) {
//                robot.boomDrive.setPower(tgtPowerBoom);
//            } else {
//                robot.boomDrive.setPower(0);
//            }
//            if (robot.sensorBoom.getState() == false) {
//                //robot.boomDrive.setPower(0);
//                initBoom();
//            }

            if (robot.sensorBoom.getState() == true && tgtPowerBoom < 0) {
                robot.boomDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                robot.boomDrive.setTargetPosition(60);
                robot.boomDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.boomDrive.setPower(Math.abs(tgtPowerBoom));
            } else if (robot.sensorBoom.getState() == true && tgtPowerBoom > 0) {
                robot.boomDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                robot.boomDrive.setTargetPosition(1150);
                robot.boomDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.boomDrive.setPower(Math.abs(tgtPowerBoom));
            } else {
                //robot.boomDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                if (robot.boomDrive.getCurrentPosition() < 100 || robot.boomDrive.getCurrentPosition() > 1100) {
                    robot.boomDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }
                robot.boomDrive.setPower(0);
            }
            if (robot.sensorBoom.getState() == false) {
                //robot.boomDrive.setPower(0);
                initBoom();
            }

            telemetry.addData("Target Power L", tgtPowerLeft);
//            telemetry.addData("Left Motor Power", leftMotor.getPower());
//            telemetry.addData("Status", "Running");
            telemetry.addData("Target Power R", tgtPowerRight);
//            telemetry.addData("Right Motor Power", rightMotor.getPower());
//            telemetry.addData("Status", "Running");
//            telemetry.update();
            telemetry.addData("Target Power B", tgtPowerBoom);
            telemetry.addData("Boom encoder",robot.boomDrive.getCurrentPosition());

            // check to see if we nned to move the servo.
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
                robot.samplerArm.setPosition(tgtSampler);
            }

            if (gamepad1.x) {
                initBoom();
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
        robot.boomDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.boomDrive.setPower(-0.2);
        while (robot.sensorBoom.getState() == true) {
            // limit sensor not triggered, so keep moving boom
            sleep(10);
        }
        robot.boomDrive.setPower(0);
        robot.boomDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.boomDrive.setTargetPosition(60);
        robot.boomDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.boomDrive.setPower(0.6);
        sleep(500);
        robot.boomDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}