/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

// LK: I wanted example code that worked with the Rev Expansion Hub IMU instead of the
//     Modern Robotics Gyro.  I started with relevant code copied from
//     github.com/realRoshanRaj/FTC_MotionProfiling/.   Since I made only minimal
//     changes, I make no claim to the code, but am including this note and updates to
//     the comments below to hopefully help others find this example when searching.

// LK: Final note - This code not re-tested yet after making lots of minor changes!
//     I may have broken something.

package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

//import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

/**
 * This file illustrates the concept of driving a path based on IMU heading and encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that you have a Bosch BNO055IMU (included in Rev Expandion Hub)
 *  with the name "imu"
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *  This code requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 *  Note: in this example, all angles are referenced to the initial coordinate frame set during the
 *  the IMU Calibration process.  (Unlike the MR Gyro, there is no equivalent to a
 *  resetZAxisIntegrator() call, although similar functionality can be coded.)
 *
 *  The angle of movement/rotation is assumed to be a standardized rotation around the robot Z axis,
 *  which means that a Positive rotation is Counter Clock Wise, looking down on the field.
 *  This is consistent with the FTC field coordinate conventions set out in the document:
 *  ftc_app\doc\tutorial\FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Leighbot: 2 IMU, sampler", group="Test")
//@Disabled
public class LeighBot_IMU_Sampler2 extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareLeighBot         robot   = new HardwareLeighBot();   // Use a Pushbot's hardware
//    BNO055IMU               imu;                               // The IMU sensor object
//
//    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
//    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
//    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
//    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
//                                                      (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.2; //0.35; //0.7;     // Nominal speed for better accuracy.
    static final double     TURN_SPEED              = 0.2; //0.25; // .5;     // Nominal half speed for better accuracy.

    static final double     HEADING_THRESHOLD       = 0.2; //1 ;      // As tight as we can make it with an integer gyro
                                                               // LK: Above was required with MR gyro, but I did
                                                               //     not test with smaller values with IMU
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.01; //0.15;     // Larger is more responsive, but also less stable
                                                               // LK: I had bad results, used 0.01 instead during testing;
                                                               //     I was using lower drive/turn speeds though.

    Orientation angles;

    boolean exitFlag, testForMinerals;
    int testForMineralsCounter;
    int testForMineralsCounterMax;

    @Override
    public void runOpMode() {

        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F, 0F, 0F};
        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;
        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;

        int saveRightPos;
        double netMovement;
        double sampleHeading;

        /*
         * Initialize the standard drive system variables.
         * The init() method of the hardware class does most of the work here
         */
        robot.init(hardwareMap);

//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
//        parameters.accelUnit            = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.calibrationDataFile  = "BNO055IMUCalibration.json"; // see the calibration sample opmode
//        parameters.loggingEnabled       = true;
//        parameters.loggingTag           = "IMU";
//        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
//        imu = hardwareMap.get(BNO055IMU.class, "imu");
//        imu.initialize(parameters);

        // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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

        // LK: The following help with inaccuracy in my test robot; not present in original gyro code
        robot.leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (Display Gyro value), and reset gyro before we move..
        while (!isStarted()) {
            angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData(">", "Robot Heading = %.1f", angles.firstAngle);
            telemetry.update();
            sleep(100);
        }

        testForMinerals = false;
        exitFlag = false;
        gyroDrive(DRIVE_SPEED, 16.5, 0);  // was 21, subtract 4.5
        gyroTurn( TURN_SPEED,  90);
        gyroDrive(DRIVE_SPEED, -14, 90);  // was 9, but I measured wrong!
//        sleep(5000);

        robot.samplerArm.setPosition(robot.SAMPLER_READ);

//        while (opModeIsActive()) {
//            // convert the RGB values to HSV values.
//            // multiply by the SCALE_FACTOR.
//            // then cast it back to int (SCALE_FACTOR is a double)
//            Color.RGBToHSV((int) (robot.sensorColor.red() * SCALE_FACTOR),
//                    (int) (robot.sensorColor.green() * SCALE_FACTOR),
//                    (int) (robot.sensorColor.blue() * SCALE_FACTOR),
//                    hsvValues);
//
//            // send the info back to driver station using telemetry function.
//            telemetry.addData("Distance (cm)",
//                    String.format(Locale.US, "%.02f", robot.sensorDistance.getDistance(DistanceUnit.CM)));
//            telemetry.addData("Alpha", robot.sensorColor.alpha());
//            telemetry.addData("Red  ", robot.sensorColor.red());
//            telemetry.addData("Green", robot.sensorColor.green());
//            telemetry.addData("Blue ", robot.sensorColor.blue());
//            telemetry.addData("Hue", hsvValues[0]);
//
//            if (robot.sensorDistance.getDistance(DistanceUnit.CM) < 10 && hsvValues[0] < 50) {
//                telemetry.addData(">", "Candidate Found!");
//            } else {
//                telemetry.addData(">", "no match");
//            }
//
//            telemetry.update();
//
//            sleep(50);
//            idle();
//        }

//       while (opModeIsActive()) {
//           testForMinerals=findMineral();
//           telemetry.update();
//          sleep(50);
//           idle();
//       }

        //robot.samplerArm.setPosition(robot.SAMPLER_UP);
        testForMinerals = true;
        testForMineralsCounter = 0;
        exitFlag = false;
        saveRightPos = robot.rightDrive.getCurrentPosition();
//        moveCounts = (int)(distance * robot.COUNTS_PER_INCH);
//        newLeftTarget = robot.leftDrive.getCurrentPosition() + moveCounts;
//        newRightTarget = robot.rightDrive.getCurrentPosition() + moveCounts;
        // 26 inches at 0 deg

        robot.samplerArm.setPosition(robot.SAMPLER_READ);
        sleep(100);

        sampleHeading=89.0; //90.0;  //90 in auto, 0 when testing just that part.
        gyroDrive(DRIVE_SPEED*.65, 40.0, sampleHeading);    // Drive FWD [31+4+4] inches //0.13
        testForMinerals = false;

        if (exitFlag) {
            exitFlag = false;
            gyroDrive(DRIVE_SPEED*.75, -4, sampleHeading);  //0.15
            robot.samplerArm.setPosition(robot.SAMPLER_PUSH);
            gyroDrive(DRIVE_SPEED, 6, sampleHeading);  //0.2
            robot.samplerArm.setPosition(robot.SAMPLER_UP);
            netMovement=(robot.rightDrive.getCurrentPosition()-saveRightPos)/robot.COUNTS_PER_INCH;
            telemetry.addData("Distance", "%2.1f", netMovement);
            telemetry.update();
            gyroDrive(DRIVE_SPEED, 40.0-netMovement, sampleHeading);  //0.2
        }

        robot.samplerArm.setPosition(robot.SAMPLER_UP);  // if we missed the mineral raise the arm
        //sleep(2000);

        gyroTurn( 0.25,  120);
        gyroDrive(0.75, 55, 120);
        gyroTurn( 0.25,  135);
        gyroDrive(1, -75, 135);

        // LK: This fuctionality not provided with the IMU code
//        gyro.resetZAxisIntegrator();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        // Put a hold after each turn
        // LK: The CW/CCW notation is opposite my interpretation; angles are correct.
        //     In testing, pausing between motions is helpful to observer what's going on,
        //     so I added sleep statements.
//        gyroDrive(DRIVE_SPEED, 48.0, 0.0);    // Drive FWD 48 inches
//        sleep(2000);
//        gyroTurn( TURN_SPEED, -45.0);         // Turn  CCW to -45 Degrees
//        sleep(2000);
//        gyroHold( TURN_SPEED, -45.0, 0.5);    // Hold -45 Deg heading for a 1/2 second
//        sleep(2000);
//        gyroDrive(DRIVE_SPEED, 12.0, -45.0);  // Drive FWD 12 inches at 45 degrees
//        sleep(2000);
//        gyroTurn( TURN_SPEED,  45.0);         // Turn  CW  to  45 Degrees
//        sleep(2000);
//        gyroHold( TURN_SPEED,  45.0, 0.5);    // Hold  45 Deg heading for a 1/2 second
//        sleep(2000);
//        gyroTurn( TURN_SPEED,   0.0);         // Turn  CW  to   0 Degrees
//        sleep(2000);
//        gyroHold( TURN_SPEED,   0.0, 1.0);    // Hold  0 Deg heading for a 1 second
//        sleep(2000);
//
//        // LK: I changed the remaining steps so it would return to original position
//        gyroDrive(DRIVE_SPEED,-24.0, 0.0);    // Drive REV 24 inches
//        sleep(2000);
//        gyroTurn( TURN_SPEED, -45.0);         // Turn  CCW to -45 Degrees
//        sleep(2000);
//        gyroDrive(DRIVE_SPEED, -12.0, -45.0);  // Drive REV 12 inches at 45 degrees
//        sleep(2000);
//        gyroTurn( TURN_SPEED,   0.0);         // Turn  CW  to   0 Degrees
//        sleep(2000);
//        gyroDrive(DRIVE_SPEED,-24.0, 0.0);    // Drive REV 24 inches
//        sleep(2000);

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }


    boolean findMineral () {
        boolean foundGold = false;
        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F, 0F, 0F};
        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;
        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;

        // convert the RGB values to HSV values.
        // multiply by the SCALE_FACTOR.
        // then cast it back to int (SCALE_FACTOR is a double)
        Color.RGBToHSV((int) (robot.sensorColor.red() * SCALE_FACTOR),
                (int) (robot.sensorColor.green() * SCALE_FACTOR),
                (int) (robot.sensorColor.blue() * SCALE_FACTOR),
                hsvValues);

        // send the info back to driver station using telemetry function.
        telemetry.addData("Distance (cm)",
                String.format(Locale.US, "%.02f", robot.sensorDistance.getDistance(DistanceUnit.CM)));
//        telemetry.addData("Alpha", robot.sensorColor.alpha());
//        telemetry.addData("Red  ", robot.sensorColor.red());
//        telemetry.addData("Green", robot.sensorColor.green());
//        telemetry.addData("Blue ", robot.sensorColor.blue());
        telemetry.addData("Hue", hsvValues[0]);

        if (robot.sensorDistance.getDistance(DistanceUnit.CM) < 10 && hsvValues[0] < 50) {
            telemetry.addData(">", "Candidate Found!");
            testForMineralsCounter++;
            //foundGold = true;
        } else {
            telemetry.addData(">", "no match");
            testForMineralsCounter--;
        }

        if (testForMineralsCounter<0) {
            testForMineralsCounter=0;
        }
        if (testForMineralsCounter > testForMineralsCounterMax) {
            testForMineralsCounterMax = testForMineralsCounter;
        }
        telemetry.addData("MnlCount",testForMineralsCounter);
        telemetry.addData("MaxCount",testForMineralsCounterMax);
        if (testForMineralsCounter>2) {
            foundGold=true;
        }
//        telemetry.update();
//
//        sleep(50);
//        idle();

        return foundGold;
    }

   /**
    *  Method to drive on a fixed compass bearing (angle), based on encoder counts.
    *  Move will stop if either of these conditions occur:
    *  1) Move gets to the desired position
    *  2) Driver stops the opmode running.
    *
    * @param speed      Target speed for forward motion.  Should allow for _/- variance for adjusting heading
    * @param distance   Distance (in inches) to move from current position.  Negative distance means move backwards.
    * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
    *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
    *                   If a relative angle is required, add/subtract from current heading.
    */
    public void gyroDrive ( double speed,
                            double distance,
                            double angle) {

        int     newLeftTarget;
        int     newRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * robot.COUNTS_PER_INCH);
            newLeftTarget = robot.leftDrive.getCurrentPosition() + moveCounts;
            newRightTarget = robot.rightDrive.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            robot.leftDrive.setTargetPosition(newLeftTarget);
            robot.rightDrive.setTargetPosition(newRightTarget);

            robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.leftDrive.setPower(speed);
            robot.rightDrive.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() && (!exitFlag) &&
                    (robot.leftDrive.isBusy() && robot.rightDrive.isBusy())) {

                if (testForMinerals) {
                    exitFlag=findMineral();
                }

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                robot.leftDrive.setPower(leftSpeed);
                robot.rightDrive.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
                telemetry.addData("Actual",  "%7d:%7d",      robot.leftDrive.getCurrentPosition(),
                        robot.rightDrive.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                telemetry.update();
            }

            // Stop all motion;
            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.leftDrive.setPower(leftSpeed);
        robot.rightDrive.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        robotError = targetAngle - angles.firstAngle;
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

}
