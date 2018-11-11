// Started with http://stemrobotics.cs.pdx.edu/node/4737?root=4196
// but changed just about everything.

// simple teleop program that drives bot using controller joysticks in arcade mode.
// this code monitors the period and stops when the period is ended.

package org.firstinspires.ftc.teamcode;

        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.HardwareLeighBot;

@TeleOp(name="LK Proportional Arcade", group="Test")
//@Disabled
public class LeighBot_ArcadeDrive extends LinearOpMode
{
//    DcMotor leftMotor, rightMotor;
    float   leftPower, rightPower, xValue, yValue, rValue, xxValue;
    float  leftTest, rightTest;

    HardwareLeighBot robot   = new HardwareLeighBot();

    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException
    {
//        leftMotor = hardwareMap.dcMotor.get("left_motor");
//        rightMotor = hardwareMap.dcMotor.get("right_motor");
//        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        robot.init(hardwareMap);

        telemetry.addData("Mode", "waiting");
        telemetry.update();

        // wait for start button.

        waitForStart();

        while (opModeIsActive())
        {
            yValue = -gamepad1.left_stick_y;   // throttle
            xValue = -gamepad1.right_stick_x;  // steer
//            rValue = -gamepad1.right_stick_x;  // rotate

//            if (yValue == 0 && xValue == 0) {  // if left stick centered, use right stick for turn
//                xValue = rValue;
//            } else {
//                if (yValue < 0) {              // mimic car steering wrt reversing
//                    xValue = -xValue;
//                }
//                xValue = xValue * Math.abs(yValue);  // make turn proportional to speed
//            }


            if (!gamepad1.right_bumper) {
                if (yValue < 0) {              // mimic car steering wrt reversing
                    xValue = -xValue;
                }
                xValue = xValue * Math.abs(yValue);  // make turn proportional to speed
            }

            leftPower =  yValue - xValue;
            rightPower = yValue + xValue;

            leftPower = Range.clip(leftPower,-1.0f,1.0f);
            rightPower = Range.clip(rightPower, -1.0f, 1.0f);

            if (gamepad1.left_bumper) {
                leftPower =  leftPower / 2;
                rightPower = rightPower / 2;
            }

//            robot.leftDrive.setPower(Range.clip(leftPower, -1.0, 1.0));
//            robot.rightDrive.setPower(Range.clip(rightPower, -1.0, 1.0));
            robot.leftDrive.setPower(leftPower);
            robot.rightDrive.setPower(rightPower);

            telemetry.addData("Mode", "running");
            telemetry.addData("stick", "  y=" + yValue + "  x=" + xValue);
            telemetry.addData("power", "  left=" + leftPower + "  right=" + rightPower);
//            telemetry.addData("xxValue", xxValue);
//            telemetry.addData("test", "  left=" + leftTest + "  right=" + rightTest);
            telemetry.update();

            idle();
        }
    }
}

