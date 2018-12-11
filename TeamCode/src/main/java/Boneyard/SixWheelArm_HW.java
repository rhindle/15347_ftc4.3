package Boneyard; //    this is telling the robot what data you are useing

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
// gives the robot info on your stuff ;)

@TeleOp(name="SixWheelArm_HW", group="MSI")
//@Disabled
public class SixWheelArm_HW extends LinearOpMode {

    HardwareMSI robot   = new HardwareMSI();   // Use defined hardware
    Orientation angles;   // For use with the IMU

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // run until the end of the match (driver presses STOP)
        double tgtPower = 0;
        double tgtPower2 = 0;
        double tgtPower3 = 0;
        double NewIdea = 0;
        while (opModeIsActive()) {


            telemetry.addData("Target Power", tgtPower);
            telemetry.addData("Left Motor Power", robot.leftMotor.getPower());
            telemetry.addData("Status", "Running");
            telemetry.addData("Target Power2", tgtPower2);
            telemetry.addData("Right Motor Power", robot.rightMotor.getPower());
            telemetry.addData("Status", "Running");
            telemetry.addData("Shoulder Servo port 1", robot.shoulderServo.getPower());
            telemetry.addData("Elbow servo port 2", robot.elbowServo.getPosition());
            telemetry.addData("Claw servo port 4", robot.clawServo.getPosition());
            telemetry.addData("wrist servo port ", robot.wristServo.getPosition());
            telemetry.addData("Distance (cm)", robot.sensorColorRange.getDistance(DistanceUnit.CM));
            telemetry.addData("Alpha", robot.sensorColor.alpha());
            telemetry.addData("Red  ", robot.sensorColor.red());
            telemetry.addData("Green", robot.sensorColor.green());
            telemetry.addData("Blue ", robot.sensorColor.blue());
            telemetry.update();

            // if (digitalTouch.getState() == false) {
            //       telemetry.addData("Button", "PRESSED");
            // } else {
            //    telemetry.addData("Button", "NOT PRESSED");

// controler 1
            // Raise arm at robot base "shoulder"
            if (gamepad1.a ){
                robot.shoulderServo.setPower( 0.5);
            }
            if (gamepad1.b ) {
                robot.shoulderServo.setPower(-0.5);
            }
            if (!gamepad1.a && !gamepad1.b)
                robot.shoulderServo.setPower(0);

            // Raise arm at arm joint elbow
            if (gamepad1.x) {
                robot.elbowServo.setPosition(robot.elbowServo.getPosition() + 0.01);
            } else if (gamepad1.y) {
                robot.elbowServo.setPosition(robot.elbowServo.getPosition() - 0.01);
            }

            // Open and close claw
            if (gamepad1.right_trigger == 1) {
                robot.clawServo.setPosition(robot.clawServo.getPosition() + 0.02);

            } else if (gamepad1. right_trigger == 1) {
                robot.clawServo.setPosition(robot.clawServo.getPosition() - 0.02);

            } else if (gamepad1.left_trigger == 0 & gamepad1.right_trigger == 0  ) {
                robot.clawServo.setPosition(0);
            }

            // Open and close claw2
            if (gamepad1.right_trigger == 1) {
                robot.clawServo2.setPower(1);

            } else if (gamepad1.left_trigger == 1) {
                robot.clawServo2.setPower(-1);

            } else if (gamepad1.left_trigger == 0 & gamepad1.right_trigger == 0  ) {
                robot.clawServo2.setPower(0);
            }

            // dunk the wrist bro
            if (gamepad1.dpad_left) {
                robot.wristServo.setPosition(robot.wristServo.getPosition() + 0.02);
            } else if (gamepad1.dpad_right) {
                robot.wristServo.setPosition(robot.wristServo.getPosition() - 0.02);
            }


// controler 2
            //driving
           if (gamepad2.a)
               NewIdea = 1;
            if (gamepad2.b)
                NewIdea = .5;
            tgtPower = this.gamepad2.left_stick_y;
            robot.leftMotor.setPower(tgtPower * NewIdea);
            tgtPower2 = -this.gamepad2.right_stick_y;
            robot.rightMotor.setPower(tgtPower2 * NewIdea);

            // lift up and down
            if (gamepad2.right_trigger == 1 ){
                robot.lifter.setPower( 0.5);
            }
            if (gamepad2.left_trigger == 1 ) {
                robot.lifter.setPower(-0.5);
            }
            if (gamepad2.right_trigger == 0 && gamepad2.left_trigger == 0)
                robot.lifter.setPower(0);
            }
        }
    }

