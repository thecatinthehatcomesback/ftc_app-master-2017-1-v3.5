/**
 Tester.java

 A Linear opmode class to test all kinds of stuff.
      Color Sensor
      Arm position
      Jewel Smaker position
      Turning using IMU

 This file is a modified version from the FTC SDK.

 Modifications by FTC Team #10273 Cat in the Hat Comes Back
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


@Autonomous(name="Tester", group="CatAuto")
public class Tester extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareCatBot robot = new HardwareCatBot();
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime delaytimer = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap, this, false, false);
        robot.lifterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();


        // After init is pushed but before Start we can change the delay using dpad up/down
        delaytimer.reset();

        while (!opModeIsActive()) {

            //Don't need "waitForStart()" since we've been looping waiting for opmode to be enabled.

            // Do the delay asked for.
            delaytimer.reset();

        }
        //initilize the sensor :)
        robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 250);

        /*\
        * Runs after hit start
        * DO STUFF FOR MODE!!!!!!!!!!!
        *
        \*/
        double jewelPos;
        double flipperPos;
        double intakeRotateSpeed;
        int lifterPosition = 0;

        runtime.reset();
        robot.jewelSmackerDown();
        robot.robotWait(2);
        robot.jewelSmackerUp();
        robot.lifterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lifterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (opModeIsActive()){
            jewelPos = robot.jewelArm.getPosition();
            flipperPos = robot.jewelFlipper.getPosition();

            // Jewel Flipper
            if (gamepad1.dpad_up  &&  runtime.milliseconds() > 300) {
                robot.jewelFlipper.setPosition(flipperPos + 0.1);
                runtime.reset();
            } else if (gamepad1.dpad_down  &&  runtime.milliseconds() > 300) {
                robot.jewelFlipper.setPosition(flipperPos - 0.1);
                runtime.reset();
            }
            // Arm Servo
            if (gamepad1.dpad_right  &&  runtime.milliseconds() > 300) {
                robot.jewelArm.setPosition(jewelPos + 0.1);
                runtime.reset();
            } else if (gamepad1.dpad_left  &&  runtime.milliseconds() > 300) {
                robot.jewelArm.setPosition(jewelPos - 0.1);
                runtime.reset();
            }
            if (gamepad1.y && runtime.milliseconds() > 50){
                lifterPosition += 10;
                runtime.reset();
                robot.lifterMotor.setTargetPosition(lifterPosition);
                robot.lifterMotor.setPower(0.7);
            }
            if (gamepad1.a && runtime.milliseconds() > 50) {
                lifterPosition -= 10;
                runtime.reset();
                robot.lifterMotor.setTargetPosition(lifterPosition);
                robot.lifterMotor.setPower(0.7);
            }

            // servo rotatey thingy //
            intakeRotateSpeed = robot.SERVO_NEUTRAL_POWER - (gamepad2.right_stick_x/1.5);
            robot.intakeRotateyThing.setPosition(intakeRotateSpeed);

            // Lifter Motor //
            //robot.lifterMotor.setPower(-gamepad2.left_stick_y);
            robot.periodicTask();

            // code for the intake motors //
            robot.intakeMotorLeft.setPower(gamepad2.left_trigger);
            robot.intakeMotorRight.setPower(gamepad2.right_trigger);
            if (gamepad2.left_bumper){
                robot.intakeMotorLeft.setPower(-0.5);
            }
            if (gamepad2.right_bumper){
                robot.intakeMotorRight.setPower(-0.5);
            }

            // IMU Sensor
            Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


            /**
             *  ...Telemetry...
             */
            telemetry.addData("xyz:", "%.1f, %.1f, %.1f", angles.firstAngle, angles.secondAngle, angles.thirdAngle);
            telemetry.addData("Jewel Position:", jewelPos);
            telemetry.addData("Flipper Position:", flipperPos);
            telemetry.addData("Current time:", runtime.seconds());
            telemetry.addData("rotatey thingy:", intakeRotateSpeed);
            telemetry.addData("alpha:", robot.jewelColors.alpha());
            telemetry.addData("red:", robot.jewelColors.red());
            telemetry.addData("blue:", robot.jewelColors.blue());
            telemetry.addData("green:", robot.jewelColors.green());
            telemetry.addData("lifter position", lifterPosition);

            /**
             * Telemetry for Glyph Censor
             */
            /*
            telemetry.addData("alpha:", robot.TopGlyphCensor.alpha());
            telemetry.addData("red:", robot.TopGlyphCensor.red());
            telemetry.addData("blue:", robot.TopGlyphCensor.blue());
            telemetry.addData("green:", robot.TopGlyphCensor.green());
            telemetry.addData("dist:", robot.TopGlyphDist.getDistance(DistanceUnit.CM));
            telemetry.addData("GlyphFinder:", robot.findGlyphColor(robot.TopGlyphCensor, robot.TopGlyphDist));
            */

            telemetry.update();

        }

    }
}
