/*
    MechanumTeleOp.java

    A Linear opmode class to be our the teleop method
    to try out mecanum drivetrain

    This file is a modified version from the FTC SDK.

    Modifications by FTC Team #10273 Cat in the Hat Comes Back
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@TeleOp(name="Mechanum TeleOp", group="CatTeleOp")

public class MechanumTeleOp extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    /* Declare OpMode members. */
    MechanumHardware robot; // use the class created for the hardware

    // constructor for class
    public MechanumTeleOp() {
        robot = new MechanumHardware();
    }

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize the hardware
        robot.init(hardwareMap, this);
        telemetry.addData("Status", "Initialized...  BOOM!");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // go
        runtime.reset();
        double driveSpeed;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            /**
             * ---   ______________________   ---
             * ---   MECHANUM DRIVE CODE!!!   ---
             * ---    \/ \/ \/ \/ \/ \/ \/    ---
             */

            double leftFront = 0;
            double rightFront = 0;
            double leftBack = 0;
            double rightBack = 0;
            //  ---  SPEED BOOST!!!  --- //
            if (gamepad1.left_bumper) {
                driveSpeed = 1;
            } else {
                driveSpeed = 0.6;
            }
            // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
            leftFront  = -gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x;
            rightFront = -gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x;
            leftBack   = -gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x;
            rightBack  = -gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x;

            leftFront = robot.limitRange(leftFront, -1.0, 1.0) * driveSpeed;
            rightFront = robot.limitRange(rightFront, -1.0, 1.0) * driveSpeed;
            leftBack = robot.limitRange(leftBack, -1.0, 1.0) * driveSpeed;
            rightBack = robot.limitRange(rightBack, -1.0, 1.0) * driveSpeed;

            // drive //
            robot.drive(leftFront, rightFront, leftBack, rightBack);


            /**
             * ---   _________   ---
             * ---   TELEMETRY   ---
             * ---   \/ \/ \/    ---
             */
            telemetry.addData("Left Front Power:", "%.2f", leftFront);
            telemetry.addData("Right Front Power:", "%.2f", rightFront);
            telemetry.addData("Left Back Power:", "%.2f", leftBack);
            telemetry.addData("Right Back Power:", "%.2f", rightBack);
            telemetry.update();
            idle();
        }
    }
}
