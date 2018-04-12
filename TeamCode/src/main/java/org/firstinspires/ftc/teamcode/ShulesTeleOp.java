/*
    ShulesTeleOp.java

    A Linear opmode class to be our main teleop method
    This version is for the mecanum intake.

    This file is a modified version from the FTC SDK.

    Modifications by FTC Team #10273 Cat in the Hat Comes Back
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Shules TeleOp", group="CatTeleOp")

public class ShulesTeleOp extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime driveModeSwitch     = new ElapsedTime();
    private HardwareCatBot.TeleOpDriveMode driveMode = HardwareCatBot.TeleOpDriveMode.TankDrive;

    /* Declare OpMode members. */
    HardwareCatBot robot; // use the class created for the hardware

    // constructor for class
    public ShulesTeleOp() {
        robot = new HardwareCatBot();
    }

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize/INIT the hardware
        robot.init(hardwareMap, this, false, false);
        telemetry.addData("Status", "Initialized...  BOOM!");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // Set jewel smacker to up pos
        robot.jewelSmackerUp();  // Init After start b/c it was called a "major movement"... ugh!

        // LED code
        robot.blinkyTimer.reset();
        robot.blinky(robot.allianceColor);
        robot.endgameOfAuto.reset();

        // go
        runtime.reset();
        double driveSpeed;
        double intakeRotateSpeed;
        double lifterPowerAdd;


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if (runtime.seconds() > 240){
                telemetry.addData("Arm: ", "dead");
                break;
            }


            /**
             * ---------   __________________   ---------
             * ---------   Gamepad 1 STUFF!!!   ---------
             * ---------    \/ \/ \/ \/ \/      ---------
             */


            /*
            Our Enums for different driving control modes which is changed by pressing "x + y + dPadUp"
             */
            if (gamepad1.x  &&  gamepad1.y  &&  gamepad1.dpad_up  && (driveModeSwitch.seconds() > 1.5)) {
                switch (driveMode) {
                    case TankDrive:
                        driveMode = HardwareCatBot.TeleOpDriveMode.SingleStick;
                        driveModeSwitch.reset();
                        break;
                    case SingleStick:
                        driveMode = HardwareCatBot.TeleOpDriveMode.TankDrive;
                        driveModeSwitch.reset();
                        break;
                }
            }

            double left = 0;
            double right = 0;
            //  ---  SPEED/SLOW BOOST!!!  --- //
            if (gamepad1.left_bumper) {
                driveSpeed = 1;
            } else if (gamepad1.right_bumper) {
                driveSpeed = 0.4;
            } else {
                driveSpeed = 0.8;
            }


            // Switch between more drive mode types
            switch (driveMode) {

                //  --- Yay it works!! ---
                case TankDrive:
                    // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
                    left = -gamepad1.left_stick_y * driveSpeed;
                    right = -gamepad1.right_stick_y * driveSpeed;
                    break;

                //  --- Yay it works!! ---
                case SingleStick:
                /*
                 *   calc values HERE:     ||  ||  ||  ||  ||
                 *                         \/  \/  \/  \/  \/
                 */
                    float x = gamepad1.left_stick_x;
                    float y = gamepad1.left_stick_y;

                    if (Math.abs(y) < 0.05) {
                        right = x * 2;
                        left = -x * 2;
                    } else if (y > 0) {                 // Forwards
                        right = y - (x / 1.25);
                        left = y + (x / 1.25);
                    } else if (y < 0) {                 // Backwards
                        right = y + (x / 1.25);
                        left = y - (x / 1.25);
                    }

                    left  = -left  * driveSpeed;
                    right = -right * driveSpeed;
                    break;
            }
            // drive using either TankDrive or SingleStick //
            robot.drive(left, right);




            /**
             * ---------   __________________   ---------
             * ---------   Gamepad 2 STUFF!!!   ---------
             * ---------    \/ \/ \/ \/ \/      ---------
             */

            // lifter motor code //
            if (gamepad2.x) {
                lifterPowerAdd = 0.15;
            } else {
                lifterPowerAdd = 0.00;
            }
            //// TODO: 1/27/2018 Add touch sensors to robot to detect the top and bottom of the lifter arm
            robot.lifterMotor.setPower(-gamepad2.left_stick_y + lifterPowerAdd);  // Added a little bit to keep the heavy intake up...
            robot.periodicTeleOpTask();


            /* --- servo rotatey thingy --- */
            // Set the servo speed...
            intakeRotateSpeed = robot.SERVO_NEUTRAL_POWER; // - gamepad2.right_stick_x/2;
            // Fail safe code for when the end of the world comes...
            if (intakeRotateSpeed > 1.0) {
                intakeRotateSpeed = 1.0;
            } else if (intakeRotateSpeed < 0.0) {
                intakeRotateSpeed = 1.0;
            }
            // Actually rotate the intake
            robot.intakeRotateyThing.setPosition(intakeRotateSpeed);


            // code for the intake motors //
            robot.intakeMotorLeft.setPower(gamepad2.left_trigger);
            robot.intakeMotorRight.setPower(gamepad2.right_trigger);
            if (gamepad2.left_bumper){
                robot.intakeMotorLeft.setPower(-0.8);
            }
            if (gamepad2.right_bumper){
                robot.intakeMotorRight.setPower(-0.8);
            }


            // Relic Arm IN and OUT //
            if (gamepad2.dpad_left) {
                robot.relicArmOut();
            } else if (gamepad2.dpad_right) {
                robot.relicArmIn();
            } else {
                robot.relicArmStop();
            }


            // Move the Elbow //
            if (gamepad2.dpad_down){
                robot.relicElbow.setPower(0.7);
            } else if(gamepad2.dpad_up){
                robot.relicElbow.setPower(-0.7);
            } else {
                robot.relicElbow.setPower(0.0);
            }


            // Gripper code //
            if (gamepad2.a) {
                robot.relicGripper.setPosition(robot.RELIC_GRIPPER_CLOSE);
            } else if (gamepad2.b) {
                robot.relicGripper.setPosition(robot.RELIC_GRIPPER_OPEN);
            }


            /**
             * ---   _________   ---
             * ---   TELEMETRY   ---
             * ---   \/ \/ \/    ---
             */
            telemetry.addData("ArmIndex Pos:", robot.armIndex);
            telemetry.addData("(GP2.right_stick) LiterPos", "Target: %2d Current: %2d", robot.lifterMotor.getTargetPosition(), robot.lifterMotor.getCurrentPosition());
            telemetry.addData("(GP1.x + y + dPadUp)DriveMode:", driveMode);
            telemetry.addData("Left Power:", "%.2f", left);
            telemetry.addData("Right Power:", "%.2f", right);
            telemetry.update();
            idle();
        }
    }

    public int limitRange(int number, int min, int max) {
        return Math.min(Math.max(number, min), max);
    }
}
