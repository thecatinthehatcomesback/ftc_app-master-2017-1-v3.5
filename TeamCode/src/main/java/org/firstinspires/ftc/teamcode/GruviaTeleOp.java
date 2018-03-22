/*
    GruviaTeleOp.java

    A Linear opmode class to be our main teleop method
    This was used with the gripper for the 2 qualifying tournaments
     It has now been replaced when we went to the mecanum intake.

    This file is a modified version from the FTC SDK.

    Modifications by FTC Team #10273 Cat in the Hat Comes Back
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Disabled
@TeleOp(name="Gruvia TeleOp", group="CatTeleOp")

public class GruviaTeleOp extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime driveModeSwitch     = new ElapsedTime();
    private ElapsedTime liftArmSwitch       = new ElapsedTime();
    private ElapsedTime gripperArmSwitch    = new ElapsedTime();
    private HardwareCatBot.TeleOpDriveMode driveMode = HardwareCatBot.TeleOpDriveMode.TankDrive;


    /* Declare OpMode members. */
    HardwareCatBot robot; // use the class created for the hardware

    // constructor for class
    public GruviaTeleOp() {
        robot = new HardwareCatBot();
    }

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize/INIT the hardware
        robot.init(hardwareMap, this, false, true);
        robot.gripperMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("Status", "Initialized...  BOOM!");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // Set jewel smacker to up pos
        robot.jewelSmackerUp();  // Init After start b/c it was called a "major movement"... ugh!

        // go
        runtime.reset();
        double driveSpeed;


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if (runtime.seconds() > 240){
                telemetry.addData("Arm:", "dead");
                break;
            }

            /**
             * ---   __________________   ---
             * ---   Gamepad 1 STUFF!!!   ---
             * ---    \/ \/ \/ \/ \/      ---
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
            //  ---  SPEED BOOST!!!  --- //
            if (gamepad1.left_bumper) {
                driveSpeed = 1;
            } else {
                driveSpeed = 0.8;
            }
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
             * ---   __________________   ---
             * ---   Gamepad 2 STUFF!!!   ---
             * ---    \/ \/ \/ \/ \/      ---
             */

            // lifter motor code //
            if (gamepad2.x && liftArmSwitch.seconds() > .2) {
                robot.lifterStepDown();     // go next level up
                liftArmSwitch.reset();      //  Reset the time so we don't just stack it up infinitely
            } else if (gamepad2.y  && liftArmSwitch.seconds() > .2) {
                robot.lifterStepUp();       //  go next level down
                liftArmSwitch.reset();      //  Reset the time so we don't just stack it up infinitely
            } else if (gamepad2.b  && liftArmSwitch.seconds() > .2) {
                robot.lifterCurrentStep();  //  set to current pos if we have added since then
                liftArmSwitch.reset();      //  Reset the time so we don't just stack it up infinitely
            } else if (gamepad2.a  && liftArmSwitch.seconds() > .2) {
                robot.lifterMotor.setTargetPosition(robot.lifterMotor.getTargetPosition() + 4);  // Add a little extra to lifter
                liftArmSwitch.reset();      //  Reset the time so we don't just stack it up infinitely
            }
            robot.lifterMotor.setTargetPosition(limitRange(robot.lifterMotor.getTargetPosition() + Math.round(-gamepad2.right_stick_y * 25), -10, 1350));
            robot.periodicTeleOpTask();

            // gripper code //
            robot.gripperMotor.setPower(gamepad2.left_stick_x * 0.3);

            // jewel smacker //
            if (gamepad2.left_bumper) {
                robot.jewelSmackerUp();
            }

            /**
             * ---   _________   ---
             * ---   TELEMETRY   ---
             * ---   \/ \/ \/    ---
             */
            telemetry.addData("ArmIndex Pos:", robot.armIndex);
            telemetry.addData("(GP2.x&y) LiterPos", "Target: %2d Current: %2d", robot.lifterMotor.getTargetPosition(), robot.lifterMotor.getCurrentPosition());
            telemetry.addData("(GP2.a&b) gripper pos:","Target: %2d Current: %2d", robot.gripperMotor.getTargetPosition(), robot.gripperMotor.getCurrentPosition());
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
