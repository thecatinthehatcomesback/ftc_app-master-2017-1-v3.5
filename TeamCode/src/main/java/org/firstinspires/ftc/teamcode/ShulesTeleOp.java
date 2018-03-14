/*
    ShulesTeleOp.java

    A Linear opmode class to be our main teleop method
    This version is for the mecanum intake.

    This file is a modified version from the FTC SDK.

    Modifications by FTC Team #10273 Cat in the Hat Comes Back
*/

/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

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
            if (gamepad2.a) {
                lifterPowerAdd = 0.15;
            } else {
                lifterPowerAdd = 0.00;
            }
            //// TODO: 1/27/2018 Add touch sensors to robot to detect the top and bottom of the lifter arm
            robot.lifterMotor.setPower(-gamepad2.left_stick_y + lifterPowerAdd);  // Added a little bit to keep the heavy intake up...
            robot.periodicTask();


            /* --- servo rotatey thingy --- */
            // Set the servo speed...
            intakeRotateSpeed = robot.SERVO_NEUTRAL_POWER - gamepad2.right_stick_x/2;
            // Fail safe code for when the end of the world comes...
            if (intakeRotateSpeed > 1.0) {
                intakeRotateSpeed = 1.0;
            } else if (intakeRotateSpeed < 0.0) {
                intakeRotateSpeed = 1.0;
            }

            // Actually move the servo
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

            // jewel smacker //
            if (gamepad2.dpad_up) {
                robot.jewelSmackerUp();
            }
            if (gamepad2.dpad_left) {
                robot.jewelFlipperLeft();
            }
            if (gamepad2.dpad_right)  {
                robot.jewelFlipperRight();
            }
            if (gamepad2.dpad_down) {
                robot.jewelFlipperCenter();
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
