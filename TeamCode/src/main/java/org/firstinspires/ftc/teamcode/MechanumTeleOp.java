/*
    FrancisTeleOp.java

    A Linear opmode class to be our main teleop method

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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
