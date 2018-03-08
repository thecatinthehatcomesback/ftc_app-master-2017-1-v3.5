/**
 NaLuAutonomous.java

 A Linear OpMode class to be an autonomous method for both Blue & Red where
 we pick which stone we are on with gamepad1 and knock the jewel off and
 place a glyph or two into the cryptobox.

 NaLu is written for use with the gripper - used in our first two tournaments.

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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


@Autonomous(name="Potato Autonomous", group="CatAuto")
public class potatoAutonomous extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareCatBot robot = new HardwareCatBot();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime delaytimer = new ElapsedTime();
    private double timeDelay;
    private boolean isRedMission = true;
    private HardwareCatBot.SOCKmission mission = HardwareCatBot.SOCKmission.CENTER;

    private HardwareCatBot.StonePos stonePos = HardwareCatBot.StonePos.Nah;

    @Override
    public void runOpMode() throws InterruptedException {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap, this, true, false);
        robot.jewelSmackerUp();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        /** how many potato to get boyfriend?
         * 3 whole potato
         * I thought it was 2 whole potato
         * The recession hit us hard
         **/
        telemetry.update();

        robot.resetEncoders();
        idle();

        robot.runToPosition();

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d",
                robot.leftMotor.getCurrentPosition(),
                robot.rightMotor.getCurrentPosition());
        telemetry.update();

        // After init is pushed but before Start we can change the delay using dpad up/down
        delaytimer.reset();

        while (!opModeIsActive() ) {
            if (this.isStopRequested()) {
                return;
            }
            // increase if up is pressed and it's been 0.2 seconds since last push
            if (gamepad1.dpad_up && (delaytimer.seconds() > 0.8)) {
                timeDelay += 1;
                delaytimer.reset();
            }
            // decrease if down is pressed and it's been 0.2 seconds since last push
            if (gamepad1.dpad_down && (delaytimer.seconds() > 0.8)) {
                if (timeDelay > 0) {
                    timeDelay -= 1;
                }
                delaytimer.reset();
            }
            if (((gamepad1.dpad_left) && delaytimer.seconds() > 0.8)) {
                if (isRedMission) {
                    isRedMission = false;
                } else {
                    isRedMission = true;
                }
            }

            if (gamepad1.dpad_right && delaytimer.seconds() > 0.8) {
                if (stonePos == HardwareCatBot.StonePos.Audience) {
                    stonePos = HardwareCatBot.StonePos.Nah;
                } else {
                    stonePos = HardwareCatBot.StonePos.Audience;
                }
            }

            telemetry.addData("Delay Timer: ", timeDelay);

            if (isRedMission) {
                telemetry.addData("Alliance:", "Red");
            } else {
                telemetry.addData("Alliance:", "Blue");
            }
            if (stonePos == HardwareCatBot.StonePos.Audience) {
                telemetry.addData("Stone:", "Audience");
            } else {
                telemetry.addData("Stone:", "Nah");
            }
            telemetry.update();

            //Don't need "waitForStart()" since we've been looping waiting for opmode to be enabled.

        }
        /**
         * Runs after hit start
         * DO STUFF FOR MODE!!!!!!!!!!!
         *
         \*/

        //initilize the sensor :)
        robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 250);


        /**
         * ---  ________  ---
         * ---  Jewels!!  ---
         * ---  \/ \/ \/  ---
         */
        // Lower the JewelSmacker
        robot.jewelSmackerDown();

        // Look for the Column to put the block in and set appropriate mission
        mission = robot.findVuMarks(3.0);
        // Wait for JewelSmacker a wee bit
        while (delaytimer.milliseconds() < 400.0) {
            idle();
        }


        // Use JewelColors to sense the color of the jewel and turn appropriately
        if ((robot.isRed() && isRedMission) || (robot.isRed() && !isRedMission)) { // Drive forward
            //  Knock it by moving the jewelFlipper...
            robot.jewelFlipper.setPosition(robot.FLIPPER_RIGHT);
            robot.robotWait(0.5);
            robot.jewelSmackerUp();

        } else if ((!robot.isRed() && isRedMission) || (!robot.isRed() && !isRedMission)) { // Drive backward
            //  Knock it by moving the jewelFlipper...
            robot.jewelFlipper.setPosition(robot.FLIPPER_LEFT);
            robot.robotWait(0.5);
            robot.jewelSmackerUp();
        }

        // Drive forward a little and then off the balance and hopefully be in same place every time...
        robot.encoderDrive(HardwareCatBot.CREEP_SPEED, 5, 4, HardwareCatBot.DRIVE_MODE.driveStraight);
        robot.encoderDrive(HardwareCatBot.CREEP_SPEED, 20, 4, HardwareCatBot.DRIVE_MODE.driveOffBalance);
        // Turn to center
        robot.absoluteGyro(0.4, 0, 1.0, HardwareCatBot.TURN_MODE.PIVOT);

        /**
         * At this point we only have knocked the jewel off and have flattened out in front of the Stone
         */
        
        /**
         * ---  _______________________  ---
         * ---  Cryptobox and cryptos!!  ---
         * ---  \/ \/ \/ \/ \/ \/ \/ \/  ---
         */
        if (isRedMission && (stonePos == HardwareCatBot.StonePos.Nah)) {
            nahRed();
        } else if (isRedMission && (stonePos == HardwareCatBot.StonePos.Audience)) {
            audienceRed();
        } else if (!isRedMission && (stonePos == HardwareCatBot.StonePos.Nah)) {
            nahBlue();
        } else if (!isRedMission && (stonePos == HardwareCatBot.StonePos.Audience)) {
            audienceBlue();
        }

    }





    //  =====  METHODS ======  //


    /**
     * ---  ____________  ---
     * ---  AUDIENCE RED  ---
     * ---  \/ \/ \/ \/   ---
     */
    private void audienceRed() throws InterruptedException {
        switch (mission) {

            case LEFT:
                // everything you own in a box to THE LEFT
                telemetry.addData("Mission:", "LEFT");
                robot.robotWait(1);
                break;
            case CENTER:
                // In THE MIDDLE of a memory
                telemetry.addData("Mission:", "CENTER");
                robot.robotWait(1);
                break;
            case RIGHT:
                // mysterious as THE RIGHT SIDE of the moon
                telemetry.addData("Mission:", "RIGHT");
                robot.robotWait(1);
                break;
        }
        telemetry.update();

    }


    /**
     * ---  _______  ---
     * ---  NAH RED  ---
     * ---   \/ \/   ---
     */
    private void nahRed() throws InterruptedException {
        switch (mission) {

            case LEFT:
                // everything you own in a box to THE LEFT
                telemetry.addData("Mission:", "LEFT");
                telemetry.update();
                robot.robotWait(1);
                break;
            case CENTER:
                // In THE MIDDLE of a memory
                telemetry.addData("Mission:", "CENTER");
                telemetry.update();
                robot.robotWait(1);
                break;
            case RIGHT:
                // mysterious as THE RIGHT SIDE of the moon
                telemetry.addData("Mission:", "RIGHT");
                telemetry.update();
                robot.robotWait(1);
                break;
        }
        telemetry.update();

    }


    /**
     * ---  ________  ---
     * ---  NAH BLUE  ---
     * ---  \/ \/ \/  ---
     */
    private void nahBlue() throws InterruptedException {
        //// TODO: 3/3/2018 THIS WORKS NOW!!  STILL NEED TO EDIT THE ANGLES!!
        robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, 5.0, 2.0, HardwareCatBot.DRIVE_MODE.driveStraight);

        switch (mission) {

            case RIGHT:
                // everything you own in a box to THE LEFT
                telemetry.addData("Mission:", "RIGHT");
                telemetry.update();
                robot.robotWait(1);
                robot.absoluteGyro(HardwareCatBot.TURN_SPEED, 85, 3.0, HardwareCatBot.TURN_MODE.PIVOT);
                robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, 25, 3.0, HardwareCatBot.DRIVE_MODE.driveStraight);
                break;
            case CENTER:
                // In THE MIDDLE of a memory
                telemetry.addData("Mission:", "CENTER");
                telemetry.update();

                break;
            case LEFT:
                // mysterious as THE RIGHT SIDE of the moon
                telemetry.addData("Mission:", "LEFT");
                telemetry.update();

                break;
        }
        telemetry.update();

        // Spit out the Glyph...
        robot.mecanumOut();
        robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, 5.0, 2.0, HardwareCatBot.DRIVE_MODE.driveStraight);
        robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, -5.0, 2.0, HardwareCatBot.DRIVE_MODE.driveStraight);
        robot.mecanumStop();
        // Just finished placing the glyph in the correct column...

        // Turn to a center point to grab glyphs...
        switch (mission)  {
            case RIGHT:
                robot.absoluteGyro(HardwareCatBot.TURN_SPEED, -50, 3.0, HardwareCatBot.TURN_MODE.PIVOT);
                break;
            case CENTER:
                robot.absoluteGyro(HardwareCatBot.TURN_SPEED, -40, 3.0, HardwareCatBot.TURN_MODE.PIVOT);
                break;
            case LEFT:
                robot.absoluteGyro(HardwareCatBot.TURN_SPEED, -30, 3.0, HardwareCatBot.TURN_MODE.PIVOT);
                break;
        }

        /**
         * Now go grab some extra glyphs!!
         */

        // Get there at hyper speed!!
        robot.encoderDrive(HardwareCatBot.HYPER_SPEED, 30, 5.0, HardwareCatBot.DRIVE_MODE.driveStraight);
        // Start the intake up and grab some glyphs
        robot.mecanumIn();
        robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, 25, 8, HardwareCatBot.DRIVE_MODE.driveStraight);
        robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, -30, 8, HardwareCatBot.DRIVE_MODE.driveStraight);
        robot.mecanumStop();
        //// TODO: 3/3/2018 This is where we need the OpenCV stuff and color sensors and data logging!!

        /**
         * Turn around and place the glyphs center column...
         */
        robot.absoluteGyro(HardwareCatBot.TURN_SPEED, 100, 3, HardwareCatBot.TURN_MODE.PIVOT);
        robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, 35, 7, HardwareCatBot.DRIVE_MODE.driveStraight);

        // Spit out the Glyph...
        robot.mecanumOut();
        robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, 5.0, 2.0, HardwareCatBot.DRIVE_MODE.driveStraight);
        robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, -5.0, 2.0, HardwareCatBot.DRIVE_MODE.driveStraight);
        robot.mecanumStop();
    }


    /**
     * ---  _____________  ---
     * ---  AUDIENCE BLUE  ---
     * ---   \/ \/ \/ \/   ---
     */
    private void audienceBlue() throws InterruptedException {
        switch (mission) {

            case RIGHT:
                telemetry.addData("Mission:", "RIGHT");
                robot.robotWait(1);
                break;
            case CENTER:
                telemetry.addData("Mission:", "CENTER");
                robot.robotWait(1);
                break;
            case LEFT:
                telemetry.addData("Mission:", "LEFT");
                robot.robotWait(1);
                break;
        }

    }
}
