/**
 NaLuAutonomous.java

 A Linear OpMode class to be an autonomous method for both Blue & Red where
 we pick which stone we are on with gamepad1 and knock the jewel off and
 place a glyph or two into the cryptobox.

 NaLu is written for use with the gripper - used in our first two tournaments.

 This file is a modified version from the FTC SDK.

 Modifications by FTC Team #10273 Cat in the Hat Comes Back
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@Disabled
@Autonomous(name="NaLu Autonomous", group="CatAuto")
public class NaLuAutonomous extends LinearOpMode {

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
        robot.init(hardwareMap, this, true, true);

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
        robot.gripperStepIn();
        robot.gripperStepIn();
        robot.robotWait(1.5);
        robot.lifterStepUp();
        //robot.lifterStepUp();
        robot.robotWait(1.0);
        delaytimer.reset();

        // Look for the Column to put the block in and set appropriate mission
        mission = robot.findVuMarks(3.0);
        // Wait for JewelSmacker a wee bit
        while (delaytimer.milliseconds() < 400.0) {
            idle();
        }
        // Use JewelColors to sense the color of the jewel and turn appropriately
        if ((robot.isRed() && isRedMission) || (robot.isRed() && !isRedMission)) { // Drive forward
            // Knock it by driving forward
            robot.encoderDrive(HardwareCatBot.CHILL_SPEED, 5, 1.0, HardwareCatBot.DRIVE_MODE.driveStraight);
            robot.jewelSmackerUp();

        } else if ((!robot.isRed() && isRedMission) || (!robot.isRed() && !isRedMission)) { // Drive backward
            // Knock it by driving backwards
            robot.encoderDrive(HardwareCatBot.CHILL_SPEED, -5, 1.0, HardwareCatBot.DRIVE_MODE.driveStraight);
            robot.jewelSmackerUp();
            robot.robotWait(1.5);
            robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, 20.0, 1.0, HardwareCatBot.DRIVE_MODE.driveStraight);
        }

        // Drive off the balance and hopefully be in same place every time
        robot.encoderDrive(HardwareCatBot.CREEP_SPEED, 20, 4, HardwareCatBot.DRIVE_MODE.driveOffBalance);
        // Drop the arm
        robot.lifterStepDown();
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
                robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, 5.0, 3.0, HardwareCatBot.DRIVE_MODE.driveStraight);
                robot.absoluteGyro(HardwareCatBot.TURN_SPEED, -35, 3.0, HardwareCatBot.TURN_MODE.PIVOT);
                robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, 9, 3.0, HardwareCatBot.DRIVE_MODE.driveStraight);
                robot.absoluteGyro(HardwareCatBot.TURN_SPEED, -45, 3.0, HardwareCatBot.TURN_MODE.PIVOT);
                robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, 2.0, 2.0, HardwareCatBot.DRIVE_MODE.driveStraight);
                telemetry.update();
                break;
            case CENTER:
                telemetry.addData("Mission:", "CENTER");
                robot.lifterStepUp();
                robot.lifterStepUp();
                robot.robotWait(1.0);
                robot.absoluteGyro(HardwareCatBot.TURN_SPEED, -40, 3.0, HardwareCatBot.TURN_MODE.PIVOT);
                robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, 7, 3.0, HardwareCatBot.DRIVE_MODE.driveStraight);
                break;
            case RIGHT:
                // mysterious as THE RIGHT SIDE of the moon
                telemetry.addData("Mission:", "RIGHT");
                robot.absoluteGyro(HardwareCatBot.TURN_SPEED, -85, 5, HardwareCatBot.TURN_MODE.PIVOT);
                robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, 3, 2, HardwareCatBot.DRIVE_MODE.driveStraight);
                break;
        }
        // In THE MIDDLE of a memory
        telemetry.addData("Mission:", "CENTER");
        robot.gripperStepOut();
        robot.gripperStepOut();
        robot.gripperStepOut();
        robot.robotWait(1.5);
        robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, -5.0, 2.0, HardwareCatBot.DRIVE_MODE.driveStraight);
        robot.absoluteGyro(HardwareCatBot.TURN_SPEED, 0, 1, HardwareCatBot.TURN_MODE.PIVOT);
        robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, 2, 2, HardwareCatBot.DRIVE_MODE.driveStraight);

        robot.robotWait(5);


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
                //robot.lifterStepUp();
                //robot.robotWait(1);
                robot.absoluteGyro(HardwareCatBot.TURN_SPEED, 55, 3.0, HardwareCatBot.TURN_MODE.TANK);
                //robot.lifterStepDown();
                robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, 8.0, 3.0, HardwareCatBot.DRIVE_MODE.driveStraight);
                break;
            case CENTER:
                // In THE MIDDLE of a memory
                telemetry.addData("Mission:", "CENTER");
                telemetry.update();
                robot.robotWait(1);
                robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, 5.0, 3.0, HardwareCatBot.DRIVE_MODE.driveStraight);
                robot.absoluteGyro(HardwareCatBot.TURN_SPEED, 35, 3.0, HardwareCatBot.TURN_MODE.PIVOT);
                robot.lifterStepDown();
                robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, 8.0, 3.0, HardwareCatBot.DRIVE_MODE.driveStraight);
                break;
            case RIGHT:
                // mysterious as THE RIGHT SIDE of the moon
                telemetry.addData("Mission:", "RIGHT");
                telemetry.update();
                robot.robotWait(1);
                robot.absoluteGyro(HardwareCatBot.TURN_SPEED, 10, 4, HardwareCatBot.TURN_MODE.PIVOT);
                robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, 5.0, 3.0, HardwareCatBot.DRIVE_MODE.driveStraight);
                robot.lifterStepDown();
                robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, 5.0, 3, HardwareCatBot.DRIVE_MODE.driveStraight);
                break;
        }
        telemetry.update();
        robot.gripperStepOut();
        robot.gripperStepOut();
        robot.robotWait(2.5);
        robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, -6.0, 2.0, HardwareCatBot.DRIVE_MODE.driveStraight);
        robot.absoluteGyro(HardwareCatBot.TURN_SPEED, 90, 3.0, HardwareCatBot.TURN_MODE.PIVOT);

        robot.robotWait(5.0);


    }
    /**
     * ---  ________  ---
     * ---  NAH BLUE  ---
     * ---  \/ \/ \/  ---
     */
    private void nahBlue() throws InterruptedException {
        switch (mission) {

            case RIGHT:
                // everything you own in a box to THE LEFT
                telemetry.addData("Mission:", "RIGHT");
                telemetry.update();
                //robot.lifterStepUp();
                robot.robotWait(1);
                robot.absoluteGyro(HardwareCatBot.TURN_SPEED, -38, 3.0, HardwareCatBot.TURN_MODE.TANK);
                //robot.lifterStepDown();
                robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, 7.0, 3.0, HardwareCatBot.DRIVE_MODE.driveStraight);
                break;
            case CENTER:
                // In THE MIDDLE of a memory
                telemetry.addData("Mission:", "CENTER");
                telemetry.update();
                robot.robotWait(1);
                robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, 5.0, 3.0, HardwareCatBot.DRIVE_MODE.driveStraight);
                robot.absoluteGyro(HardwareCatBot.TURN_SPEED, -20, 3.0, HardwareCatBot.TURN_MODE.PIVOT);
                //robot.lifterStepDown();
                robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, 8.0, 3.0, HardwareCatBot.DRIVE_MODE.driveStraight);
                break;
            case LEFT:
                // mysterious as THE RIGHT SIDE of the moon
                telemetry.addData("Mission:", "LEFT");
                telemetry.update();
                robot.robotWait(1);
                robot.absoluteGyro(HardwareCatBot.TURN_SPEED, -5, 2.0, HardwareCatBot.TURN_MODE.PIVOT);
                robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, 5.0, 3.0, HardwareCatBot.DRIVE_MODE.driveStraight);
                //robot.lifterStepDown();
                robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, 8.0, 3, HardwareCatBot.DRIVE_MODE.driveStraight);
                break;
        }
        telemetry.update();
        robot.lifterStepDown();
        robot.lifterStepDown();
        robot.robotWait(0.5);
        robot.gripperStepOut();
        robot.gripperStepOut();
        robot.gripperStepOut();
        robot.robotWait(2.5);
        robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, 5.0, 2.0, HardwareCatBot.DRIVE_MODE.driveStraight);
        robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, -10.0, 2.0, HardwareCatBot.DRIVE_MODE.driveStraight);
        robot.absoluteGyro(HardwareCatBot.TURN_SPEED, -90, 3.0, HardwareCatBot.TURN_MODE.PIVOT);

        robot.robotWait(5.0);


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
                robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, 5.0, 3.0, HardwareCatBot.DRIVE_MODE.driveStraight);
                robot.absoluteGyro(HardwareCatBot.TURN_SPEED, 50, 3.0, HardwareCatBot.TURN_MODE.PIVOT);
                //robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, 9, 3.0, HardwareCatBot.DRIVE_MODE.driveStraight);
                //robot.absoluteGyro(HardwareCatBot.TURN_SPEED, 45, 3.0, HardwareCatBot.TURN_MODE.PIVOT);
                robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, 7.0, 2.0, HardwareCatBot.DRIVE_MODE.driveStraight);
                telemetry.update();
                break;
            case CENTER:
                telemetry.addData("Mission:", "CENTER");
                //robot.lifterStepUp();
                //robot.lifterStepUp();
                robot.robotWait(1.0);
                robot.absoluteGyro(HardwareCatBot.TURN_SPEED, 55, 3.0, HardwareCatBot.TURN_MODE.PIVOT);
                //robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, -5, 3.0, HardwareCatBot.DRIVE_MODE.driveStraight);
                robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, 5, 3.0, HardwareCatBot.DRIVE_MODE.driveStraight);
                break;
            case LEFT:
                telemetry.addData("Mission:", "LEFT");
                //robot.lifterStepUp();
                robot.absoluteGyro(HardwareCatBot.TURN_SPEED, 76, 5, HardwareCatBot.TURN_MODE.PIVOT);
                robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, 3, 2, HardwareCatBot.DRIVE_MODE.driveStraight);
                break;
        }
        robot.lifterStepDown();
        robot.lifterStepDown();
        robot.lifterStepDown();
        robot.robotWait(0.5);
        robot.gripperStepOut();
        robot.gripperStepOut();
        robot.gripperStepOut();
        robot.robotWait(1.5);
        robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, 3.0, 20, HardwareCatBot.DRIVE_MODE.driveStraight);
        robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, -8.0, 2.0, HardwareCatBot.DRIVE_MODE.driveStraight);
        robot.absoluteGyro(HardwareCatBot.TURN_SPEED, 10, 1, HardwareCatBot.TURN_MODE.PIVOT);
        robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, 7, 2, HardwareCatBot.DRIVE_MODE.driveStraight);

        robot.robotWait(5);
    }
}
