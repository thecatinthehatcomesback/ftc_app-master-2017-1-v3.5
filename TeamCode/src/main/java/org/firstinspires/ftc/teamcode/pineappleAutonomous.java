/**
 PineappleAutonomous.java

 A Linear OpMode class to be an autonomous method for both Blue & Red where
 we pick which stone we are on with gamepad1 and knock the jewel off and
 place a glyph or two into the cryptobox.

 Pineapple is written for use with the mecanum intake and linear slides
 used for the Detroit Worlds.

 This file is a modified version from the FTC SDK.

 Modifications by FTC Team #10273 Cat in the Hat Comes Back
 */
package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Pineapple Autonomous", group="CatAuto")
public class pineappleAutonomous extends LinearOpMode {

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
        robot.jewelFlipperLeft();
        robot.lifterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lifterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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
                delaytimer.reset();
            }

            if (gamepad1.dpad_right && delaytimer.seconds() > 0.8) {
                if (stonePos == HardwareCatBot.StonePos.Audience) {
                    stonePos = HardwareCatBot.StonePos.Nah;
                } else {
                    stonePos = HardwareCatBot.StonePos.Audience;
                }
                delaytimer.reset();
            }

            // LED code...
            if (isRedMission) {
                robot.blinky(HardwareCatBot.LED_LightUpType.RED);
                robot.allianceColor = HardwareCatBot.LED_LightUpType.RED;
            } else {
                robot.blinky(HardwareCatBot.LED_LightUpType.BLUE);
                robot.allianceColor = HardwareCatBot.LED_LightUpType.BLUE;
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
        robot.IMUinit();
        Log.d("catbot", String.format("imu angle = %d", robot.getAngle()));

        /**
         * ---  ________  ---
         * ---  Jewels!!  ---
         * ---  \/ \/ \/  ---
         */
        // Lower the JewelSmacker
        robot.jewelSmackerDown();
        delaytimer.reset();
        Log.d("catbot", "smackerDown ");
        mission = robot.findVuMarks(3.0);
        Log.d("catbot", "finishVumark ");
        robot.shutDownVuforia();
        robot.lifterStepUp();
        // Wait for JewelSmacker a wee bit
        while (delaytimer.milliseconds() < 400.0) {
            idle();
        }


        // Use JewelColors to sense the color of the jewel and turn appropriately
        if ((robot.isRed() && isRedMission) || (!robot.isRed() && !isRedMission)) { // Drive forward
            //  Knock it by moving the jewelFlipper...
            Log.d("catbot", String.format("jewelRight Red %d Blue %d Mission %s",
                    robot.jewelColors.red(), robot.jewelColors.blue(),isRedMission? "RED": "Blue"));
            robot.jewelFlipper.setPosition(robot.FLIPPER_RIGHT);
            robot.robotWait(0.5);
            robot.jewelSmackerUp();

        } else if ((!robot.isRed() && isRedMission) || (robot.isRed() && !isRedMission)) { // Drive backward
            //  Knock it by moving the jewelFlipper...
            Log.d("catbot", String.format("jewelLeft Red %d Blue %d Mission %s",
                robot.jewelColors.red(), robot.jewelColors.blue(),isRedMission? "RED": "Blue"));

            robot.jewelFlipper.setPosition(robot.FLIPPER_LEFT);
            robot.robotWait(0.5);
            robot.jewelSmackerUp();
        }
        Log.d("catbot", "smackerUp");
        Log.d("catbot", String.format("before stone imu angle = %d", robot.getAngle()));

        if (stonePos == HardwareCatBot.StonePos.Nah){

            // Drive forward a little and then off the balance and hopefully be in same place every time...
            robot.encoderDrive(HardwareCatBot.CREEP_SPEED, 5, 4, HardwareCatBot.DRIVE_MODE.driveStraight);
            robot.encoderDrive(HardwareCatBot.CREEP_SPEED, 20, 4, HardwareCatBot.DRIVE_MODE.driveOffBalance);
            // Turn to center
            robot.absoluteGyro(0.4, 0, 1.0, HardwareCatBot.TURN_MODE.PIVOT);

        }

        robot.jewelSmackerUp();
        Log.d("catbot", String.format("before mission imu angle = %d", robot.getAngle()));
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

        /**
         *  WAG THE TAIL!!
         */
        delaytimer.reset();
        robot.jewelArm.setPosition(robot.ARM_ALMOST_UP);
        while (opModeIsActive()) {
            if (delaytimer.milliseconds() > robot.TAIL_SWITCH_TIME_MS *2) {
                delaytimer.reset();
                robot.jewelFlipperLeft();
            } else if (delaytimer.milliseconds() > robot.TAIL_SWITCH_TIME_MS) {
                robot.jewelFlipperRight();
            }
        }
    }





    //  =====  METHODS ======  //


    /**
     * ---  ____________  ---
     * ---  AUDIENCE RED  ---
     * ---  \/ \/ \/ \/   ---
     */
    private void audienceRed() throws InterruptedException {
        //robot.encoderDrive(HardwareCatBot.CREEP_SPEED, 2.0, 2.0, HardwareCatBot.DRIVE_MODE.driveStraight);
        robot.absoluteGyro(HardwareCatBot.CHILL_SPEED, -60, 2.0, HardwareCatBot.TURN_MODE.PIVOT);
        // Drive forward a little and then off the balance and hopefully be in same place every time...
        robot.encoderDrive(HardwareCatBot.CREEP_SPEED, 5, 4, HardwareCatBot.DRIVE_MODE.driveStraight);
        robot.encoderDrive(HardwareCatBot.CREEP_SPEED, 20, 4, HardwareCatBot.DRIVE_MODE.driveOffBalance);
        // Turn to center
        robot.absoluteGyro(0.4, 90, 1.0, HardwareCatBot.TURN_MODE.PIVOT);


        switch (mission) {

            case LEFT:
                Log.d("catbot", "right");
                robot.jewelFlipperLeft();
                robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, 5.0, 3.0, HardwareCatBot.DRIVE_MODE.driveStraight);
                robot.absoluteGyro(HardwareCatBot.TURN_SPEED, -120, 3.0, HardwareCatBot.TURN_MODE.PIVOT);
                robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, 8.0, 2.0, HardwareCatBot.DRIVE_MODE.driveStraight);
                telemetry.update();
                break;
            case CENTER:
                Log.d("catbot", "center");
                robot.jewelFlipperCenter();
                robot.absoluteGyro(HardwareCatBot.TURN_SPEED, -130, 3.0, HardwareCatBot.TURN_MODE.PIVOT);
                robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, 6, 3.0, HardwareCatBot.DRIVE_MODE.driveStraight);
                break;
            case RIGHT:
                Log.d("catbot", "left");
                robot.jewelFlipperRight();
                robot.absoluteGyro(HardwareCatBot.TURN_SPEED, -157, 5, HardwareCatBot.TURN_MODE.PIVOT);
                robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, 5, 2, HardwareCatBot.DRIVE_MODE.driveStraight);
                break;
        }

        // Spit out the Glyph...
        robot.mecanumOut();
        robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, 5.0, 2.0, HardwareCatBot.DRIVE_MODE.driveStraight);
        robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, -7.0, 2.0, HardwareCatBot.DRIVE_MODE.driveStraight);
        robot.mecanumStop();
        robot.lifterStepDown();
        // Just finished placing the glyph in the correct column...

        // Turn to a center point to grab glyphs...
        switch (mission)  {
            case LEFT:
                robot.absoluteGyro(HardwareCatBot.TURN_SPEED, -15, 3.0, HardwareCatBot.TURN_MODE.PIVOT);
                break;
            case CENTER:
                robot.absoluteGyro(HardwareCatBot.TURN_SPEED, -20, 3.0, HardwareCatBot.TURN_MODE.PIVOT);
                break;
            case RIGHT:
                robot.absoluteGyro(HardwareCatBot.TURN_SPEED, -15, 3.0, HardwareCatBot.TURN_MODE.PIVOT);
                break;
        }

        /**
         * Now go grab some extra glyphs!!
         */

        // Get there at hyper speed!!
        robot.encoderDrive(HardwareCatBot.HYPER_SPEED, 8, 5.0, HardwareCatBot.DRIVE_MODE.driveStraight);

        // Start the intake up and grab some glyphs
        robot.mecanumIn();
        double driven;
        driven = robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, 25, 3, HardwareCatBot.DRIVE_MODE.driveStraight);
        robot.initDogeCV();
        robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, -(driven + 6), 4, HardwareCatBot.DRIVE_MODE.driveStraight);
        robot.mecanumStop();
        //// TODO: 3/3/2018 This is where we need the OpenCV stuff and color sensors and data logging!!

        /**
         * Turn around and place the glyphs center column...
         */
        robot.lifterStepUp();
        robot.lifterStepUp();
        if (robot.cryptoboxDetector.isCryptoBoxDetected()){
            int cryptoAngle = (int)robot.cryptoboxAngle(robot.cryptoboxDetector, HardwareCatBot.SOCKmission.CENTER);
            int robotangle = robot.getAngle();
            int turnAngle = 180 - cryptoAngle + 90 + robotangle;
            Log.d("catbot", String.format("crypto found angle = %d", turnAngle));
            robot.absoluteGyro(HardwareCatBot.TURN_SPEED, turnAngle, 3.0, HardwareCatBot.TURN_MODE.PIVOT);
        } else {
            Log.d("catbot", String.format("crypto NOT found angle = %d", -150));
            robot.absoluteGyro(HardwareCatBot.TURN_SPEED, -150, 3, HardwareCatBot.TURN_MODE.PIVOT);
        }

        robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, 9, 2, HardwareCatBot.DRIVE_MODE.driveStraight);

        // Spit out the Glyph...
        robot.mecanumOut();
        robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, 5.0, 2.0, HardwareCatBot.DRIVE_MODE.driveStraight);
        robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, -7.0, 2.0, HardwareCatBot.DRIVE_MODE.driveStraight);
        robot.lifterStepDown();
        robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, 7.0, 2.0, HardwareCatBot.DRIVE_MODE.driveStraight);
        robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, -7.0, 2.0, HardwareCatBot.DRIVE_MODE.driveStraight);

    }


    /**
     * ---  _______  ---
     * ---  NAH RED  ---
     * ---   \/ \/   ---
     */
    private void nahRed() throws InterruptedException {

        robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, 7.0, 2.0, HardwareCatBot.DRIVE_MODE.driveStraight);

        switch (mission) {
            case LEFT:
                // everything you own in a box to THE LEFT
                Log.d("catbot", "right");
                robot.jewelFlipperLeft();
                robot.absoluteGyro(HardwareCatBot.TURN_SPEED, -83, 3.0, HardwareCatBot.TURN_MODE.PIVOT);
                robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, 25, 3.0, HardwareCatBot.DRIVE_MODE.driveStraight);
                break;
            case CENTER:
                // In THE MIDDLE of a memory
                Log.d("catbot", "center");
                robot.jewelFlipperCenter();
                robot.absoluteGyro(HardwareCatBot.TURN_SPEED, -88, 3.0, HardwareCatBot.TURN_MODE.PIVOT);
                robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, 28, 3.0, HardwareCatBot.DRIVE_MODE.driveStraight);
                break;
            case RIGHT:
                // mysterious as THE RIGHT SIDE of the moon
                Log.d("catbot", "left");
                robot.jewelFlipperRight();
                robot.absoluteGyro(HardwareCatBot.TURN_SPEED, -97, 3.0, HardwareCatBot.TURN_MODE.PIVOT);
                robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, 31, 3.0, HardwareCatBot.DRIVE_MODE.driveStraight);
                break;
        }

        // Spit out the Glyph...
        robot.mecanumOut();
        robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, 5.0, 2.5, HardwareCatBot.DRIVE_MODE.driveStraight);
        robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, -7.0, 2.5, HardwareCatBot.DRIVE_MODE.driveStraight);
        robot.mecanumStop();
        robot.lifterStepDown();
        // Just finished placing the glyph in the correct column...

        // Turn to a center point to grab glyphs...
        switch (mission)  {
            case RIGHT:
                robot.absoluteGyro(HardwareCatBot.TURN_SPEED, 30, 3.0, HardwareCatBot.TURN_MODE.PIVOT);
                break;
            case CENTER:
                robot.absoluteGyro(HardwareCatBot.TURN_SPEED, 30, 3.0, HardwareCatBot.TURN_MODE.PIVOT);
                break;
            case LEFT:
                robot.absoluteGyro(HardwareCatBot.TURN_SPEED, 30, 3.0, HardwareCatBot.TURN_MODE.PIVOT);
                break;
        }

        /**
         * Now go grab some extra glyphs!!
         */

        // Get there at hyper speed!!
        robot.encoderDrive(HardwareCatBot.HYPER_SPEED, 28, 5.0, HardwareCatBot.DRIVE_MODE.driveStraight);
        // Start the intake up and grab some glyphs
        robot.mecanumIn();
        double driven;
        driven = robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, 25, 3, HardwareCatBot.DRIVE_MODE.driveStraight);
        robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, -driven, 4, HardwareCatBot.DRIVE_MODE.driveStraight);
        robot.mecanumStop();
        //// TODO: 3/3/2018 This is where we need the OpenCV stuff and color sensors and data logging!!

        /**
         * Turn around and place the glyphs center column...
         */
        robot.lifterStepUp();
        robot.lifterStepUp();
        robot.absoluteGyro(HardwareCatBot.TURN_SPEED, -96, 3, HardwareCatBot.TURN_MODE.PIVOT);
        robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, 35, 3, HardwareCatBot.DRIVE_MODE.driveStraight);

        // Spit out the Glyph...
        robot.mecanumOut();
        robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, 5.0, 2.0, HardwareCatBot.DRIVE_MODE.driveStraight);
        robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, -7.0, 2.0, HardwareCatBot.DRIVE_MODE.driveStraight);
        robot.mecanumStop();
        robot.lifterStepDown();
        robot.encoderDrive(HardwareCatBot.DRIVE_SPEED,7.0, 2.0, HardwareCatBot.DRIVE_MODE.driveStraight);
        robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, -8.0,2.0, HardwareCatBot.DRIVE_MODE.driveStraight);

    }


    /**
     * ---  ________  ---
     * ---  NAH BLUE  ---
     * ---  \/ \/ \/  ---
     */
    private void nahBlue() throws InterruptedException {

        robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, 7.0, 2.0, HardwareCatBot.DRIVE_MODE.driveStraight);

        switch (mission) {

            case RIGHT:
                // everything you own in a box to THE LEFT
                Log.d("catbot", "right");
                robot.jewelFlipperRight();
                robot.absoluteGyro(HardwareCatBot.TURN_SPEED, 83, 3.0, HardwareCatBot.TURN_MODE.PIVOT);
                robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, 25, 3.0, HardwareCatBot.DRIVE_MODE.driveStraight);
                break;
            case CENTER:
                // In THE MIDDLE of a memory
                Log.d("catbot", "center");
                robot.jewelFlipperCenter();
                robot.absoluteGyro(HardwareCatBot.TURN_SPEED, 100, 3.0, HardwareCatBot.TURN_MODE.PIVOT);
                Log.d("catbot", String.format("after turn imu angle = %d", robot.getAngle()));
                robot.robotWait(1);
                Log.d("catbot", String.format("after wait imu angle = %d", robot.getAngle()));
                robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, 28, 3.0, HardwareCatBot.DRIVE_MODE.driveStraight);
                break;
            case LEFT:
                // mysterious as THE RIGHT SIDE of the moon
                Log.d("catbot", "left");
                robot.jewelFlipperLeft();
                robot.absoluteGyro(HardwareCatBot.TURN_SPEED, 112, 3.0, HardwareCatBot.TURN_MODE.PIVOT);
                robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, 31, 3.0, HardwareCatBot.DRIVE_MODE.driveStraight);
                break;
        }
        telemetry.update();
        Log.d("catbot", String.format("after drive imu angle = %d", robot.getAngle()));

        // Spit out the Glyph...
        robot.mecanumOut();
        robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, 5.0, 2.0, HardwareCatBot.DRIVE_MODE.driveStraight);
        robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, -7.0, 2.0, HardwareCatBot.DRIVE_MODE.driveStraight);
        robot.mecanumStop();
        robot.lifterStepDown();
        // Just finished placing the glyph in the correct column...

        // Turn to a center point to grab glyphs...
        switch (mission)  {
            case RIGHT:
                robot.absoluteGyro(HardwareCatBot.TURN_SPEED, -55, 3.0, HardwareCatBot.TURN_MODE.PIVOT);
                break;
            case CENTER:
                robot.absoluteGyro(HardwareCatBot.TURN_SPEED, -50, 3.0, HardwareCatBot.TURN_MODE.PIVOT);
                break;
            case LEFT:
                robot.absoluteGyro(HardwareCatBot.TURN_SPEED, -45, 3.0, HardwareCatBot.TURN_MODE.PIVOT);
                break;
        }

        /**
         * Now go grab some extra glyphs!!
         */

        // Get there at hyper speed!!
        robot.encoderDrive(HardwareCatBot.HYPER_SPEED, 28, 5.0, HardwareCatBot.DRIVE_MODE.driveStraight);
        // Start the intake up and grab some glyphs
        robot.mecanumIn();
        double driven;
        driven = robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, 25, 3, HardwareCatBot.DRIVE_MODE.driveStraight);
        robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, -driven, 4, HardwareCatBot.DRIVE_MODE.driveStraight);
        robot.mecanumStop();
        //// TODO: 3/3/2018 This is where we need the OpenCV stuff and color sensors and data logging!!

        /**
         * Turn around and place the glyphs center column...
         */
        robot.lifterStepUp();
        robot.lifterStepUp();
        robot.absoluteGyro(HardwareCatBot.TURN_SPEED, 105, 3, HardwareCatBot.TURN_MODE.PIVOT);
        robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, 25, 3, HardwareCatBot.DRIVE_MODE.driveStraight);

        // Spit out the Glyph...
        robot.mecanumOut();
        robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, 5.0, 2.0, HardwareCatBot.DRIVE_MODE.driveStraight);
        robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, -7.0, 2.0, HardwareCatBot.DRIVE_MODE.driveStraight);
        robot.mecanumStop();
        robot.lifterStepDown();
        robot.encoderDrive(HardwareCatBot.DRIVE_SPEED,7.0, 2.0, HardwareCatBot.DRIVE_MODE.driveStraight);
        robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, -5.0,2.0, HardwareCatBot.DRIVE_MODE.driveStraight);
    }


    /**
     * ---  _____________  ---
     * ---  AUDIENCE BLUE  ---
     * ---   \/ \/ \/ \/   ---
     */
    private void audienceBlue() throws InterruptedException {
        //robot.encoderDrive(HardwareCatBot.CREEP_SPEED, 2.0, 2.0, HardwareCatBot.DRIVE_MODE.driveStraight);
        robot.absoluteGyro(HardwareCatBot.CHILL_SPEED, 85, 2.0, HardwareCatBot.TURN_MODE.PIVOT);
        // Drive forward a little and then off the balance and hopefully be in same place every time...
        robot.encoderDrive(HardwareCatBot.CREEP_SPEED, 5, 4, HardwareCatBot.DRIVE_MODE.driveStraight);
        robot.encoderDrive(HardwareCatBot.CREEP_SPEED, 20, 4, HardwareCatBot.DRIVE_MODE.driveOffBalance);
        // Turn to center
        //robot.absoluteGyro(0.4, 90, 1.0, HardwareCatBot.TURN_MODE.PIVOT);


        switch (mission) {

            case RIGHT:
                Log.d("catbot", "right");
                robot.jewelFlipperRight();
                robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, 5.0, 3.0, HardwareCatBot.DRIVE_MODE.driveStraight);
                robot.absoluteGyro(HardwareCatBot.TURN_SPEED, 128, 3.0, HardwareCatBot.TURN_MODE.PIVOT);
                robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, 7.0, 2.0, HardwareCatBot.DRIVE_MODE.driveStraight);
                telemetry.update();
                break;
            case CENTER:
                Log.d("catbot", "center");
                robot.jewelFlipperCenter();
                robot.absoluteGyro(HardwareCatBot.TURN_SPEED, 137, 3.0, HardwareCatBot.TURN_MODE.PIVOT);
                robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, 5, 3.0, HardwareCatBot.DRIVE_MODE.driveStraight);
                break;
            case LEFT:
                Log.d("catbot", "left");
                robot.jewelFlipperLeft();
                robot.absoluteGyro(HardwareCatBot.TURN_SPEED, 166, 5, HardwareCatBot.TURN_MODE.PIVOT);
                robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, 3, 2, HardwareCatBot.DRIVE_MODE.driveStraight);
                break;
        }

        // Spit out the Glyph...
        robot.mecanumOut();
        robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, 5.0, 2.0, HardwareCatBot.DRIVE_MODE.driveStraight);
        robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, -7.0, 2.0, HardwareCatBot.DRIVE_MODE.driveStraight);
        robot.mecanumStop();
        robot.lifterStepDown();
        // Just finished placing the glyph in the correct column...

        // Turn to a center point to grab glyphs...
        switch (mission)  {
            case RIGHT:
                robot.absoluteGyro(HardwareCatBot.TURN_SPEED, 10, 3.0, HardwareCatBot.TURN_MODE.PIVOT);
                break;
            case CENTER:
                robot.absoluteGyro(HardwareCatBot.TURN_SPEED, 8, 3.0, HardwareCatBot.TURN_MODE.PIVOT);
                break;
            case LEFT:
                robot.absoluteGyro(HardwareCatBot.TURN_SPEED, 20, 3.0, HardwareCatBot.TURN_MODE.PIVOT);
                break;
        }

        /**
         * Now go grab some extra glyphs!!
         */

        // Get there at hyper speed!!
        robot.encoderDrive(HardwareCatBot.HYPER_SPEED, 8, 5.0, HardwareCatBot.DRIVE_MODE.driveStraight);

        // Start the intake up and grab some glyphs
        robot.mecanumIn();
        double driven;
        driven = robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, 25, 3, HardwareCatBot.DRIVE_MODE.driveStraight);
        robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, -driven, 4, HardwareCatBot.DRIVE_MODE.driveStraight);
        robot.mecanumStop();
        //// TODO: 3/3/2018 This is where we need the OpenCV stuff and color sensors and data logging!!

        /**
         * Turn around and place the glyphs center column...
         */
        robot.lifterStepUp();
        robot.lifterStepUp();

        switch (mission) {
            case RIGHT:
                robot.absoluteGyro(HardwareCatBot.TURN_SPEED, 150, 3, HardwareCatBot.TURN_MODE.PIVOT);
                break;
            case CENTER:
                robot.absoluteGyro(HardwareCatBot.TURN_SPEED, 155, 3, HardwareCatBot.TURN_MODE.PIVOT);
                break;
            case LEFT:
                robot.absoluteGyro(HardwareCatBot.TURN_SPEED, 150, 3, HardwareCatBot.TURN_MODE.PIVOT);
                break;
        }
        robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, 15, 3, HardwareCatBot.DRIVE_MODE.driveStraight);

        // Spit out the Glyph...
        robot.mecanumOut();
        robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, 5.0, 2.0, HardwareCatBot.DRIVE_MODE.driveStraight);
        robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, -7.0, 2.0, HardwareCatBot.DRIVE_MODE.driveStraight);
        robot.lifterStepDown();
        robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, 7.0, 2.0, HardwareCatBot.DRIVE_MODE.driveStraight);
        robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, -5.0, 2.0, HardwareCatBot.DRIVE_MODE.driveStraight);

    }
}
