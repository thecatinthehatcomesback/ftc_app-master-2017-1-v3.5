/**
 DriveTester.java

 A Linear opmode class to be an autonomous method for testing if drive a
 distance works accurately.  This tests that the encoders work and the
 calibration for number of ticks per inch is accurate.

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
@Autonomous(name="Driver Test", group="CatAuto")
public class DriveTester extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareCatBot robot = new HardwareCatBot();   // Use a Pushbot's hardware
    private ElapsedTime delaytimer = new ElapsedTime();
    private ElapsedTime driveDistanceTimer = new ElapsedTime();
    private double distanceToDrive = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap, this, false, false);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
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
        driveDistanceTimer.reset();

        while (!opModeIsActive()) {

            //Don't need "waitForStart()" since we've been looping waiting for opmode to be enabled.

            // Do the delay asked for.
            delaytimer.reset();

            // Choose distance
            if (gamepad1.dpad_up && (driveDistanceTimer.seconds() > 0.25)) {
                distanceToDrive = distanceToDrive + 1;
                driveDistanceTimer.reset();
            } else if (gamepad1.dpad_down && (driveDistanceTimer.seconds() > 0.25)) {
                distanceToDrive = distanceToDrive - 1;
                driveDistanceTimer.reset();
            }
            telemetry.addData("Driving Distance:", distanceToDrive);
            telemetry.update();
        }
        //initilize the sensor :)
        robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 250);

        /*\
        * Runs after hit start
        * DO STUFF FOR MODE!!!!!!!!!!!
        *
        \*/
        robot.mecanumIn();
        robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, distanceToDrive, 15, HardwareCatBot.DRIVE_MODE.driveStraight);
        robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, -distanceToDrive, 15, HardwareCatBot.DRIVE_MODE.driveStraight);
        robot.mecanumStop();

        telemetry.addData("Driving Distance:", distanceToDrive);
        telemetry.update();


    }
}
