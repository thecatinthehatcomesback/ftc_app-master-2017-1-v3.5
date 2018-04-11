/**
 TightenTheIdlersTest.java

 A Linear opmode class to test if the idlers are loose.

 This file is a modified version from the FTC SDK.

 Modifications by FTC Team #10273 Cat in the Hat Comes Back
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous(name="Tighten Idlers Test", group="CatAuto")
public class TightenTheIdlersTest extends LinearOpMode {

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
        robot.IMUinit();
        // IMU Sensor
        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        /*\
        * Runs after hit start
        * DO STUFF FOR MODE!!!!!!!!!!!
        *
        \*/


        /**
         * FIRST CHECK ON LEFT SIDE...
         */

        // Turn completely around...
        robot.absoluteGyro(HardwareCatBot.TURN_SPEED, 90, 5.0, HardwareCatBot.TURN_MODE.PIVOT);

        // Drive a distance and check the angle...
        robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, 10, 2, HardwareCatBot.DRIVE_MODE.driveStraight);

        // Make sure we are good on this side...
        boolean passed180 = false;
        int angle = robot.getAngle();
        if (angle < 89  &&  angle > 91) {
            telemetry.addData("90 Degree Turn:", "Passed");
            passed180 = true;
        } else {
            telemetry.addData("90 Degree Turn:", "****Failed****");
            passed180 = false;
            telemetry.addData("Angle:", String.format("IMU Angle = %d", angle));
        }
        telemetry.update();


        // Wait a bit to let the telemetry read...
        robot.robotWait(3);


        /**
         * FIRST CHECK ON RIGHT SIDE...
         */

        // Turn completely around...
        robot.absoluteGyro(HardwareCatBot.TURN_SPEED, 0, 5.0, HardwareCatBot.TURN_MODE.PIVOT);

        // Drive a distance and check the angle...
        robot.encoderDrive(HardwareCatBot.DRIVE_SPEED, 10, 2, HardwareCatBot.DRIVE_MODE.driveStraight);

        // Make sure we are good on this side...
        if (angles.firstAngle > 1  &&  angles.firstAngle < -1) {
            telemetry.addData("0 Degree Turn:", "Passed");
        } else {
            telemetry.addData("0 Degree Turn:", "****Failed****");
        }
        telemetry.addData("Angle:", String.format("IMU Angle = %d", robot.getAngle()));
        telemetry.addData("90 Degree Turn: ",passed180 ? "Passed" : "***Failed****");
        telemetry.addData("90 angle", angle);
        telemetry.update();


        // Wait a bit to let the telemetry read...
        robot.robotWait(20);
    }
}
