/**
 Tester.java

 A Linear opmode class to test all kinds of stuff.
      Color Sensor
      Arm position
      Jewel Smaker position
      Turning using IMU

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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


@Autonomous(name="Tester", group="CatAuto")
public class Tester extends LinearOpMode {

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
        robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 250);

        /*\
        * Runs after hit start
        * DO STUFF FOR MODE!!!!!!!!!!!
        *
        \*/
        double jewelPos;
        runtime.reset();
        robot.jewelSmackerUp();
        while (opModeIsActive()){
            jewelPos = robot.jewelArm.getPosition();

            Orientation angles   = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


            telemetry.addData("xyz:", "%.1f, %.1f, %.1f", angles.firstAngle, angles.secondAngle, angles.thirdAngle);
            telemetry.addData("alpha:", robot.TopGlyphCensor.alpha());
            telemetry.addData("red:", robot.TopGlyphCensor.red());
            telemetry.addData("blue:", robot.TopGlyphCensor.blue());
            telemetry.addData("green:", robot.TopGlyphCensor.green());
            telemetry.addData("dist:", robot.TopGlyphDist.getDistance(DistanceUnit.CM));
            telemetry.addData("GlyphFinder:", robot.findGlyphColor(robot.TopGlyphCensor, robot.TopGlyphDist));
            telemetry.addData("Jewel Position:", jewelPos);
            telemetry.addData("Current time:", runtime.seconds());

            telemetry.update();

        }

    }
}
