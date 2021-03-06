/**
 OpenCVTest.java

 A Linear opmode class to be an autonomous method for testing use
 of OpenCV using the DogeCV library.  Should be able to identify
 the cryptobox and orient our bot to drive to it.

 This file is a modified version from the FTC SDK.

 Modifications by FTC Team #10273 Cat in the Hat Comes Back
 */
package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.CryptoboxDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@Autonomous(name="OpenCV Test", group="CatAuto")
public class OpenCVTest extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareCatBot robot = new HardwareCatBot();   // Use a Pushbot's hardware
    private ElapsedTime delaytimer = new ElapsedTime();
    private CryptoboxDetector cryptoboxDetector = null;


    @Override
    public void runOpMode() throws InterruptedException {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap, this, false, false);
        robot.IMUinit();
        /*
         *  Init the OpenCV...
         */
        cryptoboxDetector = new CryptoboxDetector();
        cryptoboxDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());

        //cryptoboxDetector.rotateMat = false;

        cryptoboxDetector.enable();

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
        robot.drive(0, 0);

        /*\
        * Runs after hit start
        * DO STUFF FOR MODE!!!!!!!!!!!
        *
        \*/
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + delaytimer.toString());
            telemetry.addData("isCryptoBoxDetected", cryptoboxDetector.isCryptoBoxDetected());
            telemetry.addData("isColumnDetected ", cryptoboxDetector.isColumnDetected());

            telemetry.addData("Column Left ", cryptoboxDetector.getCryptoBoxLeftPosition());
            telemetry.addData("Column Center ", cryptoboxDetector.getCryptoBoxCenterPosition());
            telemetry.addData("Column Right ", cryptoboxDetector.getCryptoBoxRightPosition());



            telemetry.addData("Angle Left", robot.cryptoboxAngle(cryptoboxDetector, HardwareCatBot.SOCKmission.LEFT));
            telemetry.addData("Angle Center", robot.cryptoboxAngle(cryptoboxDetector, HardwareCatBot.SOCKmission.CENTER));
            telemetry.addData("Angle Right", robot.cryptoboxAngle(cryptoboxDetector, HardwareCatBot.SOCKmission.RIGHT));
            //robot.drive(0.2 + leftAngle, 0.2 - leftAngle);
            /*Log.d("catbot", String.format("speed %.2f %.2f isdetected %s %s center %d",
                    0.2 + leftAngle, 0.2 - leftAngle,
                    cryptoboxDetector.isCryptoBoxDetected() ? "true": "false",
                    cryptoboxDetector.isColumnDetected() ? "true": "false",
                    cryptoboxDetector.getCryptoBoxCenterPosition()) );*/
            telemetry.update();
        }
        cryptoboxDetector.disable();
    }
}
