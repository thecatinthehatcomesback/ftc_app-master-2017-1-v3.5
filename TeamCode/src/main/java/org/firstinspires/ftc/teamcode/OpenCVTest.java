/**
 DriveTester.java

 A Linear opmode class to be an autonomous method for testing if drive a
 distance works accurately.  This tests that the encoders work and the
 calibration for number of ticks per inch is accurate.

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

import android.util.Log;

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

        /*
         *  Init the OpenCV...
         */
        cryptoboxDetector = new CryptoboxDetector();
        cryptoboxDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());

        cryptoboxDetector.rotateMat = false;

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

            telemetry.update();
            //center
            double adjustment;
            if (cryptoboxDetector.isColumnDetected()){
                adjustment = (cryptoboxDetector.getCryptoBoxCenterPosition() - 400.0)/800.0;
            } else {
                adjustment = 0;
            }
            robot.drive(0.2 + adjustment, 0.2 - adjustment);
            Log.d("catbot", String.format("speed %.2f %.2f isdetected %s %s center %d",
                    0.2 + adjustment, 0.2 - adjustment,
                    cryptoboxDetector.isCryptoBoxDetected() ? "true": "false",
                    cryptoboxDetector.isColumnDetected() ? "true": "false",
                    cryptoboxDetector.getCryptoBoxCenterPosition()) );
        }
    }
}
