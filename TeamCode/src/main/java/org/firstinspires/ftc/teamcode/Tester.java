/**
 shooterAuto.java

 A Linear opmode class to be an autonomous method for both Blue and Red to either just shoot
 the balls into the center vortex, shoot the balls and then park under the center vortex, or
 shoot the balls and park on the corner vortex.

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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


@Autonomous(name="All Tests", group="CatAuto")
public class Tester extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareCatBot robot = new HardwareCatBot();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime delaytimer = new ElapsedTime();
    private double timeDelay;


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
        int gripperPos;
        runtime.reset();
        robot.jewelSmackerUp();
        while (opModeIsActive()){
            jewelPos = robot.jewelSmacker.getPosition();
            if (gamepad1.dpad_up && (runtime.seconds() > 0.25)) {
                robot.jewelSmacker.setPosition(jewelPos + 0.05);
                telemetry.addData("Adding", "something");
                runtime.reset();
            } else if (gamepad1.dpad_down && (runtime.seconds() > 0.25)) {
                robot.jewelSmacker.setPosition(jewelPos - 0.05);
                telemetry.addData("Subtracting", "something");
                runtime.reset();
            }
            if (gamepad1.x && (runtime.seconds() > 0.25)){
                robot.encoderDrive(HardwareCatBot.CHILL_SPEED, 10, 3.0, HardwareCatBot.DRIVE_MODE.driveStraight);
                runtime.reset();
            }
            if (gamepad1.y && (runtime.seconds() > 0.25)){
                robot.encoderDrive(HardwareCatBot.CHILL_SPEED, 20, 3.0, HardwareCatBot.DRIVE_MODE.driveStraight);
                runtime.reset();
            }
            if (gamepad2.dpad_left) {
                telemetry.addData("Turn type:", "TANK 90");
                robot.absoluteGyro(HardwareCatBot.TURN_SPEED, 90, 3, HardwareCatBot.TURN_MODE.TANK);
            } else if (gamepad2.dpad_right) {
                telemetry.addData("Turn type:", "TANK -90");
                robot.absoluteGyro(HardwareCatBot.TURN_SPEED, -90, 3, HardwareCatBot.TURN_MODE.TANK);
            } else if (gamepad2.dpad_up) {
                telemetry.addData("Turn type:", "PIVOT -90");
                robot.absoluteGyro(HardwareCatBot.TURN_SPEED, -90, 3, HardwareCatBot.TURN_MODE.PIVOT);
            } else if (gamepad2.dpad_down) {
                telemetry.addData("Turn type:", "PIVOT 90");
                robot.absoluteGyro(HardwareCatBot.TURN_SPEED, 90, 3, HardwareCatBot.TURN_MODE.PIVOT);
            }
            gripperPos = robot.gripperMotor.getTargetPosition();
            if (gamepad1.dpad_left && (runtime.seconds() > 0.25)) {
                robot.gripperMotor.setTargetPosition(gripperPos + 10);
                runtime.reset();
                robot.gripperMotor.setPower(.1);
            } else if (gamepad1.dpad_right && (runtime.seconds() > 0.25)) {
                robot.gripperMotor.setTargetPosition(gripperPos - 10);
                runtime.reset();
            robot.gripperMotor.setPower(.1);
            }
            Orientation angles   = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


            telemetry.addData("xyz:", "%.1f, %.1f, %.1f", angles.firstAngle, angles.secondAngle, angles.thirdAngle);
            telemetry.addData("lifter pos:", robot.lifterMotor.getCurrentPosition());
            telemetry.addData("gripper Position", gripperPos);
            telemetry.addData("alpha:", robot.jewelColors.alpha());
            telemetry.addData("red:", robot.jewelColors.red());
            telemetry.addData("blue:", robot.jewelColors.blue());
            telemetry.addData("green:", robot.jewelColors.green());
            telemetry.addData("Jewel Position:", jewelPos);
            telemetry.addData("Current time:", runtime.seconds());
            telemetry.update();
        }

    }
}
