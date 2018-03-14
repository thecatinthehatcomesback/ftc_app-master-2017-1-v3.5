/*
    MotorPosTest.java

    A Linear opmode class to test arm motor position.

    This file is a modified version from the FTC SDK.

    Modifications by FTC Team #10273 Cat in the Hat Comes Back
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Motor Pos Test", group="CatTeleOp")

public class MotorPosTest extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime driveModeSwitch = new ElapsedTime();
    private ElapsedTime liftArmSwitch = new ElapsedTime();
    private HardwareCatBot.TeleOpDriveMode driveMode = HardwareCatBot.TeleOpDriveMode.TankDrive;


    /* Declare OpMode members. */
    HardwareCatBot robot; // use the class created for the hardware

    // constructor for class
    public MotorPosTest() {
        robot = new HardwareCatBot();
    }

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize the hardware
        robot.init(hardwareMap, this, false, false);
        telemetry.addData("Status", "Initialized...  BOOM!");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // go
        runtime.reset();
        double driveSpeed;
        boolean isShooting = false;
        ElapsedTime launchTime = new ElapsedTime();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if (runtime.seconds() > 240){
                telemetry.addData("Arm:", "dead");
                break;
            }


            // Add a little extra to lifter
            if (gamepad2.a) {
                robot.lifterMotor.setTargetPosition(robot.lifterMotor.getTargetPosition() + 25);
            }
            robot.periodicTeleOpTask();

            /**
             * ---   _________   ---
             * ---   TELEMETRY   ---
             * ---   \/ \/ \/    ---
             */
            telemetry.addData("ArmIndex Pos:", robot.armIndex);
            telemetry.addData("(GP2.x&y) LiterPos", "Target: %2d Current: %2d", robot.lifterMotor.getTargetPosition(), robot.lifterMotor.getCurrentPosition());
            telemetry.update();
            idle();
        }
    }
}
