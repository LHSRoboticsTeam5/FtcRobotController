package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


/**
 * This 2023-2024 OpMode shows a way to drive using encoders to park from a starting position.
 */
@Autonomous(name = "AutoRedFar", group = "")
public class AutoRedFar extends LinearOpMode {

    @Override
    public void runOpMode() {

        RobotHardware robot = new RobotHardware(this);
        robot.init();

        waitForStart();

        // Strictly-speaking, checking opModeIsActive() is not really needed here.
        if (opModeIsActive()) {

            // Set these variables to a initial distance, you will need to change this number
            int driveLeftInches = 20;
            int driveRightInches = 20;
            robot.autoDriveRobot(driveLeftInches, driveRightInches);
            //set the variables to a positive and negative value to turn the robot
            driveLeftInches =-20;
            driveRightInches = 20;
            robot.autoDriveRobot(driveLeftInches, driveRightInches);
            // Set these variables both to positive to drive strait again
            driveLeftInches = 20;
            driveRightInches = 20;
            robot.autoDriveRobot(driveLeftInches, driveRightInches);
        }

        robot.shutDown();
    }

}   // end class