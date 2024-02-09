package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


/**
 * This 2023-2024 OpMode shows a way to drive using encoders to park from a starting position.
 */
@Autonomous(name = "AutoRedFar", group = "")
public class AutoRedFar extends LinearOpMode {

    private RobotHardware robot;
    @Override
    public void runOpMode() {

        robot = new RobotHardware(this);
        robot.init();

        waitForStart();

        // Strictly-speaking, checking opModeIsActive() is not really needed here.
        if (opModeIsActive()) {

            // Set these variables to a initial distance, you will need to change this number
            int driveLeftInches = -25;
            int driveRightInches = -25;

            robot.autoDriveRobot(driveLeftInches, driveRightInches);
            robot.driveToSpike(SpikeColor.RED, 575);
            // Set these variables both to positive to drive straight again
            int positionNumber = robot.getSpikeObjectPosition();
            turnToSpike(positionNumber);
        }
    }
    private void turnToSpike(int positionNumber){
        if (positionNumber==2){
            robot.autoDriveRobot(10,10);
        }
        else if (positionNumber == 1){
            robot.autoDriveRobot(28,-28);
            robot.autoDriveRobot(-5,-5);
            robot.autoDriveRobot(10,10);
        }
        else{
            robot.autoDriveRobot(-28,28);
            robot.autoDriveRobot(-5,-5);
            robot.autoDriveRobot(10,10);
        }
    }

}   // end class written by Luciano A. Martinez and Payson Richardson














/*package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;



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
            int driveLeftInches = 14;
            int driveRightInches = 14   ;
            robot.autoDriveRobot(driveLeftInches, driveRightInches);
            //set the variables to a positive and negative value to turn the robot
            driveLeftInches =14;
            driveRightInches =-14;
            robot.autoDriveRobot(driveLeftInches, driveRightInches);
            // Set these variables both to positive to drive strait again
            driveLeftInches = 75;
            driveRightInches = 75;
            robot.autoDriveRobot(driveLeftInches, driveRightInches);
        }
    }

}   // end class*/