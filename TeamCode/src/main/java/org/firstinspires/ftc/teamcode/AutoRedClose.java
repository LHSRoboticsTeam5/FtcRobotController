package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


/**
 * This 2023-2024 OpMode shows a way to drive using encoders to park from a starting position.
 */
@Autonomous(name = "AutoRedClose", group = "")
public class AutoRedClose extends LinearOpMode {
    private RobotHardware robot;
    @Override
    public void runOpMode() {

        //RobotHardware robot = new RobotHardware(this);
        // robot.init();
        robot = new RobotHardware(this);
        robot.init();

        waitForStart();

        // Strictly-speaking, checking opModeIsActive() is not really needed here.
        if (opModeIsActive()) {

            // Set these variables to a initial distance, you will need to change this number
            int driveLeftInches = -25;
            int driveRightInches = -25;
            robot.autoDriveRobot(driveLeftInches, driveRightInches);
            robot.driveToSpike(SpikeColor.RED,100);
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


}   // end class// end class
/*package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;



@Autonomous(name = "AutoRedClose", group = "")
public class AutoRedClose extends LinearOpMode {
    private RobotHardware robot;
    @Override
    public void runOpMode() {

        //RobotHardware robot = new RobotHardware(this);
        // robot.init();
        robot = new RobotHardware(this);
        robot.init();

        waitForStart();

        // Strictly-speaking, checking opModeIsActive() is not really needed here.
        if (opModeIsActive()) {

            // Set these variables to a initial distance, you will need to change this number
            int driveLeftInches = -25;
            int driveRightInches = -25;
            robot.autoDriveRobot(driveLeftInches, driveRightInches);
            robot.driveToSpike(SpikeColor.RED,100,-.1 );
            int positionNumber = robot.getSpikeObjectPosition();
            robot.turnToSpike(positionNumber);
        }
    }


}  */ // end class




/*package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;



@Autonomous(name = "AutoRedClose", group = "")
public class AutoRedClose extends LinearOpMode {

    @Override
    public void runOpMode() {

        RobotHardware robot = new RobotHardware(this);
        robot.init();

        waitForStart();

        // Strictly-speaking, checking opModeIsActive() is not really needed here.
        if (opModeIsActive()) {

                // Set these variables to a initial distance, you will need to change this number
                int driveLeftInches = 25;
                int driveRightInches = 25;
                robot.autoDriveRobot(driveLeftInches, driveRightInches);
        }
    }

} */  // end class

