package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


/**
 * This 2023-2024 OpMode shows a way to drive using encoders to park from a starting position.
 */
@Autonomous(name = "Red", group = "")
public class Red extends LinearOpMode {

    private RobotHardware robot;

    @Override
    public void runOpMode() {
        robot = new RobotHardware(this);
        robot.init();

        waitForStart();

        // Strictly-speaking, checking opModeIsActive() is not really needed here.
        if (opModeIsActive()) {

            // Set these variables to a initial distance, you will need to change this number
            // This is how the robot moves
            robot.autoDriveRobot(-30, -30);//forward
            robot.autoDriveRobot(25, -25);//turn right
            robot.autoDriveRobot(10, 10);//forward
            robot.LowerArm();
            robot.moveCog();
            sleep(1000);
            robot.stopCog();
            robot.RaiseArm();
            robot.autoDriveRobot(25, -25);//turn right
            robot.autoDriveRobot(20, 20);//back
            robot.LowerArm();
            robot.reverseCog();
            sleep(1000);
            robot.stopCog();
            robot.RaiseArm();
            robot.autoDriveRobot(25, -25);//turn right
            robot.autoDriveRobot(30, 30);//forward
            robot.autoDriveRobot(25, -25);//turn right
            robot.autoDriveRobot(40, 40);//forward
            while (opModeIsActive()) {
                robot.openlaunchServo();
            }
        }
    }
}
// end class

