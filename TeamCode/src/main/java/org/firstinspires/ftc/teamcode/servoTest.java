package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


/**
 * This 2023-2024 OpMode shows a way to drive using encoders to park from a starting position.
 */
@Autonomous(name = "servoTest", group = "")
public class servoTest extends LinearOpMode {

    private RobotHardware robot;
    @Override
    public void runOpMode() {
        robot = new RobotHardware(this);
        robot.init();

        waitForStart();

        // Strictly-speaking, checking opModeIsActive() is not really needed here.
        while (opModeIsActive()) {

            robot.openlaunchServo();
            //robot.closelaunchServo();
            telemetry.addData("servo", "moving");
            telemetry.update();
        }
    }

    }

  // end class

