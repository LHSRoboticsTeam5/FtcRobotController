package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "BlueLeft")
public class BlueLeft extends LinearOpMode {
    @Override
    public void runOpMode() {
///
        HardwareForRobot robot = new HardwareForRobot(this);
        robot.init();
        waitForStart();
        if (opModeIsActive()) {
            robot.emptyBucket();
            robot.takeOut();
            robot.jointDown();
            sleep(630);
            robot.stopJoint();
            robot.hitBar();
            robot.autoDriveRobot(-15, -15);
            sleep(200);
            robot.beltup();
            sleep(2200);
            robot.stopBelt();
            robot.autoDriveRobot(-10, -10);
            robot.resetBucket();
            robot.resetBucket();
            sleep(600);
            robot.autoDriveRobot(12,12);
            robot.beltdown();
            sleep(1600);
            robot.stopBelt();
            robot.resetEncoders();
            robot.autoDriveRobot(-30,30);

            robot.resetEncoders();
            robot.autoDriveRobot(-80, -80);
            robot.resetEncoders();
            robot.autoDriveRobot(30,-30);
            robot.resetEncoders();
            robot.autoDriveRobot(25, 25);
            robot.resetEncoders();
            robot.hitBar();
            robot.jointDown();
            sleep(10
                    );
            robot.stopJoint();

            sleep(100000);
        }


        robot.shutDown();
    }


}