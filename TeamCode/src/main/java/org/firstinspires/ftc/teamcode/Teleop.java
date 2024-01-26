package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Manually control robot", group = "")
public class Teleop  extends LinearOpMode {

    @Override
    public void runOpMode() {
        RobotHardware robot = new RobotHardware(this);
        robot.init();

        // Wait for the DS start button to be touched.
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            robot.manuallyDriveRobot(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

            if (gamepad1.x) {
                robot.releaseDrone();
            }
        }
    }
}