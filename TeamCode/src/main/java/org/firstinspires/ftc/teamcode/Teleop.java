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

            if (gamepad1.a) {
                robot.LowerArm();
            }
            else if (gamepad1.b) {
                robot.RaiseArm();
            }
            else {
                robot.stopArm();
            }

            if (gamepad1.left_bumper)
            {
                robot.moveCog();
            }

            if (gamepad1.right_bumper)
            {
                robot.stopCog();
            }

            if (gamepad1.dpad_left)   {
                robot.openGrabber();
            }
            if (gamepad2.left_bumper){
                robot.lowerVerticalSlide();
            }

            if (gamepad2.right_bumper){
                robot.liftVerticalSlide();
            }

            if(gamepad2.x){
                robot.stopVerticalSlide();
            }

            if (gamepad1.dpad_right){
                robot.closeGrabber();
            }

            if (gamepad1.dpad_up){
                robot.reverseCog();
            }

            while (gamepad1.x) {
                robot.closelaunchServo();
                telemetry.addData("servo", "moving");
                telemetry.update();
            }

            while (gamepad1.y) {
                robot.openlaunchServo();
                telemetry.addData("servo", "moving");
                telemetry.update();
            }
        }
    }
}