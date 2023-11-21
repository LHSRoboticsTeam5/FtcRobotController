package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "TeleOpExample", group = "TeleOp")
public class TeleOpExample extends LinearOpMode {

    private DcMotor motorFrontLeft;
    private DcMotor motorFrontRight;
    private DcMotor motorBackLeft;
    private DcMotor motorBackRight;

    @Override
    public void runOpMode() {
        motorFrontLeft = hardwareMap.get(DcMotor.class, "front_left_motor");
        motorFrontRight = hardwareMap.get(DcMotor.class, "front_right_motor");
        motorBackLeft = hardwareMap.get(DcMotor.class, "back_left_motor");
        motorBackRight = hardwareMap.get(DcMotor.class, "back_right_motor");

        // Reverse the motors if needed
       motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        //motorBackRight.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            // Get joystick input values
            double leftPower = gamepad1.left_stick_y;
            double rightPower = gamepad1.right_stick_y;

            // Clip the values to stay within the range [-1, 1]
            leftPower = Range.clip(leftPower, -1, 1);
            rightPower = Range.clip(rightPower, -1, 1);

            // Set power to the motors
            motorFrontLeft.setPower(leftPower);
            motorFrontRight.setPower(rightPower);
            motorBackLeft.setPower(leftPower);
            motorBackRight.setPower(rightPower);

            telemetry.addData("Motor Power", "Left: %.2f, Right: %.2f", leftPower, rightPower);
            telemetry.update();
        }
    }
}
