package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "OpMode2", group = "Autonomous")
public class OpMode2 extends LinearOpMode {
    private DcMotor motorFrontLeft;
    private DcMotor motorFrontRight;
    private DcMotor motorBackLeft;
    private DcMotor motorBackRight;
    ElapsedTime runtime = new ElapsedTime();


    private static final int COUNTS_PER_REVOLUTION = 560; // Adjust for your robot
    private static final double WHEEL_DIAMETER_INCHES = 4.0; // Adjust for your robot
    private static final double DRIVE_SPEED = 0.5; // Adjust motor power
    private static final double DISTANCE_INCHES = 12; // Adjust the distance you want to move

    static final double COUNTS_PER_INCH = (COUNTS_PER_REVOLUTION) /

            (WHEEL_DIAMETER_INCHES * 3.1415);

    @Override
    public void runOpMode() {
        motorFrontLeft = hardwareMap.get(DcMotor.class, "front_left_motor");
        motorFrontRight = hardwareMap.get(DcMotor.class, "front_right_motor");
        motorBackLeft = hardwareMap.get(DcMotor.class, "back_left_motor");
        motorBackRight = hardwareMap.get(DcMotor.class, "back_right_motor");

        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotor.Direction.FORWARD);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);

        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        encoderDrive(0.5,21,21,5);
        stopM();
        encoderDrive(0.75,-20,20,5);
        stopM();
        encoderDrive(0.5,88,88,8);

    }

    public void encoderDrive(double speed,

                             double leftInches, double rightInches,

                             double timeoutS) {

        int newLeftFrontTarget;

        int newRightFrontTarget;

        int newLeftRearTarget;

        int newRightRearTarget;


        // Ensure that the opmode is still active

        if (opModeIsActive()) {


            // Determine new target position, and pass to motor controller

            newLeftFrontTarget = motorFrontLeft.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);

            newLeftRearTarget = motorBackLeft.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);

            newRightFrontTarget = motorFrontRight.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);

            newRightRearTarget = motorBackRight.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);

            motorFrontLeft.setTargetPosition(newLeftFrontTarget);

            motorBackLeft.setTargetPosition(newLeftRearTarget);

            motorFrontRight.setTargetPosition(newRightFrontTarget);

            motorBackRight.setTargetPosition(newRightRearTarget);


            // Turn On RUN_TO_POSITION

            motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.

            runtime.reset();

            motorFrontLeft.setPower(Math.abs(speed));

            motorBackLeft.setPower(Math.abs(speed));

            motorFrontRight.setPower(Math.abs(speed));

            motorBackRight.setPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.

            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits

            // its target position, the motion will stop.  This is "safer" in the event that the robot will

            // always end the motion as soon as possible.

            // However, if you require that BOTH motors have finished their moves before the robot continues

            // onto the next step, use (isBusy() || isBusy()) in the loop test.

            while (opModeIsActive() &&

                    (runtime.seconds() < timeoutS) &&

                    (motorFrontLeft.isBusy() && motorBackLeft.isBusy() && motorFrontRight.isBusy() && motorBackRight.isBusy())) {


                // Display it for the driver.

                telemetry.addData("Path1", "Running to %7d :%7d", newLeftFrontTarget, newRightFrontTarget, newLeftRearTarget, newRightRearTarget);

                telemetry.addData("Path2", "Running at %7d :%7d",


                        motorFrontLeft.getCurrentPosition(),

                        motorBackLeft.getCurrentPosition(),

                        motorFrontRight.getCurrentPosition(),

                        motorBackRight.getCurrentPosition());


            }


            // Stop all motion;

            motorFrontLeft.setPower(0);

            motorFrontRight.setPower(0);

            motorBackLeft.setPower(0);

            motorBackRight.setPower(0);


            // Turn off RUN_TO_POSITION

            motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            sleep(250);   // optional pause after each move

        }
    }
    public void stopM() {
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
        sleep(1000);
    }
}

