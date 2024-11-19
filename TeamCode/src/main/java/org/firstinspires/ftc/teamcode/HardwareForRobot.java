package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import java.util.List;

/**
 * Instead of each Op Mode class redefining the robot's hardware resources within its implementation,
 * This RobotHardware class has a given robot's component resources defined and set up all in one place.
 * It also has convenience methods like driveRobot(), setArmPower(), setHandPosition(), etc.
 * that work for that robot.
 *
 * Where possible, the actual hardware objects are "abstracted" (or hidden) so the OpMode code just
 * makes calls into the class, rather than accessing the internal hardware directly.
 * This is why the objects are declared "private".
 */

public class HardwareForRobot {
    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.
    private IMU imu;
    static final double COUNTS_PER_MOTOR_REV = 560;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double ROBOT_TRACK_WIDTH_INCHES = 16;
    static final RevHubOrientationOnRobot.LogoFacingDirection IMU_LOGO_DIRECTION =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;
    static final RevHubOrientationOnRobot.UsbFacingDirection IMU_USB_DIRECTION =
            RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;


    double tolerance = 4; //degrees
    // Define all the HardwareDevices (Motors, Servos, etc.).
    // Make them private so they can't be accessed externally.
    private DcMotor leftFrontWheel;
    private DcMotor rightFrontWheel;
    private DcMotor leftRearWheel;
    private DcMotor rightRearWheel;

    private DcMotor belt;
    private DcMotor joint1;
    private DcMotor joint2;
    private CRServo spin;
    private Servo bucket;

    private WebcamName webCam;
    // Define other HardwareDevices as needed.

    private Servo   intakeServos;
    //hprivate TfodProcessor tfod;
    private VisionPortal visionPortal;

    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double DEFAULT_WHEEL_MOTOR_SPEED = .4;
    public static final double MID_SERVO       =  0.5 ;
    final static int Encoder_CPR = 1440;
    final static int WHEEL_DIAMETER =4;
    final static int DISTANCE = 24;

    final static double CIRCUMFERENCE = Math.PI + WHEEL_DIAMETER;
    final static double Rotations = DISTANCE / CIRCUMFERENCE;
    final static double COUNTS = Encoder_CPR + Rotations;

    public static final int BELT_SPEED = 1;
    public static final double HAND_SPEED      =  0.02 ;  // sets rate to move servo
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;

    // The one and only constructor requires a reference to an OpMode.
    public HardwareForRobot(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Call init() to initialize all the robot's hardware.
     */
    public void init() {
        initWheelMotors();
        initServos();
        initpullys();

        initIMU();

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
        //initTfod();
        //initVisionPortal();
        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }

    private void initIMU() {
        imu = myOpMode.hardwareMap.get(IMU.class, "imu");
        //Define hub orientation.

        ImuOrientationOnRobot imuOrientation =
                new RevHubOrientationOnRobot(IMU_LOGO_DIRECTION, IMU_USB_DIRECTION);
        //Initialize IMU instance with hub orientation.

        imu.initialize((new IMU.Parameters(imuOrientation)));
        imu.resetYaw();
    }

    public double getHeadingDegrees() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    /**
     * Call shutDown() to stop and close all the robot's hardware.
     */
    public void shutDown() {
        //tfod.shutdown();
        //visionPortal.close();
    }

    /**
     * Initialize all the wheel motors.
     * This method must be called ONCE when the OpMode is initialized.
     *
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    private void initWheelMotors()    {
        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        leftFrontWheel = myOpMode.hardwareMap.get(DcMotor.class, "LFront");
        rightFrontWheel = myOpMode.hardwareMap.get(DcMotor.class, "RFront");
        leftRearWheel = myOpMode.hardwareMap.get(DcMotor.class, "LBack");
        rightRearWheel = myOpMode.hardwareMap.get(DcMotor.class, "RBack");



        // To drive forward, most robots need the motors on one side to be reversed,
        // because the axles point in opposite directions.
        // Note: The settings here assume direct drive on left and right wheels.
        // Gear Reduction or 90 Deg drives may require direction flips
        leftFrontWheel.setDirection(DcMotor.Direction.REVERSE);
        leftRearWheel.setDirection(DcMotor.Direction.REVERSE);
        rightFrontWheel.setDirection(DcMotor.Direction.REVERSE);
        rightRearWheel.setDirection(DcMotor.Direction.FORWARD);

        setRunModeForAllWheels(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        leftFrontWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set wheel motors to not resist turning when motor is stopped.
        leftFrontWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFrontWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftRearWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightRearWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
    public void resetEncoders() {
        setRunModeForAllWheels(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    /**
     * Initialize all servos.
     */
    private void initServos() {
        // Define and initialize ALL installed servos.
        intakeServos = myOpMode.hardwareMap.get(Servo.class, "Wrist");
        bucket = myOpMode.hardwareMap.get(Servo.class, "Bucket");
        spin = myOpMode.hardwareMap.get(CRServo.class, "Intake");
    }

    private void initpullys() {
        belt = myOpMode.hardwareMap.get(DcMotor.class, "Slides");
        joint1 = myOpMode.hardwareMap.get(DcMotor.class, "Arm");


    }






    /**
     * Initialize the TensorFlow Object Detection processor.
     */

    /**
     * Initialize the VisionPortal. tfod is expected to be initialized before calling this method.
     */
    ;


/*
    public void manuallyDriveRobot() {
        double r = Math.hypot(myOpMode.gamepad1.left_stick_x, myOpMode.gamepad1.left_stick_y);
        double robotAngle = Math.atan2(myOpMode.gamepad1.left_stick_y, myOpMode.gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = myOpMode.gamepad1.right_stick_x * .5;
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        rightFrontWheel.setPower(v1);
        leftFrontWheel.setPower(v2);
        rightRearWheel.setPower(v3);
        leftRearWheel.setPower(v4);

        if(myOpMode.gamepad1.y)
            Drone.setPosition(.7);
    }
*/
    /**
     * Mecanum drive
     */
    public void manuallyDriveRobot(double lStickX, double lStickY, double rStickX) {
        double vectorLength = Math.hypot(lStickX, lStickY);
        double robotAngle = Math.atan2(lStickY, lStickX) - Math.PI / 4;
        double rightXscale = rStickX * .5;
        final double rightFrontVelocity = vectorLength * Math.cos(robotAngle) + rightXscale;
        final double leftFrontVelocity = vectorLength * Math.sin(robotAngle) - rightXscale;
        final double rightRearVelocity = vectorLength * Math.sin(robotAngle) + rightXscale;
        final double leftRearVelocity = vectorLength * Math.cos(robotAngle) - rightXscale;
        // Use existing function to drive both wheels.
        setDrivePower(leftFrontVelocity, rightFrontVelocity, leftRearVelocity, rightRearVelocity);
    }
    /**
     * Pass the requested wheel motor powers to the appropriate hardware drive motors.
     */
    public void setDrivePower(double leftFrontPower, double rightFrontPower, double leftRearPower, double rightRearPower) {
        // Output the values to the motor drives.
        leftFrontWheel.setPower(leftFrontPower);
        rightFrontWheel.setPower(rightFrontPower);
        leftRearWheel.setPower(leftRearPower);
        rightRearWheel.setPower(rightRearPower);
    }
    public void autoDriveRobot(int leftInches, int rightInches, double speed) {
        int leftInchesToCPI = (int) (leftInches * COUNTS_PER_INCH);
        int rightInchesToCPI = (int) (rightInches * COUNTS_PER_INCH);

        int leftFrontTarget = leftFrontWheel.getCurrentPosition() + leftInchesToCPI;
        int leftRearTarget = rightFrontWheel.getCurrentPosition() + leftInchesToCPI;
        int rightFrontTarget = rightRearWheel.getCurrentPosition() + rightInchesToCPI;
        int rightRearTarget = leftRearWheel.getCurrentPosition() + rightInchesToCPI;

        leftFrontWheel.setTargetPosition(leftFrontTarget);
        leftRearWheel.setTargetPosition(leftRearTarget);
        rightFrontWheel.setTargetPosition(rightFrontTarget);
        rightRearWheel.setTargetPosition(rightRearTarget);

        setRunModeForAllWheels(DcMotor.RunMode.RUN_TO_POSITION);
        //Set up telemetry
        myOpMode.telemetry.setAutoClear(false);
        myOpMode.telemetry.addData("Heading", "Current Wheel Positions");

        Telemetry.Item LFrontItem = myOpMode.telemetry.addData("LF Wheel",
                leftFrontWheel.getCurrentPosition());

        Telemetry.Item leftRearWheelItem = myOpMode.telemetry.addData("LR Wheel",
                leftRearWheel.getCurrentPosition());

        Telemetry.Item RFrontItem = myOpMode.telemetry.addData("RF Wheel",
                rightFrontWheel.getCurrentPosition());

        Telemetry.Item rightRearWheelItem = myOpMode.telemetry.addData("RR Wheel",
                rightRearWheel.getCurrentPosition());

        myOpMode.telemetry.update();

        // Power all wheels for as long as they are busy.
        setPowerAllWheels(speed);

        while (leftFrontWheel.isBusy() && leftRearWheel.isBusy() && rightFrontWheel.isBusy()
                && rightRearWheel.isBusy()) {
            LFrontItem.setValue(leftFrontWheel.getCurrentPosition());
            leftRearWheelItem.setValue(leftRearWheel.getCurrentPosition());
            RFrontItem.setValue(rightFrontWheel.getCurrentPosition());
            rightRearWheelItem.setValue(rightRearWheel.getCurrentPosition());
            myOpMode.telemetry.update();


        }





        setPowerAllWheels(0); //Whoa.
        //  myOpMode.telemetry.setAutoClear(true);
    }

    public void autoDriveRobotWithHeading(int inches, double heading, double speed) {
        int inchesToCPI = (int) (inches * WHEEL_DIAMETER_INCHES);

        int leftFrontTarget = leftFrontWheel.getCurrentPosition() + inchesToCPI;
        int leftRearTarget = leftRearWheel.getCurrentPosition() + inchesToCPI;
        int rightFrontTarget = rightFrontWheel.getCurrentPosition() + inchesToCPI;
        int rightRearTarget = rightRearWheel.getCurrentPosition() + inchesToCPI;

        leftFrontWheel.setTargetPosition(leftFrontTarget);
        leftRearWheel.setTargetPosition(leftRearTarget);
        rightFrontWheel.setTargetPosition(rightFrontTarget);
        rightRearWheel.setTargetPosition(rightRearTarget);

        setRunModeForAllWheels(DcMotor.RunMode.RUN_TO_POSITION);

        // Set the required driving speed. Use a positive amount here.
        // Start driving straight, and then enter the control loop
        setPowerAllWheels(Math.abs(speed));

        while (myOpMode.opModeIsActive() && leftFrontWheel.isBusy() && leftRearWheel.isBusy() && rightFrontWheel.isBusy() && rightRearWheel.isBusy()) {
            double headingSpeedAdjustment = getHeadingCorrection(heading, 0.03);

            // if driving in reverse, the motor correction also needs to be reversed
            if (inches < 0)
                headingSpeedAdjustment *= -1.0;

            setPowerAllWheels(speed, headingSpeedAdjustment);
        }

        setPowerAllWheels(0); //Whoa
    }
    /**
     * Adjust the left and right motor speeds by the passed
     * headingSpeedAdjustment, then set power to the left
     * and right wheels to those powers.
     *
     * For example, setPowerAllWheels(.4, -.6) sets the left motors
     * to 1 and the right motors to -.2.
     * @param speed - power of the motor, a value in the interval [-1.0, 1.0]
     * @param headingSpeedAdjustment - yaw in degrees (+/- 180)
     */public void setPowerAllWheels(double speed, double headingSpeedAdjustment) {
        double leftSpeed = speed + headingSpeedAdjustment;
        double rightSpeed = speed - headingSpeedAdjustment;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0)
        {
            leftSpeed /= max;
            rightSpeed /= max;
        }

        leftFrontWheel.setPower(leftSpeed);
        leftRearWheel.setPower(leftSpeed);
        rightFrontWheel.setPower(rightSpeed);
        rightRearWheel.setPower(rightSpeed);
    }

    /**
     *  Spin on the central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the OpMode running.
     * @param heading Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                If a relative angle is required, add/subtract from current heading.
     * @param speed Desired speed of turn. (range 0 to +1.0)
     */
    public void turnToHeading(double heading, double speed) {
        HeadingTelemetry headingTelemetry = new HeadingTelemetry("Turning");
        headingTelemetry.setTargetHeading(heading);

        setRunModeForAllWheels(DcMotor.RunMode.RUN_USING_ENCODER);

        double headingDelta = heading - getHeadingDegrees();
        double headingSpeedAdjustment;

        //keep looping while we are still active, and not on heading.
        while (myOpMode.opModeIsActive() && (Math.abs(headingDelta) > 1)) {
            // Determine required steering to keep on heading
            headingSpeedAdjustment = getHeadingCorrection(heading, .01);
            // Clip the speed to the maximum permitted value.
            headingSpeedAdjustment = Range.clip(headingSpeedAdjustment, -speed, speed);

            headingTelemetry.updateHeadingData(headingSpeedAdjustment);

            // Pivot in place by applying the turning correction
            setPowerAllWheels(0, headingSpeedAdjustment);
            headingTelemetry.updateWheelData();
            headingDelta = heading - getHeadingDegrees();
        }
        setPowerAllWheels(0); //Whoa
        headingTelemetry.reset();
    }
    /**
     * Calculate a motor speed that can be used to turn the robot to the desiredHeading.
     * Get the difference between the passed desiredHeading and the actual heading.
     * Normalize the difference to be in terms of yaw (+/- 180 degrees).
     * Multiply the difference by gainFactor to get a motor speed (which will never
     * be > 1 or < -1).
     * @param desiredHeading yaw degrees (+/- 180)
     * @param gainFactor factor to convert heading difference to a motor speed
     * @return motor speed in the range of -1 to 1.
     */
    public double getHeadingCorrection(double desiredHeading, double gainFactor) {
        double headingError = desiredHeading - this.getHeadingDegrees();

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction.
        // Limit the result to +/- 1.0
        return Range.clip(headingError * gainFactor, -1, 1);
    }



    public void parkRobot() {

    }

    /**
     * autoDriveRobot using DEFAULT_WHEEL_MOTOR_SPEED.
     * @param leftInches
     * @param rightInches
     */
    public void autoDriveRobot(int leftInches, int rightInches) {
        autoDriveRobot(leftInches, rightInches, DEFAULT_WHEEL_MOTOR_SPEED);
    }


    /**
     * raiseArm using DEFAULT_WHEEL_MOTOR_SPEED.
     * @param rotation
     */
    //public void raisebelt(int rotation) {
    // raisebelt(rotation, );
    //  }

    /**
     * Set the RunMode for all wheel motors to the passed runMode.
     * @param runMode
     */



    public void setRunModeForArm(DcMotor.RunMode runMode) {
        belt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setRunModeForAllWheels(DcMotor.RunMode runMode) {
        leftFrontWheel.setMode(runMode);
        leftRearWheel.setMode(runMode);
        rightFrontWheel.setMode(runMode);
        rightRearWheel.setMode(runMode);
    }

    /**
     * Set the Power for all wheels to the passed speed.
     * @param speed
     */

    public void setPowerArm(double speed) {
        double absoluteSpeed = Math.abs(speed);
        belt.setPower(absoluteSpeed);
    }


    public void setPowerAllWheels(double speed) {
        double absoluteSpeed = Math.abs(speed);
        leftFrontWheel.setPower(absoluteSpeed);
        leftRearWheel.setPower(absoluteSpeed);
        rightFrontWheel.setPower(absoluteSpeed);
        rightRearWheel.setPower(absoluteSpeed);
    }



    /**

     * Pass the requested arm power to the appropriate hardware drive motor
     *
     * @param power driving power (-1.0 to 1.0)
     */

    public void setArmPower(double power) {
        belt.setPower(power);
    }

    /**
     * Send the two hand-servos to opposing (mirrored) positions, based on the passed offset.
     *
     * @param offset
     */







    /**
     * Move Drone servo so that drone is released.
     */


    public void beltup() {belt.setPower(-1);}
    public void beltdown() {belt.setPower(1);}
    public void  stopBelt() {belt.setPower(-.1);}
    public void jointDown() {joint1.setPower(-1.5); }
    public void jointUp() {joint1.setPower(1.5); }
    public void stopJoint() {joint1.setPower(0); }
    public void joint2Down() {joint2.setPower(-1.5);}
    public void takeIn() {intakeServos.setPosition(.9);}
    public void takeOut() {intakeServos.setPosition(.3);}
    public void hitBar() {intakeServos.setPosition(.5);}
    public void emptyBucket() {bucket.setPosition(.45);}
    public void resetBucket() {bucket.setPosition(.15);}
    public void spinIn() {spin.setPower(1);}
    public void spinStop() {spin.setPower(0);}

    public void spinOut() {spin.setPower(-1);}
    public class HeadingTelemetry {
        private Telemetry.Item leftTargetPositionsItem;
        private Telemetry.Item rightTargetPositionsItem;
        private Telemetry.Item leftActualPositionsItem;
        private Telemetry.Item rightActualPositionsItem;
        private Telemetry.Item headingItem;
        private Telemetry.Item headingAdjustmentItem;
        private Telemetry.Item leftWheelSpeedsItem;
        private Telemetry.Item rightWheelSpeedsItem;
        private Telemetry telemetry = myOpMode.telemetry;
        private double targetHeading = 0.0;
        private double currentHeading = 0.0;

        /**
         * Construct and set up all telemetry.
         * Passing anything but "Turning" (case insensitive) causes wheel targets and positions
         * to also be sent to telemetry.
         * @param motionHeading value such as "Driving" or "Turning"
         */
        public HeadingTelemetry(String motionHeading) {
            telemetry.setAutoClear(false);
            telemetry.addData("Motion", motionHeading);

            if (!"Turning".equalsIgnoreCase(motionHeading)) {
                telemetry.addData("Wheel Targets", "");
                leftTargetPositionsItem = telemetry.addData("Left Tgt F:R", "%7d:%7d", 0, 0);
                rightTargetPositionsItem = telemetry.addData("Right Tgt F:R", "%7d:%7d", 0, 0);

                telemetry.addData("Wheel Positions", "");
                leftActualPositionsItem = telemetry.addData("Left Pos F:R", "%7d:%7d", 0, 0);
                rightActualPositionsItem = telemetry.addData("Right Pos F:R", "%7d:%7d", 0, 0);
            }

            headingItem = telemetry.addData("Heading-Target:Current", "%5.2f : %5.0f", targetHeading, currentHeading);
            headingAdjustmentItem = telemetry.addData("Heading Adj:Speed Adj", "%5.1f : %5.1f",0.0, 0.0);

            telemetry.addData("Wheel Speeds", "");
            leftWheelSpeedsItem = telemetry.addData("Left Whl Spds F:R", "%5.2f : %5.2f", 0.0, 0.0);
            rightWheelSpeedsItem = telemetry.addData("Right Whl Spds F:R","%5.2f : %5.2f", 0.0, 0.0);
        }

        /**
         * Set target wheel positions.
         * If constructed with "Turning" (case insensitive), this method does nothing.
         * @param leftFrontPosition
         * @param leftRearPosition
         * @param rightFrontPosition
         * @param rightRearPosition
         */
        public void setTargetPositions(int leftFrontPosition, int leftRearPosition, int rightFrontPosition, int rightRearPosition) {
            if (leftTargetPositionsItem == null)
                return;
            leftTargetPositionsItem.setValue("%7d:%7d", leftFrontPosition, leftRearPosition);
            rightTargetPositionsItem.setValue("%7d:%7d", rightFrontPosition, rightRearPosition);
            telemetry.update();
        }

        /**
         * Update actual wheel positions and speeds.
         * If constructed with "Turning" (case insensitive), actual position data is ignored.
         */
        public void updateWheelData() {
            if (leftActualPositionsItem != null) {
                leftActualPositionsItem.setValue("%7d:%7d", leftFrontWheel.getCurrentPosition(), leftRearWheel.getCurrentPosition());
                rightActualPositionsItem.setValue("%7d:%7d", rightFrontWheel.getCurrentPosition(), rightRearWheel.getCurrentPosition());
            }

            leftWheelSpeedsItem.setValue("%5.2f : %5.2f", leftFrontWheel.getPower(), leftRearWheel.getPower());
            rightWheelSpeedsItem.setValue("%5.2f : %5.2f", rightFrontWheel.getPower(), rightRearWheel.getPower());

            telemetry.update();
        }

        /**
         * Set the target heading telemetry
         * @param heading Target
         */
        public void setTargetHeading(double heading) {
            targetHeading = heading;
            headingItem.setValue("%5.2f : %5.0f", targetHeading, currentHeading);
            telemetry.update();
        }

        /**
         * Update all heading telemetry.
         * @param speedAdjustment Speed adjustment to show in telemetry
         */
        public void updateHeadingData(double speedAdjustment) {
            currentHeading = getHeadingDegrees();
            headingItem.setValue("%5.2f : %5.0f", targetHeading, currentHeading);
            headingAdjustmentItem.setValue("%5.1f : %5.1f", targetHeading - currentHeading, speedAdjustment);
            telemetry.update();
        }

        /**
         * Telemetry Auto Clear is set to false when this class is constructed.
         * Call this once when finished with the instance to set Auto Clear to true.
         */
        public void reset() {
            this.telemetry.setAutoClear(true);
        }
    }
}