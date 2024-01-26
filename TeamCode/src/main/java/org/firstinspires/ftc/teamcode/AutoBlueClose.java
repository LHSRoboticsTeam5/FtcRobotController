package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


/**
 * This 2023-2024 OpMode shows a way to drive using encoders to park from a starting position.
 */
@Autonomous(name = "AutoBlueClose", group = "")
public class AutoBlueClose extends LinearOpMode {
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
            robot.driveToSpike(SpikeColor.BLUE,100);
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

}   // end class hi



/*package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "AutoBlueClose", group = "")
public class AutoBlueClose extends LinearOpMode {

    @Override
    public void runOpMode() {
        int propPositionNumber = 1;

        RobotHardware robot = new RobotHardware(this);
        robot.init();

        waitForStart();
            if (opModeIsActive()) {
                robot.autoDriveRobot(-20,-20 );
                robot.setRunModeForAllWheels(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.setPowerAllWheels(-0.5);
                RGBAcolors colors;
                while (opModeIsActive()) {
                   colors = robot.getSensorColors();
                    if(colors.getRed()>2000) {
                        break;
                    }
                }
            }
            robot.setPowerAllWheels(0);
            telemetry.setAutoClear(false);
            propPositionNumber = robot.getSpikeObjectPosition();
            int pixelTurnDistance = 12;
            int ninetyDegreeDistance = 17;
            double turnSpeed = .2;
            if(propPositionNumber == 2){
               robot.autoDriveRobot(5,5);
               robot.autoDriveRobot(ninetyDegreeDistance, ninetyDegreeDistance * -1, turnSpeed);
            } else if (propPositionNumber == 1){
                robot.autoDriveRobot(pixelTurnDistance, pixelTurnDistance *-1, turnSpeed);
            } else {
                robot.autoDriveRobot(pixelTurnDistance, pixelTurnDistance*-1, turnSpeed);
            }
            telemetry.addData("Op Mode", propPositionNumber);
            telemetry.update();
        }
    }
   // end class */