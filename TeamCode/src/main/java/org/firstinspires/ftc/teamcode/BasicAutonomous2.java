package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.teamcode.BasicAutonomous.RobotStates;


/**
 * Created the Dames of Dunshire.
 *
 */


@Autonomous(name="Auto Long Side", group="Dunshire Opmode")
public class BasicAutonomous2 extends OpMode {

    private double startTime;
    boolean firstTime = true;
    private int paddlePosition, targetPaddlePosition;

    HardwarePushbot robot = new HardwarePushbot();


    enum RobotStates {AUTONOMOUS_START, AUTONOMOUS_SHOOT, AUTONOMOUS_SHOOT_2, AUTONOMOUS_RELOAD, AUTONOMOUS_PUSH_BALL, AUTONOMOUS_SLIDE, AUTONOMOUS_PARK };

    RobotStates robotState;

    @Override
    public void init() {

        robot.init(hardwareMap);


        robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.shooterMotor.setMode((DcMotor.RunMode.RUN_USING_ENCODER));   // Shooter should go through 1 turn

        robot.rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        robot.leftFrontMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        robot.leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        robot.rightBackMotor.setDirection(DcMotor.Direction.REVERSE);

        robot.collectorMotor.setDirection(DcMotor.Direction.FORWARD);
        robot.shooterMotor.setDirection(DcMotor.Direction.FORWARD);


        paddlePosition = robot.shooterMotor.getCurrentPosition();  // remember the start position
        targetPaddlePosition = paddlePosition + 1440 + 1440 + 360;
        robot.shooterMotor.setTargetPosition(targetPaddlePosition);

        robot.rightFrontMotor.setPower(0.0);
        robot.leftBackMotor.setPower(0.0);
        robot.rightBackMotor.setPower(0.0);
        robot.leftFrontMotor.setPower(0.0);

        robot.shooterMotor.setPower(0.0);
        robot.collectorMotor.setPower(0.0);

//        robot.leftMotor.setTargetPosition(10 * 1440);   // turn 10 rotations
//        robot.rightMotor.setTargetPosition(10 * 1440);  // turn 10 rotations

        startTime = 0.0;   // used to decide when we started turning.

        telemetry.addData("Status 1", "Called init()");

        return;

    }

    @Override
    public void start() {

        super.start();

        telemetry.addData("Status 2", "Called start()");

        startTime = getRuntime();  // used to decide if we are running more than 10 seconds.

        robotState = RobotStates.AUTONOMOUS_START;

        return;

    }

    @Override
    public void loop() {

        double elapsedTime;

        switch (robotState) {

            case AUTONOMOUS_START:

                robot.rightFrontMotor.setPower(0.7);
                robot.leftBackMotor.setPower(0.7);
                elapsedTime = getRuntime() - startTime;
                if (elapsedTime > 1.0) {
                    robot.rightFrontMotor.setPower(0.0);
                    robot.leftBackMotor.setPower(0.0);
                    robotState = RobotStates.AUTONOMOUS_SHOOT;
                    break;
                } else {
                    break;
                }

            case AUTONOMOUS_SHOOT:

                robot.shooterMotor.setPower(1.0);
                if (robot.shooterMotor.getCurrentPosition() >= robot.shooterMotor.getTargetPosition()) {
                    robot.shooterMotor.setPower(0.0);
                    targetPaddlePosition = robot.shooterMotor.getCurrentPosition() + 1440 + 1440 + 360;
                    robot.shooterMotor.setTargetPosition(targetPaddlePosition);
                    robotState = RobotStates.AUTONOMOUS_RELOAD;
                    startTime = getRuntime();
                    break;
                } else {
                    break;
                }


            case AUTONOMOUS_RELOAD:
                robot.collectorMotor.setPower(0.8);
                elapsedTime = getRuntime() - startTime;

                if (elapsedTime > 2.75) {
                    robot.collectorMotor.setPower(0.0);
                    robotState = RobotStates.AUTONOMOUS_SHOOT_2;
                    break;

                } else {
                    break;
                }

            case AUTONOMOUS_SHOOT_2:

                robot.shooterMotor.setPower(1.0);

                if (robot.shooterMotor.getCurrentPosition() >= robot.shooterMotor.getTargetPosition()) {
                    robot.shooterMotor.setPower(0.0);
                    targetPaddlePosition = robot.shooterMotor.getCurrentPosition() + 1440 + 1440 + 360;
                    startTime = getRuntime();
                    robotState = RobotStates.AUTONOMOUS_PUSH_BALL;
                    break;

                } else {
                    break;
                }


            case AUTONOMOUS_PUSH_BALL:

                robot.rightFrontMotor.setPower(0.9);
                robot.leftBackMotor.setPower(0.9);

                elapsedTime = getRuntime() - startTime;
                if (elapsedTime > 1.2) {
                    robot.rightFrontMotor.setPower(0.0);
                    robot.leftBackMotor.setPower(0.0);
                    startTime = getRuntime();
                    robotState = RobotStates.AUTONOMOUS_SLIDE;
                    break;
                } else {
                    break;
                }

            case AUTONOMOUS_SLIDE:

                telemetry.addData("Status: ", "slide Left!");

                robot.rightBackMotor.setPower(0.5);
                robot.rightFrontMotor.setPower(-0.5);
                robot.leftBackMotor.setPower(-0.5);
                robot.leftFrontMotor.setPower(0.5);

                elapsedTime = getRuntime() - startTime;
                if (elapsedTime > 0.8) {
                    robot.rightFrontMotor.setPower(0.0);
                    robot.leftBackMotor.setPower(0.0);
                    robotState = RobotStates.AUTONOMOUS_PARK;
                    break;
                } else  {
                    break;
                }

            case AUTONOMOUS_PARK:

                robot.shooterMotor.setPower(0.0);
                robot.rightFrontMotor.setPower(0.0);
                robot.leftBackMotor.setPower(0.0);
                robot.leftFrontMotor.setPower(0.0);
                robot.rightBackMotor.setPower(0.0);

                telemetry.addData("Status: ", "Time's up! stopping!");
                requestOpModeStop();
                break;

            default:

                telemetry.addData("Status: ", "Error! Landed in default case. Robot stopping!");
                requestOpModeStop();
                break;

        }

        return;

    }

    @Override
    public void stop() {

        super.stop();
        telemetry.addData("Status 5", "Stop called!");
    }


    public void oldloop() {

        double elapsedTime;

        elapsedTime = getRuntime() - startTime;

        if (elapsedTime < 12.0) {
            return;
        }
        else {
            if (firstTime ) {
                robot.rightFrontMotor.setPower(-0.9);
                robot.leftBackMotor.setPower(-0.9);
                firstTime = false;
            }
            if(elapsedTime > 15.5) {
                robot.rightFrontMotor.setPower(0.0);
                robot.leftBackMotor.setPower(0.0);
                telemetry.addData("Status 4", "Time's up! stopping!");
                requestOpModeStop();
            }
            else {
                return;
            }
            return;
        }
    }

}