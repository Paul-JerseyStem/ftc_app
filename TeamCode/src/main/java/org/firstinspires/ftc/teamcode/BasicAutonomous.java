package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;



/**
 * Created the Dames of Dunshire.
 *
 */


@Autonomous(name="Swanason's Basic Autonomous", group="Dunshire Opmode")
public class BasicAutonomous extends OpMode {

    private double matchStartTime;
    boolean firstTime = true;

    HardwarePushbot robot = new HardwarePushbot();

    @Override
    public void init() {

        robot.init(hardwareMap);


        robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        robot.leftFrontMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        robot.leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        robot.rightBackMotor.setDirection(DcMotor.Direction.REVERSE);

        robot.rightFrontMotor.setPower(0.0);
        robot.leftBackMotor.setPower(0.0);
        robot.rightBackMotor.setPower(0.0);
        robot.leftFrontMotor.setPower(0.0);

//        robot.leftMotor.setTargetPosition(10 * 1440);   // turn 10 rotations
//        robot.rightMotor.setTargetPosition(10 * 1440);  // turn 10 rotations

        matchStartTime = 0.0;   // used to decide when we started turning.

        telemetry.addData("Status 1", "Called init()");

        return;

    }

    @Override
    public void start() {

        super.start();

        telemetry.addData("Status 2", "Called start()");

        matchStartTime = getRuntime();  // used to decide if we are running more than 10 seconds.

        return;

    }

    @Override
    public void loop() {

        double elapsedTime;

        elapsedTime = getRuntime() - matchStartTime;

        if (elapsedTime < 12.0) {
            return;
        }
        else {
            if (firstTime ) {
                robot.rightFrontMotor.setPower(-0.9);
                robot.leftBackMotor.setPower(-0.9);
                firstTime = false;
            }
            if(elapsedTime > 14.5) {
                robot.rightFrontMotor.setPower(0.0);
                robot.leftBackMotor.setPower(0.0);
//              telemetry.addData("Left Position:", "%6d", robot.leftMotor.getCurrentPosition());
//              telemetry.addData("Right Position:", "%6d", robot.rightMotor.getCurrentPosition());
//              telemetry.addData("Elapsed Time: ",  "%3.3f", elapsedTime);
                telemetry.addData("Status 4", "Time's up! stopping!");
                requestOpModeStop();
            }
            else {
                return;
            }
            return;
        }

    }

    @Override
    public void stop() {

        super.stop();
        telemetry.addData("Status 5", "Stop called!");
    }

}