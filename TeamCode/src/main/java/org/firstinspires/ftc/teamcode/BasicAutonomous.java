package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;



/**
 * Created the Dames of Dunshire.
 *
 */


@Autonomous(name="Swanason's Basic Autonomous", group="Dunshire Opmode")
public class BasicAutonomous extends OpMode {

    private double matchStartTime;

    HardwarePushbot robot = new HardwarePushbot();

    @Override
    public void init() {

        robot.init(hardwareMap);


//        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        robot.leftMotor.setPower(0.0);
        robot.rightMotor.setPower(0.0);

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

        robot.leftMotor.setPower(0.5);
        robot.rightMotor.setPower(0.5);

        return;

    }

    @Override
    public void loop() {

        double elapsedTime;

        elapsedTime = getRuntime() - matchStartTime;

        if(elapsedTime > 2.0) {
            robot.leftMotor.setPower(0.0);
            robot.rightMotor.setPower(0.0);
//            telemetry.addData("Left Position:", "%6d", robot.leftMotor.getCurrentPosition());
//            telemetry.addData("Right Position:", "%6d", robot.rightMotor.getCurrentPosition());
//            telemetry.addData("Elapsed Time: ",  "%3.3f", elapsedTime);
            telemetry.addData("Status 4", "Time's up! stopping!");
        }

        return;

    }

    @Override
    public void stop() {

        super.stop();
        telemetry.addData("Status 5", "Stop called!");
    }

}