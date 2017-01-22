package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * Created by fieldj1 on 12/3/16.
 */
@TeleOp(name="OmmiWheelDrive", group="Linear Opmode")
public class OmmiWheelDrive extends OpMode {

    HardwarePushbot robot       = new HardwarePushbot();

    double leftFront;
    double rightFront;
    double leftBack;
    double rightBack;
    double collector;
    double shooter;
    double sensitivity = 0.5;
//    private boolean left_bumper;
//    private boolean right_bumper;

    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        robot.leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        robot.rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        robot.leftBackMotor.setDirection(DcMotor.Direction.FORWARD);
        robot.rightBackMotor.setDirection(DcMotor.Direction.REVERSE);

        robot.collectorMotor.setDirection(DcMotor.Direction.FORWARD);
        robot.shooterMotor.setDirection(DcMotor.Direction.FORWARD);

        robot.leftFrontMotor.setPower(0.0);
        robot.rightFrontMotor.setPower(0.0);
        robot.leftBackMotor.setPower(0.0);
        robot.rightBackMotor.setPower(0.0);

        robot.collectorMotor.setPower(0.0);
        robot.shooterMotor.setPower(0.0);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");


    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start()
    {
        this.sensitivity = 0.5;

    }

    @Override
    public void loop() {


        if (Math.abs(gamepad1.left_stick_y) > 0.10) {
            rightFront = gamepad1.left_stick_y * sensitivity;
            leftBack = gamepad1.left_stick_y * sensitivity;
        }
        else {
            rightFront = 0.0;
            leftBack = 0.0;
        }


        if (Math.abs(gamepad1.right_stick_x) > 0.10) {

            rightBack = gamepad1.right_stick_x * sensitivity;
            leftFront = gamepad1.right_stick_x * sensitivity;
        }
        else {
            rightBack = 0.0;
            leftFront = 0.0;
        }

        if (gamepad1.dpad_right)
        {
            rightBack = -.5;
            rightFront = .5;
            leftFront = .5;
            leftBack = -.5;
        }

        if(gamepad1.dpad_left)
        {
            rightBack = .5;
            rightFront = -.5;
            leftFront = -.5;
            leftBack = .5;
        }

        if (gamepad1.right_bumper)
        {
            collector = 1.0;
        }
        else {
            collector = 0.0;
        }

        if (gamepad1.left_bumper)
        {
            shooter = 1.0;
        }
        else
        {
            shooter = 0.0;
        }

        robot.leftFrontMotor.setPower(leftFront);
        robot.rightFrontMotor.setPower(-rightFront);  // could reverse direction with -
        robot.leftBackMotor.setPower(-leftBack);      // could reverse direction with -
        robot.rightBackMotor.setPower(rightBack);

        robot.collectorMotor.setPower(collector); // collector

        robot.shooterMotor.setPower(shooter);  //shooter

        telemetry.addData("right front", "%.2f", rightFront);
        telemetry.addData("right back", "%.2f", rightBack);
        telemetry.addData("left front",  "%.2f", leftFront);
        telemetry.addData("left back", "%.2f", leftBack);
        telemetry.addData("collector",  "%.2f", collector);
        telemetry.addData("shooter", "%.2f", shooter);

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }

}
