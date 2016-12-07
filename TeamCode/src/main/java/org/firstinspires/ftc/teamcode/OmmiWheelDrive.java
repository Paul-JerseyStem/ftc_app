package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * Created by fieldj1 on 12/3/16.
 */
@TeleOp(name="OmmiWheelDrive", group="Linear Opmode")
public abstract class OmmiWheelDrive extends OpMode {

    HardwarePushbot robot       = new HardwarePushbot();

    double leftFront;
    double rightFront;
    double leftBack;
    double rightBack;
    //double forward;
    //double backward;
    double sensitivity = 0.5;

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
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");
    }

    @Override
    public void init_loop() {

    }

    @Override
    public abstract void start();

    {
        this.sensitivity = 0.5;

    }

    @Override
    public void loop() {

        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        // set both wheels (left and right) to the same speed, based on "y" deflection.
       // forward = gamepad1.left_stick_y;
        //backward = gamepad1.left_stick_y;

        //boolean rightTurn = gamepad1.dpad_right;
        //boolean leftTurn = gamepad1.dpad_left;


        if(Math.abs(gamepad1.left_stick_y) > 0)
        {
            rightFront = gamepad1.left_stick_y*sensitivity;
            leftBack = gamepad1.left_stick_y*sensitivity;
        }

        if(Math.abs(gamepad1.right_stick_x) > 0)
        {
            rightBack = gamepad1.right_stick_x*sensitivity;
            leftFront = gamepad1.right_stick_x*sensitivity;
        }

        if (gamepad1.dpad_right)
        {
            rightBack = -.5;
            rightFront = -.5;
            leftFront = .5;
            leftBack = .5;
        }

        if(gamepad1.dpad_left)
        {
            rightBack = .5;
            rightFront = .5;
            leftFront = -.5;
            leftBack = -.5;
        }

       /* if (gamepad1.dpad_left)
        {
            leftTurn = gamepad1.dpad_left;
        }

         if (Math.abs(gamepad1.right_stick_x) > 0.10) {
            if (gamepad1.right_stick_x > 0) {
                right = gamepad1.right_stick_x*sensitivity;
                left = -gamepad1.right_stick_x*sensitivity;
            } else if (gamepad1.right_stick_x < 0){
                left = -gamepad1.right_stick_x*sensitivity;
                right = gamepad1.right_stick_x*sensitivity;
            }
        }

        if (Math.abs(gamepad1.left_stick_y) > 0.10) {
            if (gamepad1.left_stick_y > 0) {
                forward = gamepad1.left_stick_y*sensitivity;
                backward = -gamepad1.left_stick_y*sensitivity;
            } else if (gamepad1.left_stick_y < 0){
                backward = -gamepad1.left_stick_y*sensitivity;
                forward = gamepad1.left_stick_y*sensitivity;
            }
        } */


        robot.leftFrontMotor.setPower(leftFront);
        robot.rightFrontMotor.setPower(rightFront);
        robot.leftBackMotor.setPower(leftBack);
        robot.rightBackMotor.setPower(rightBack);
        telemetry.addData("right front", "%.2f", rightFront);
        telemetry.addData("right back", "%.2f", rightBack);
        telemetry.addData("left front",  "%.2f", leftFront);
        telemetry.addData("left back", "%.2f", leftBack);

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }

    }
