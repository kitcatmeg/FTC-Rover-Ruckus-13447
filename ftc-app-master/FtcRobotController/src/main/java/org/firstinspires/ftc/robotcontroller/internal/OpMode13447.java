package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class OpMode13447 extends OpMode {
    BoschIMU imu;
    DcMotor backLeftWheel;
    DcMotor backRightWheel;
    DcMotor axle;
    DcMotor frontRightWheel;
    DcMotor frontLeftWheel;
    DcMotor lift;
    Servo arm;
    Servo claim;

    @Override
    public void init() {
        imu = new BoschIMU(hardwareMap.get(BNO055IMU.class, "imu"));
        imu.init();
        backLeftWheel = hardwareMap.dcMotor.get("Back Left Wheel");
        backRightWheel = hardwareMap.dcMotor.get("Back Right Wheel");
        axle = hardwareMap.dcMotor.get("Axle");
        frontRightWheel = hardwareMap.dcMotor.get("Front Right Wheel");
        frontLeftWheel = hardwareMap.dcMotor.get("Front Left Wheel");
        lift = hardwareMap.dcMotor.get("Lift");
        arm = hardwareMap.servo.get("Arm");
        claim = hardwareMap.servo.get("Claim");

        frontRightWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightWheel.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void init_loop() {
        backLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        axle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        int div = 1;

        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_stick_x;
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;
        frontLeftWheel.setPower(v2/div);
        frontRightWheel.setPower(v1/div);
        backLeftWheel.setPower(v4/div);
        backRightWheel.setPower(v3/div);         // mecanum drive

        if (gamepad1.left_trigger > .5) {
            axle.setPower(1);
        } else if (gamepad1.right_trigger > .5) {
            axle.setPower(-1);
        } else if (gamepad1.left_bumper) {
            axle.setPower(0);
        } else if (gamepad1.right_bumper) {
            axle.setPower(0);
        }

        if (gamepad1.dpad_up) {
            arm.setPosition(0);
        } else if (gamepad1.dpad_down) {
            arm.setPosition(1);
        }

        if (gamepad1.y) {
            lift.setPower(-1);
        } else if (gamepad1.a) {
            lift.setPower(1);
        } else {
            lift.setPower(0);
        }

        if (gamepad1.b) {
            div = div == 1 ? 2 : 1;
            sleep(300);
        }

        if (gamepad1.dpad_right) {
            claim.setPosition(0.85);
        } else if (gamepad1.dpad_left) {
            claim.setPosition(0.3);
        }

        telemetry.addData("Back Right Wheel Encoder", backRightWheel.getCurrentPosition());
        telemetry.addData("Back Left Wheel Encoder", backLeftWheel.getCurrentPosition());
        telemetry.addData("Front Right Wheel Encoder", frontRightWheel.getCurrentPosition());
        telemetry.addData("Front Left Wheel Encoder", frontLeftWheel.getCurrentPosition());
    }

    public void sleep(int millis) {
        try {
            Thread.sleep(millis);
        } catch(Exception e) {
            System.out.println(e.getStackTrace());
        }
    }

}