/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous (name="Claim Auton")
public class ClaimAuton13447 extends OpMode {

    BoschIMU imu;
    DcMotor backLeftWheel;
    DcMotor backRightWheel;
    DcMotor axle;
    DcMotor frontRightWheel;
    DcMotor frontLeftWheel;
    DcMotor lift;
    Servo arm;
    Servo claim;

    int step = -1;

    @Override
    public void init(){
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

    public void init_loop(){
        backLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        axle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop(){
        if (step == -1) {
            imu.setCurrentPosToZero();
            step++;
        } else if (step == 0) {
            claimUp();
            sensorUp();
            climbDown();
            sleep(1875);
            stopMotors();
            sleep(200);
            step++;
        } else if (step == 1){
            if (imu.getZAngle() > 0) {
                turnRight();
                sleep(20 * (int)imu.getZAngle());
                stopMotors();
                sleep(200);
            } else if (imu.getZAngle() < 0) {
                turnLeft();
                sleep(20 * (int)imu.getZAngle());
                stopMotors();
                sleep(200);
            } else {
                stopMotors();
                sleep(200);
            }
            step++;
        } else if (step == 2) {
            strafeLeft();
            sleep(385);
            stopMotors();
            sleep(200);
            step++;
        } else if (step == 3){
            if (imu.getZAngle() > 0) {
                turnRight();
                sleep(20 * (int)imu.getZAngle());
                stopMotors();
                sleep(200);
            } else if (imu.getZAngle() < 0) {
                turnLeft();
                sleep(20 * (int)imu.getZAngle());
                stopMotors();
                sleep(200);
            } else {
                stopMotors();
                sleep(200);
            }
            step++;
        } else if (step == 4) {
            moveBackward();
            climbUp();
            sleep(750);
            stopMotors();
            sleep(200);
            step++;
        } else if (step == 5){
            if (imu.getZAngle() > 0) {
                turnRight();
                sleep(20 * (int)imu.getZAngle());
                stopMotors();
                sleep(200);
            } else if (imu.getZAngle() < 0) {
                turnLeft();
                sleep(20 * (int)imu.getZAngle());
                stopMotors();
                sleep(200);
            } else {
                stopMotors();
                sleep(200);
            }
            step++;
        } else if (step == 6) {
            strafeRight();
            sleep(300 );
            stopMotors();
            sleep(250);
            step++;
        } else if (step == 7) {
            if (imu.getZAngle() > 0) {
                turnRight();
                sleep(20 * (int)imu.getZAngle());
                stopMotors();
                sleep(200);
            } else if (imu.getZAngle() < 0) {
                turnLeft();
                sleep(20 * (int)imu.getZAngle());
                stopMotors();
                sleep(200);
            } else {
                stopMotors();
                sleep(200);
            }
            step++;
        } else if (step == 8) {
            moveBackward();
            sleep(1000);
            stopMotors();
            sleep(200);
            step++;
        } else if (step == 9) {
            if (imu.getZAngle() > 0) {
                turnRight();
                sleep(20 * (int)imu.getZAngle());
                stopMotors();
                sleep(200);
            } else if (imu.getZAngle() < 0) {
                turnLeft();
                sleep(20 * (int)imu.getZAngle());
                stopMotors();
                sleep(200);
            } else {
                stopMotors();
                sleep(200);
            }
            step++;
        } else if (step == 10) {
            claimDown();
            sleep(3000);
            step++;
        } else if (step == 11) {
            claimUp();
            sleep(3000);
            step++;
        } else if (step == 12) {
            moveForward();
            sleep(350);
            step++;
        } else {
            stopMotors();
        }

    }

    public void sleep(int millis) {
        try {
            Thread.sleep(millis);
        } catch (Exception e) {
            System.out.println(e.getStackTrace());
        }
    }

    public void stopMotors() {
        frontRightWheel.setPower(0);
        frontLeftWheel.setPower(0);
        backRightWheel.setPower(0);
        backLeftWheel.setPower(0);
        lift.setPower(0);
    }

    public void moveBackward() {
        frontRightWheel.setPower(0.6);
        frontLeftWheel.setPower(0.6);
        backRightWheel.setPower(0.6);
        backLeftWheel.setPower(0.6);
    }

    public void moveForward() {
        frontRightWheel.setPower(-0.6);
        frontLeftWheel.setPower(-0.6);
        backRightWheel.setPower(-0.6);
        backLeftWheel.setPower(-0.6);
    }

    public void climbDown() {
        lift.setPower(-1);
    }

    public void climbUp() {
        lift.setPower(1);
    }

    public void strafeRight() {
        frontLeftWheel.setPower(-.6);
        backLeftWheel.setPower(.6);
        frontRightWheel.setPower(.6);
        backRightWheel.setPower(-.6);
    }

    public void strafeLeft() {
        frontLeftWheel.setPower(.6);
        backLeftWheel.setPower(-.6);
        frontRightWheel.setPower(-.6);
        backRightWheel.setPower(.6);
    }

    public void turnRight() {
        frontRightWheel.setPower(-0.6);
        frontLeftWheel.setPower(0.6);
        backRightWheel.setPower(-0.6);
        backLeftWheel.setPower(0.6);
    }

    public void turnLeft() {
        frontRightWheel.setPower(0.6);
        frontLeftWheel.setPower(-0.6);
        backRightWheel.setPower(0.6);
        backLeftWheel.setPower(-0.6);
    }

    public void sensorDown() {
        arm.setPosition(1);
    }

    public void sensorUp() {
        arm.setPosition(0);
    }

    public void claimUp() {
        claim.setPosition(0.85);
    }

    public void claimDown() {
        claim.setPosition(0.3);
    }


}
