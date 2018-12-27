package org.firstinspires.ftc.robotcontroller.internal;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class SensorAuton13447 extends OpMode {
    DcMotor backLeftWheel;
    DcMotor backRightWheel;
    DcMotor axle;
    DcMotor frontRightWheel;
    DcMotor frontLeftWheel;
    DcMotor lift;
    ColorSensor colorSensor;
    Servo arm;
    Servo claim;
    private GoldAlignDetector detector;

    int step = 0;
    int gold = 0;
    int step1 = 0;
    int step2 = 0;
    int step3 = 0;

    @Override
    public void init(){
        backLeftWheel = hardwareMap.dcMotor.get("Back Left Wheel");
        backRightWheel = hardwareMap.dcMotor.get("Back Right Wheel");
        axle = hardwareMap.dcMotor.get("Axle");
        frontRightWheel = hardwareMap.dcMotor.get("Front Right Wheel");
        frontLeftWheel = hardwareMap.dcMotor.get("Front Left Wheel");
        lift = hardwareMap.dcMotor.get("Lift");
        colorSensor = hardwareMap.colorSensor.get("Color Sensor");
        arm = hardwareMap.servo.get("Arm");
        claim = hardwareMap.servo.get("Claim");

        telemetry.addData("Status", "DogeCV 2018.0 - Gold Align Example");

        // Set up detector
        detector = new GoldAlignDetector(); // Create detector
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance()); // Initialize it with the app context and camera
        detector.useDefaults(); // Set detector to use default settings

        // Optional tuning
        detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005; //

        detector.ratioScorer.weight = 5; //
        detector.ratioScorer.perfectRatio = 1.0; // Ratio adjustment

        detector.enable(); // Start the detector!

        frontRightWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightWheel.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void init_loop(){
        backLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        if (step == 0) {
            climbDown();
            sleep(1820);
            step++;
        } else if (step == 1) {
            stopMotors();
            sleep(200);
            step++;
        } else if (step == 2) {
            strafeLeft();
            sensorDown();
            sleep(385);
            step++;
        } else if (step == 3) {
            stopMotors();
            sleep(200);
            step++;
        } else if (step == 4) {
            moveBackward();
            climbUp();
            sleep(840);
            step++;
        } else if (step == 5) {
            stopMotors();
            sleep(225);
            step++;
            step++;
        } else if (step == 7) {
            strafeLeft();
            sleep(475);
            step++;
        } else if (step == 8) {
            stopMotors();
            sleep(1000);
            step++;
            if (goldFound() == true) {
                gold++;
            }
        } else if (step == 9 && gold == 0) {
            strafeRight();
            sleep(970);
            step++;
        } else if (step == 10 && gold == 0) {
            stopMotors();
            sleep(1000);
            step++;
            if (goldFound() == true) {
                gold++;
                gold++;
            }
        } else if (step == 11 && gold == 0) {
            strafeRight();
            sleep(970);
            step++;
        } else if (step == 12 && gold == 0) {
            stopMotors();
            sleep(1000);
            step++;
            if (goldFound() == true) {
                gold++;
                gold++;
                gold++;
            }
        } else if (step > 7 && gold == 1) {
            moveBackward();
            sleep(500);
            step1++;
            /*if (step1 == 1) {
                stopMotors();
                sleep(200);
                step1++;
            } else if (step1 == 2) {
                turnLeft();
                sleep(750);
                step1++;
            } else if (step1 == 3) {
                stopMotors();
                sleep(200);
                step1++;
            } else if (step1 == 4)
                moveBackward();
                sleep(500);
                step1++;
            } else if (step1 == 13) {
                stopMotors();
                sleep(200);
                step1++;
            } else if (step1 == 14) {
                claimDown();
                sleep(3000);
                step1++;
            } else if (step1 == 15) {
                claimUp();
                sleep(3000);
                step1++;
            } else if (step1 == 16) {
                moveForward();
                sleep(750);
                step1++;
            } else if (step1 == 17) {
                stopMotors();
                sleep(200);
                step1++;
            } else if (step1 == 18) {
                sensorUp();
                sleep(3000);
                step1++;*/
            } else if (step > 7 && gold == 2) {
                moveBackward();
                sleep(500);
                /*if (step == 11) {
                    stopMotors();
                    sleep(200);
                    step++;
                } else if (step == 12) {
                    claimDown();
                    sleep(3000);
                    step++;
                } else if (step == 13) {
                    claimUp();
                    sleep(3000);
                    step++;
                } else if (step == 14) {
                    moveForward();
                    sleep(1500);
                    step++;
                }*/
            } else if (step > 7 && gold == 3) {
                moveBackward();
                sleep(500);
                /*if (step == 13) {
                    stopMotors();
                    sleep(200);
                    step++;
                } else if (step == 14) {
                    turnRight();
                    sleep(750);
                    step++;
                } else if (step == 15) {
                    stopMotors();
                    sleep(200);
                    step++;
                } else if (step == 16) {
                    moveBackward();
                    sleep(500);
                    step++;
                } else if (step == 17) {
                    stopMotors();
                    sleep(200);
                    step++;
                } else if (step == 18) {
                    claimDown();
                    sleep(3000);
                    step++;
                } else if (step == 15) {
                    claimUp();
                    sleep(3000);
                    step++;
                } else if (step == 16) {
                    moveForward();
                    sleep(750);
                    step++;
                } else if (step == 17) {
                    stopMotors();
                    sleep(200);
                    step++;
                } else if (step == 18) {
                    sensorUp();
                    sleep(3000);
                    step++;
                }*/
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
        frontLeftWheel.setPower(-0.65);
        backLeftWheel.setPower(0.65);
        frontRightWheel.setPower(0.65);
        backRightWheel.setPower(-0.65);
    }

    public void strafeLeft() {
        frontLeftWheel.setPower(0.65);
        backLeftWheel.setPower(-0.6);
        frontRightWheel.setPower(-0.65);
        backRightWheel.setPower(0.65);
    }

    public void turnRight() {
        frontRightWheel.setPower(-0.65);
        frontLeftWheel.setPower(0.65);
        backRightWheel.setPower(-0.65);
        backLeftWheel.setPower(0.65);
    }

    public void turnLeft() {
        frontRightWheel.setPower(0.65);
        frontLeftWheel.setPower(-0.65);
        backRightWheel.setPower(0.65);
        backLeftWheel.setPower(-0.65);
    }

    public void sensorDown() {
        arm.setPosition(0.96);
    }

    public void sensorUp() {
        arm.setPosition(0.1 );
    }

    public void claimUp() {
        claim.setPosition(0.85);
    }

    public void claimDown() {
        claim.setPosition(0.3);
    }

    public boolean goldFound() {
        if((colorSensor.red() > 50 && colorSensor.red() < 175) && (colorSensor.green() > 50 && colorSensor.green() < 125) && (colorSensor.blue() > 25 && colorSensor.blue() < 65)){
            return true;
        } else{
            return false;
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        // Disable the detector
        detector.disable();
    }


}
