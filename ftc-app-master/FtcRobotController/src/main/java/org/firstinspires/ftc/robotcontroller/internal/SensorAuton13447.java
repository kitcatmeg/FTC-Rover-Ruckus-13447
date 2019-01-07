package org.firstinspires.ftc.robotcontroller.internal;

import android.graphics.Camera;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.opencv.core.Point;
import org.opencv.core.Rect;

@Autonomous
public class SensorAuton13447 extends OpMode {

    BoschIMU imu;
    DcMotor backLeftWheel;
    DcMotor backRightWheel;
    DcMotor axle;
    DcMotor frontRightWheel;
    DcMotor frontLeftWheel;
    DcMotor lift;
    Camera phone;
    Servo arm;
    Servo claim;
    private GoldAlignDetector detector;
    private GoldMineralDetector goldMineralDetector;

    // Results of the detector
    private boolean found    = false; // Is the gold mineral found
    private Point screenPosition = new Point(); // Screen position of the mineral
    private Rect foundRect = new Rect(); // Found rect

    int step = -1;
    int gold = 0;
    int step1 = 0;
    int step2 = 0;
    int step3 = 0;

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
        axle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
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
        } else if (step == 7) {
            strafeLeft();
            sleep(475);
            stopMotors();
            sleep(2000);
            step++;
            if (isFound() == true) {
                gold++;
            }
        } else if (step == 8 && gold == 0) {
            strafeRight();
            sleep(970);
            stopMotors();
            sleep(2000);
            step++;
            if (isFound() == true) {
                gold++;
                gold++;
            }
        } else if (step == 9 && gold == 0) {
            strafeRight();
            sleep(970);
            stopMotors();
            sleep(2000);
            step++;
            if (isFound() == true) {
                gold++;
                gold++;
                gold++;
            }
        } /*else if (step > 6 && gold == 1) {
            moveBackward();
            sleep(500);
            stopMotors();
            sleep(200);
            step1++;
            if (step1 == 1) {
                turnLeft();
                sleep(750);
                stopMotors();
                sleep(200);
                step1++;
            } else if (step1 == 2) {
                moveBackward();
                sleep(500);
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
                step1++;
            }
        } else if (step > 6 && gold == 2) {
            moveBackward();
            sleep(1000);
            stopMotors();
            sleep(200);
            step2++;
            if (step2 == 1) {
                claimDown();
                sleep(3000);
                step2++;
            } else if (step2 == 2) {
                claimUp();
                sleep(3000);
                step2++;
            } else if (step2 == 3) {
                moveForward();
                sleep(1500);
                step2++;
            } else if (step2 == 4) {
                claimDown();
                sleep(3000);
                step2++;
            } else if (step2 == 5) {
                claimUp();
                sleep(3000);
                step2++;
            } else if (step2 == 6) {
                moveForward();
                sleep(350);
                stopMotors();
                sleep(200);
                step2++;
            } else {
                stopMotors();
            }
        } else if (step > 6 && gold == 3) {
            moveBackward();
            sleep(500);
            stopMotors();
            sleep(200);
            step3++;
            if (step3 == 1) {
                turnRight();
                sleep(750);
                stopMotors();
                sleep(200);
                step3++;
            } else if (step3 == 2) {
                moveBackward();
                sleep(500);
                stopMotors();
                sleep(200);
                step3++;
            } else if (step3 == 3) {
                claimDown();
                sleep(3000);
                step3++;
            } else if (step3 == 4) {
                claimUp();
                sleep(3000);
                step3++;
            } else if (step3 == 5) {
                moveForward();
                sleep(750);
                stopMotors();
                sleep(200);
                step++;
            } else if (step3 == 6) {
                sensorUp();
                sleep(3000);
                step3++;
            } else {
                stopMotors();
            }
        } */else {
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

    /**
     * Returns if a mineral is being tracked/detected
     * @return if a mineral is being tracked/detected
     */
    public boolean isFound() {
        return found;
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