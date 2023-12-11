package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.drive.GearRotationMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Timer;

public class Drivetrain extends Subsystem {
    private Telemetry telemetry;
    private Gamepad gamepad1;
    private Gamepad gamepad2;

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DigitalChannel backLeftTouch;
    private DigitalChannel backRightTouch;
    private Servo airPlane;

    private double drivePower = 0.7;
    private Timer timer;

    public GearRotationMecanumDrive roadRunnerDrive;
    private LinearOpMode opMode;

    public Drivetrain(LinearOpMode opMode, Telemetry telemetry, Pose2d initialPose) {
        this.telemetry = telemetry;
        this.gamepad1 = opMode.gamepad1;
        this.gamepad2 = opMode.gamepad2;
        timer = new Timer(opMode);

        HardwareMap map = opMode.hardwareMap;

        frontLeft = map.get(DcMotorEx.class, "BR");
        backLeft = map.get(DcMotorEx.class, "FR");
        backRight = map.get(DcMotorEx.class, "FL");
        frontRight = map.get(DcMotorEx.class, "BL");
        backLeftTouch = map.get(DigitalChannel.class, "BLT");
        backRightTouch = map.get(DigitalChannel.class, "BRT");
        airPlane = map.get(Servo.class, "Plane");

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

     //   airPlane.setPosition(0);

        roadRunnerDrive = new GearRotationMecanumDrive(map);
        roadRunnerDrive.setPoseEstimate(initialPose);
        this.opMode = opMode;

        stopAndReset();
    }

    public void mecanumDrive(double power) {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        if (gamepad1.dpad_up) {
            y = 0.3;
            x = 0;
            rx = 0;
        } else if (gamepad1.dpad_down) {
            y = -0.3;
            x = 0;
            rx = 0;
        } else if (gamepad1.dpad_right) {
            y = 0;
            x = 0.4;
            rx = 0;
        } else if (gamepad1.dpad_left) {
            y = 0;
            x = -0.4;
            rx = 0;
        }

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        frontLeft.setPower(frontLeftPower * power);
        backLeft.setPower(backLeftPower * power);
        frontRight.setPower(frontRightPower * power);
        backRight.setPower(backRightPower * power);
    }

    public void moveforward(int distance) {
        double averageEncoderTicks = 0;
        while (distance < averageEncoderTicks) {
            averageEncoderTicks = getAverageEncoder();

            setPowers(0.9);
        }
    }

    private void setPowers(double FL, double FR, double BL, double BR) {
        this.frontLeft.setPower(FL);
        this.frontRight.setPower(FR);
        this.backLeft.setPower(BL);
        this.backRight.setPower(BR);
    }

    public void displayEnValue() {
        telemetry.addData("Front Left", frontLeft.getCurrentPosition());
        telemetry.addData("Front Right", frontRight.getCurrentPosition());
        telemetry.addData("Back Left", backLeft.getCurrentPosition());
        telemetry.addData("Back Right", backRight.getCurrentPosition());

    }

    private void setPowers(double power) {
        setPowers(power, power, power, power);
    }

    private double getAverageEncoder() { //front right motor w/out encoder
        return (frontLeft.getCurrentPosition() + backRight.getCurrentPosition() + backLeft.getCurrentPosition()) / 3.0;
    }

    public void forwardTest() {
        setPowers(0.2);
    }
    public void stopAndReset() {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        timer.wait(100);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    public void moveBackwardsTouchSensor() {
        double flPower = 0;
        double frPower = 0;
        double blPower = 0;
        double brPower = 0;
        timer.resetTimer();

        while ((!backLeftTouch.getState() && !backRightTouch.getState()) && opMode.opModeIsActive()) {
            if (timer.getTime() > 2500) {
                break;
            }

            if(!backLeftTouch.getState()) {
                flPower = -0.2;
                blPower = -0.2;
            }
            if(!backRightTouch.getState()) {
                frPower = -0.2;
                brPower = -0.2;
            }
            setPowers(flPower,frPower,blPower,brPower);

        }
         setPowers(0);
    }
    @Override
    public void update() {
        if (Lift.isLiftUp) {
            drivePower = 0.25;
        } else {
            if (gamepad1.left_bumper) {
                drivePower = 0.25;
            } else {
                drivePower = 0.9;
            }
        }

        if(gamepad2.x){
            airPlane.setPosition(-1);
        } else {
            airPlane.setPosition(1);
        }


        mecanumDrive(drivePower);
        }
    }