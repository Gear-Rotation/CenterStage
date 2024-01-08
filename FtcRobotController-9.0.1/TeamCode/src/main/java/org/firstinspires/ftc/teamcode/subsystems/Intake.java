package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Timer;
import org.firstinspires.ftc.teamcode.util.UTILToggle;

public class Intake extends Subsystem {

    private final LinearOpMode opMode;
    private final Timer timer;
    private DcMotor intake;
    private Telemetry telemetry;
    private Servo folder;
    private Gamepad gamepad1;
    private Gamepad gamepad2;
    private UTILToggle folderToggle = new UTILToggle();
    private boolean isFolderOpen = true;

    public Intake(OpMode opMode, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.gamepad1 = opMode.gamepad1;
        this.gamepad2 = opMode.gamepad2;

        this.opMode = (LinearOpMode) opMode;

        this.timer = new Timer(this.opMode);


        HardwareMap map = opMode.hardwareMap;

        intake = map.get(DcMotor.class, "Intake");
        folder = map.get(Servo.class, "Folder");


        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        disarmIntake();
    }

    public void armIntake(){
        folder.setPosition(0);
    }

    public void disarmIntake() {
        folder.setPosition(0.58);
    }

    public void depositPixel() {
        armIntake();
        timer.wait(500);
        timer.resetTimer();
        while(opMode.opModeIsActive() && timer.getTime() < 2000){
            intake.setPower(-0.4);
        }
        intake.setPower(0);
    }

    public void raiseIntake() {
        disarmIntake();
        timer.wait(500);
    }

    @Override
    public void update() {
        if (gamepad1.right_trigger > 0.25 || gamepad2.right_trigger > 0.25) {
            intake.setPower(0.7);
        } else if (gamepad1.left_trigger > 0.25) {
            intake.setPower(gamepad1.left_trigger * -0.5);
        } else {
            intake.setPower(0);
        }

//        if (gamepad2.left_trigger > 0.25) {
//            intake.setPower(gamepad2.left_trigger * 0.5);
//        } else {
//            intake.setPower(0);
//        }

        if(gamepad2.left_trigger > 0.25) {
            folder.setPosition(0);
        } else {
            folder.setPosition(0.58);
        }

//        if (folderToggle.status(gamepad1.right_bumper) == UTILToggle.Status.COMPLETE) {
//            if (isFolderOpen) {
//                armIntake();
//                isFolderOpen = false;
//            } else {
//                disarmIntake();
//                isFolderOpen = true;
//            }

    }
}
