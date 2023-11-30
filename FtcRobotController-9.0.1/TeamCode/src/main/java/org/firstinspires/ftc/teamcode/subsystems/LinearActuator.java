package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Timer;

public class LinearActuator extends Subsystem{
    private Telemetry telemetry;
    private DcMotor linearActuator;
    private Gamepad gamepad1;
    private Gamepad gamepad2;
    private LinearOpMode opMode;
    private Timer timer;


    public LinearActuator(OpMode opMode, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.gamepad1 = opMode.gamepad1;
        this.gamepad2 = opMode.gamepad2;

        this.opMode = (LinearOpMode) opMode;

        this.timer = new Timer(this.opMode);


        HardwareMap map = opMode.hardwareMap;


        linearActuator = map.get(DcMotor.class, "LAM");
        linearActuator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearActuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//initialization servo placement
        linearActuator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void update() {
//        if(gamepad2.dpad_up){
//            linearActuator.setPower(0.6);
//        }

        if(gamepad2.left_stick_y > 0.5) {
            linearActuator.setPower(-0.4);
        } else if(gamepad2.left_stick_y < -0.5) {
            linearActuator.setPower(0.6);
        } else {
            linearActuator.setPower(0);
        }

       telemetry.addData("lam position", linearActuator.getCurrentPosition());
        telemetry.addData("lam joystick", gamepad2.left_stick_y);

    }
}
