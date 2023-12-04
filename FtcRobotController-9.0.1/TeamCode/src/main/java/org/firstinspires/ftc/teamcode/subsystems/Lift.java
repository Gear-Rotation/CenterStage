package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Timer;

public class Lift extends Subsystem {
    private Telemetry telemetry;
    private Gamepad gamepad1;
    private Gamepad gamepad2;
    private final int UPPER_BOUND = -2989;
    private DcMotor slide;
    private Servo block;
    private LinearOpMode opMode;
    private Timer timer;
    private DigitalChannel slideTouch;

    public static boolean isLiftUp = false;

    public enum SlideStates {
        MANUAL_CONTROL,
        RUN_POSITION,
        RESET,
        HOLDING
    }


    //    public LiftStates currentState = LiftStates.IDLE;
    public SlideStates currentSlideState = SlideStates.MANUAL_CONTROL;

    public Lift(OpMode opMode, Telemetry telemetry) {
        isLiftUp = false;
        this.telemetry = telemetry;
        this.gamepad1 = opMode.gamepad1;
        this.gamepad2 = opMode.gamepad2;

        this.opMode = (LinearOpMode) opMode;

        this.timer = new Timer(this.opMode);


        HardwareMap map = opMode.hardwareMap;

        slide = map.get(DcMotor.class, "Slide");
        block = map.get(Servo.class,   "Block");
        slideTouch = map.get(DigitalChannel.class, "ST");
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//initialization servo placement
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        block.setPosition(0.47);
    }

    // accesbility return name (paramterType parmater ...) {

    public void liftToPosition(int encoderValue) {
        slide.setTargetPosition(encoderValue);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(0.7);
    }

    public boolean hasReachedTarget(double threshold) {
        return Math.abs(slide.getTargetPosition() - slide.getCurrentPosition()) < threshold;
    }
    public int getCurrentPosition() {
        return slide.getCurrentPosition();
    }
//drops servo onto board
    public void autoDeposit() {
        block.setPosition(0.2);
    }
//closes servo after dropping
    public void autoClose(){
        block.setPosition(0.47);
    }
//teleop below
@Override
public void update() {
        //if the right joystick(vertical) is pushing up
    if (Math.abs(gamepad2.right_stick_y) > 0.05) {
        //if the slides position is less than or equal to the limit and the right joystick is push down than the slide should not move
        if(slide.getCurrentPosition() <= UPPER_BOUND && gamepad2.right_stick_y < 0) {
            slide.setPower(0);
        } else {
            //if the touch sensor is clicked and the right joystick is down set slides power to 0
            //CHANGE BELOW
            if(slideTouch.getState() && (gamepad2.right_stick_y > 0)) {
                slide.setPower(0);
            } else {
                //set the slides power to how much the joystick is being pushed from -1,1 * 0.5 power
                slide.setPower(gamepad2.right_stick_y * 0.75);
            }
        }
    } else {
        //if nothing is being done then set slides power to 0
        slide.setPower(0);
    }
    if (gamepad2.right_bumper){
        block.setPosition(0.2);
    } else {
        block.setPosition(0.47);
    }

//        if (slideTouch.getState()) {
//            slide.setPower(0.6);
//        } else {
//            slide.setPower(0);
//        }

//        telemetry.addData("slide encoder pos: ", slide.getCurrentPosition());



        telemetry.addData("Gamepad 2 right stick y", gamepad2.right_stick_y);
        telemetry.addData("Current Slide State", slide.getPower());
        telemetry.addData("Encoder Position", slide.getCurrentPosition());
        telemetry.addData("slide touch",slideTouch.getState());

    }
}

// -1839
// -2903
// -