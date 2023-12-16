package org.firstinspires.ftc.teamcode.opModes.test;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opModes.AutoOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.vision.FrameGrabberRed;

@Autonomous(name = "Vision Test")
public class VisionTest extends AutoOpMode {
    @Override
    public void setup() {
        FrameGrabberRed fg = new FrameGrabberRed(this, this.robot);

        while (!isStarted()) {
            if (gamepad1.dpad_up) {
                fg.redFilter.offset = new Vector2d(fg.redFilter.offset.getX(),fg.redFilter.offset.getY() + 0.001);
            }
            if (gamepad1.dpad_down) {
                fg.redFilter.offset = new Vector2d(fg.redFilter.offset.getX(),fg.redFilter.offset.getY() - 0.001);
            }

            if (gamepad1.dpad_left) {
                fg.redFilter.offset = new Vector2d(fg.redFilter.offset.getX() - 0.001,fg.redFilter.offset.getY());
            }

            if (gamepad1.dpad_right) {
                fg.redFilter.offset = new Vector2d(fg.redFilter.offset.getX() + 0.001,fg.redFilter.offset.getY());
            }

            telemetry.addData("position", fg.redFilter.position);
            telemetry.addData("running", "test");
            telemetry.addData("test", fg.redFilter.offset);
            telemetry.update();
        }
        while (!isStarted()) {
            if (gamepad2.dpad_up) {
                fg.redFilter.offset1 = new Vector2d(fg.redFilter.offset1.getX(),fg.redFilter.offset1.getY() + 0.001);
            }
            if (gamepad2.dpad_down) {
                fg.redFilter.offset1 = new Vector2d(fg.redFilter.offset1.getX(),fg.redFilter.offset1.getY() - 0.001);
            }

            if (gamepad2.dpad_left) {
                fg.redFilter.offset1 = new Vector2d(fg.redFilter.offset1.getX() - 0.001,fg.redFilter.offset1.getY());
            }

            if (gamepad2.dpad_right) {
                fg.redFilter.offset1 = new Vector2d(fg.redFilter.offset1.getX() + 0.001,fg.redFilter.offset1.getY());
            }

            telemetry.addData("position", fg.redFilter.position);
            telemetry.addData("running", "test");
            telemetry.addData("test", fg.redFilter.offset1);
            telemetry.update();
        }
    }

    @Override
    public void run() {

    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this, this.telemetry, getInitialPose());
        setup();
        waitForStart();
        run();
    }
}
