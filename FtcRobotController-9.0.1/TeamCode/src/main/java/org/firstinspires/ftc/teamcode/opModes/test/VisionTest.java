package org.firstinspires.ftc.teamcode.opModes.test;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opModes.AutoOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.vision.FrameGrabberBlue2;

@Autonomous(name = "Vision Test")
public class VisionTest extends AutoOpMode {
    @Override
    public void setup() {
        FrameGrabberBlue2 fg = new FrameGrabberBlue2(this, this.robot);

        while (!isStarted()) {
            if (gamepad1.dpad_up) {
                fg.blueFilterFar.offset1 = new Vector2d(fg.blueFilterFar.offset1.getX(),fg.blueFilterFar.offset1.getY() + 0.001);
            }
            if (gamepad1.dpad_down) {
                fg.blueFilterFar.offset1 = new Vector2d(fg.blueFilterFar.offset1.getX(),fg.blueFilterFar.offset1.getY() - 0.001);
            }

            if (gamepad1.dpad_left) {
                fg.blueFilterFar.offset1 = new Vector2d(fg.blueFilterFar.offset1.getX() - 0.001,fg.blueFilterFar.offset1.getY());
            }

            if (gamepad1.dpad_right) {
                fg.blueFilterFar.offset1 = new Vector2d(fg.blueFilterFar.offset1.getX() + 0.001,fg.blueFilterFar.offset1.getY());
            }

            telemetry.addData("position", fg.blueFilterFar.position);
            telemetry.addData("running", "test");
            telemetry.addData("test", fg.blueFilterFar.offset1);
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
