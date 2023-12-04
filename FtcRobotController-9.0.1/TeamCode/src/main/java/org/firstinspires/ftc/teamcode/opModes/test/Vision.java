package org.firstinspires.ftc.teamcode.opModes.test;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opModes.AutoOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.vision.FrameGrabberBlue;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "Vision Tet")
public class Vision extends AutoOpMode {
    @Override
    public void setup() {
        FrameGrabberBlue fg = new FrameGrabberBlue(this, this.robot);
        OpenCvWebcam camera;

        while (!isStarted()) {
            if (gamepad1.dpad_up) {
                fg.blueFilter.offset = new Vector2d(fg.blueFilter.offset1.getX(),fg.blueFilter.offset.getY() + 0.01);
            }

            if (gamepad1.dpad_down) {
                fg.blueFilter.offset = new Vector2d(fg.blueFilter.offset1.getX(),fg.blueFilter.offset.getY() - 0.01);
            }

            if (gamepad1.dpad_left) {
                fg.blueFilter.offset = new Vector2d(fg.blueFilter.offset1.getX() - 0.01,fg.blueFilter.offset.getY());
            }

            if (gamepad1.dpad_right) {
                fg.blueFilter.offset = new Vector2d(fg.blueFilter.offset1.getX() + 0.01,fg.blueFilter.offset.getY());
            }

            telemetry.addData("running", "test");
            telemetry.addData("test", fg.blueFilter.offset1);
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