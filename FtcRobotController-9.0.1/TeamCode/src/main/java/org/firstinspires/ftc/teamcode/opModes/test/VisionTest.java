package org.firstinspires.ftc.teamcode.opModes.test;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.opModes.AutoOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.vision.FrameGrabber;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "Vision Tet")
public class VisionTest extends AutoOpMode {
    @Override
    public void setup() {
        FrameGrabber fg = new FrameGrabber(this, this.robot);
        OpenCvWebcam camera;

        while (!isStarted()) {
            if (gamepad1.dpad_up) {
                fg.redFilter.offset = new Vector2d(fg.redFilter.offset.getX(),fg.redFilter.offset.getY() + 0.01);
            }

            if (gamepad1.dpad_down) {
                fg.redFilter.offset = new Vector2d(fg.redFilter.offset.getX(),fg.redFilter.offset.getY() - 0.01);
            }

            if (gamepad1.dpad_left) {
                fg.redFilter.offset = new Vector2d(fg.redFilter.offset.getX() - 0.01,fg.redFilter.offset.getY());
            }

            if (gamepad1.dpad_right) {
                fg.redFilter.offset = new Vector2d(fg.redFilter.offset.getX() + 0.01,fg.redFilter.offset.getY());
            }

            telemetry.addData("running", "test");
            telemetry.addData("test", fg.redFilter.offset);
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
