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

        while (!isStarted()) {
            if (gamepad1.dpad_up) {
                fg.redFilter.offset1 = new Vector2d(fg.redFilter.offset1.getX(),fg.redFilter.offset1.getY() + 0.001);
            }
            if (gamepad1.dpad_down) {
                fg.redFilter.offset1 = new Vector2d(fg.redFilter.offset1.getX(),fg.redFilter.offset1.getY() - 0.001);
            }

            if (gamepad1.dpad_left) {
                fg.redFilter.offset1 = new Vector2d(fg.redFilter.offset1.getX() - 0.001,fg.redFilter.offset1.getY());
            }

            if (gamepad1.dpad_right) {
                fg.redFilter.offset1 = new Vector2d(fg.redFilter.offset1.getX() + 0.001,fg.redFilter.offset1.getY());
            }

            telemetry.addData("position", fg.redFilter.position);
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
