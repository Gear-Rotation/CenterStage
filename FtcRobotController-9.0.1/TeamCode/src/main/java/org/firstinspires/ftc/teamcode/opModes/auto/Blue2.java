package org.firstinspires.ftc.teamcode.opModes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opModes.AutoOpMode;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.vision.FrameGrabberBlue;
import org.firstinspires.ftc.teamcode.vision.BlueFilterFar;

@Autonomous(name = "Blue Far")
public class Blue2 extends AutoOpMode {
    Vector2d zoneLeft = new Vector2d(-43, -40);
    Vector2d zoneMiddle = new Vector2d(-40, -36);
    Vector2d zoneRight = new Vector2d(-45, -30);
    BlueFilterFar.State position = BlueFilterFar.State.NOT_FOUND;


    public Pose2d getInitialPose() {
        return new Pose2d(-64, -30.25, Math.toRadians(0));
    }

    @Override
    public void setup() {
        FrameGrabberBlue fg = new FrameGrabberBlue(this, this.robot);

        while (!isStarted()) {
            if (gamepad1.dpad_up) {
                fg.blueFilter.offset1 = new Vector2d(fg.blueFilter.offset1.getX(),fg.blueFilter.offset1.getY() + 0.001);
            }
            if (gamepad1.dpad_down) {
                fg.blueFilter.offset1 = new Vector2d(fg.blueFilter.offset1.getX(),fg.blueFilter.offset1.getY() - 0.001);
            }

            if (gamepad1.dpad_left) {
                fg.blueFilter.offset1 = new Vector2d(fg.blueFilter.offset1.getX() - 0.001,fg.blueFilter.offset1.getY());
            }

            if (gamepad1.dpad_right) {
                fg.blueFilter.offset1 = new Vector2d(fg.blueFilter.offset1.getX() + 0.001,fg.blueFilter.offset1.getY());
            }

            position = fg.blueFilterFar.position;

            telemetry.addData("position", position);
            telemetry.update();
        }

        fg.closeCamera();
    }


    @Override
    public void run() {
        TrajectorySequence toAlign = robot.drive.roadRunnerDrive.trajectorySequenceBuilder(getCurrentPose())
                .strafeRight(10)
                .build();
        robot.drive.roadRunnerDrive.followTrajectorySequence(toAlign);
        int zone = 0;
        TrajectorySequence right = robot.drive.roadRunnerDrive.trajectorySequenceBuilder(getCurrentPose())
                .lineToConstantHeading(zoneRight)
                .build();
        TrajectorySequence middle = robot.drive.roadRunnerDrive.trajectorySequenceBuilder(getCurrentPose())
                .lineToConstantHeading(zoneMiddle)
                .build();
        TrajectorySequence left = robot.drive.roadRunnerDrive.trajectorySequenceBuilder(getCurrentPose())
                .lineToConstantHeading(zoneLeft)
                .turn(Math.toRadians(50))
                .forward(4)
                .build();

        if (position == BlueFilterFar.State.RIGHT) {
            robot.drive.roadRunnerDrive.followTrajectorySequence(right);
        } else if (position == BlueFilterFar.State.CENTER) {
            robot.drive.roadRunnerDrive.followTrajectorySequence(middle);
        } else {
            robot.drive.roadRunnerDrive.followTrajectorySequence(left);
        }

        robot.intake.depositPixel();

//        TrajectorySequence toBoardCenter = robot.drive.roadRunnerDrive.trajectorySequenceBuilder(getCurrentPose())
//                .lineToLinearHeading(new Pose2d(-36, 49, Math.toRadians(-90)))
//                .build();
//
//        TrajectorySequence toBoardLeft = robot.drive.roadRunnerDrive.trajectorySequenceBuilder(getCurrentPose())
//                .lineToLinearHeading(new Pose2d(-38, 49, Math.toRadians(-90)))
//                .build();
//
//        TrajectorySequence toBoard = robot.drive.roadRunnerDrive.trajectorySequenceBuilder(getCurrentPose())
//                .lineToLinearHeading(new Pose2d(-22.4, 50.5, Math.toRadians(-90)))
//                .build();
//
//        if (position == RedFilterFar.State.LEFT) {
//            robot.drive.roadRunnerDrive.followTrajectorySequence(toBoardLeft);
//        } else if (position == RedFilterFar.State.CENTER) {
//            robot.drive.roadRunnerDrive.followTrajectorySequence(toBoardCenter);
//        } else {
//            robot.drive.roadRunnerDrive.followTrajectorySequence(toBoard);
//        }
//
//        //     robot.drive.roadRunnerDrive.followTrajectorySequence(toBoard);
//
//
//        robot.drive.moveBackwardsTouchSensor();
//
//        robot.timer.wait(500);
//
//        robot.lift.liftToPosition(-800);
//        while(opModeIsActive() && !robot.lift.hasReachedTarget(10)) {
//            telemetry.addData("Current Height: ", robot.lift.getCurrentPosition());
//            telemetry.update();
//        }
//
//        robot.timer.wait(200);
//
//        robot.lift.autoDeposit();
//        robot.timer.wait(1000);
//        robot.lift.autoClose();
//
//        robot.lift.liftToPosition(-100);
//        while(opModeIsActive() && !robot.lift.hasReachedTarget(10)) {
//            telemetry.addData("Current Height: ", robot.lift.getCurrentPosition());
//            telemetry.update();
//        }
//
//        TrajectorySequence toPark = robot.drive.roadRunnerDrive.trajectorySequenceBuilder(getCurrentPose())
//                .strafeLeft(24)
//                .build();
//        robot.drive.roadRunnerDrive.followTrajectorySequence(toPark);
//
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this, this.telemetry, getInitialPose());
        setup();
        waitForStart();
        run();
    }
}






//package org.firstinspires.ftc.teamcode.opModes.auto;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//
//import org.firstinspires.ftc.teamcode.opModes.AutoOpMode;
//import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
//import org.firstinspires.ftc.teamcode.subsystems.Robot;
//import org.firstinspires.ftc.teamcode.vision.FrameGrabber;
//import org.firstinspires.ftc.teamcode.vision.RedFilter;
//
//@Autonomous(name = "Blue Close")
//public class Blue1 extends AutoOpMode {
//    Vector2d zoneLeft = new Vector2d(-45, 19);
//    Vector2d zoneMiddle = new Vector2d(-40, 12);
//    Vector2d zoneRight = new Vector2d(-45, 6);
//    RedFilter.State position = RedFilter.State.NOT_FOUND;
//
//
//    public Pose2d getInitialPose() {
//        return new Pose2d(-64, 10.25, Math.toRadians(0));
//    }
//
//
//    @Override
//    public void setup() {
//        FrameGrabber fg = new FrameGrabber(this, this.robot);
//
//        while (!isStarted()) {
//            if (gamepad1.dpad_up) {
//                fg.redFilter.offset1 = new Vector2d(fg.redFilter.offset1.getX(),fg.redFilter.offset1.getY() + 0.001);
//            }
//            if (gamepad1.dpad_down) {
//                fg.redFilter.offset1 = new Vector2d(fg.redFilter.offset1.getX(),fg.redFilter.offset1.getY() - 0.001);
//            }
//
//            if (gamepad1.dpad_left) {
//                fg.redFilter.offset1 = new Vector2d(fg.redFilter.offset1.getX() - 0.001,fg.redFilter.offset1.getY());
//            }
//
//            if (gamepad1.dpad_right) {
//                fg.redFilter.offset1 = new Vector2d(fg.redFilter.offset1.getX() + 0.001,fg.redFilter.offset1.getY());
//            }
//
//            position = fg.redFilter.position;
//
//            telemetry.addData("position", position);
//            telemetry.update();
//        }
//
//        fg.closeCamera();
//    }
//
//
//    @Override
//    public void run() {
//        int zone = 0;
//        TrajectorySequence left = robot.drive.roadRunnerDrive.trajectorySequenceBuilder(getCurrentPose())
//                .lineToConstantHeading(zoneLeft)
//                .build();
//        TrajectorySequence middle = robot.drive.roadRunnerDrive.trajectorySequenceBuilder(getCurrentPose())
//                .lineToConstantHeading(zoneMiddle)
//                .build();
//        TrajectorySequence right = robot.drive.roadRunnerDrive.trajectorySequenceBuilder(getCurrentPose())
//                .lineToConstantHeading(zoneRight)
//                .turn(Math.toRadians(-50))
//                .forward(4)
//                .build();
//
//        if (position == RedFilter.State.LEFT) {
//            robot.drive.roadRunnerDrive.followTrajectorySequence(left);
//        } else if (position == RedFilter.State.CENTER) {
//            robot.drive.roadRunnerDrive.followTrajectorySequence(middle);
//        } else {
//            robot.drive.roadRunnerDrive.followTrajectorySequence(right);
//        }
//
//        robot.intake.depositPixel();
//
//        TrajectorySequence toBoardCenter = robot.drive.roadRunnerDrive.trajectorySequenceBuilder(getCurrentPose())
//                .lineToLinearHeading(new Pose2d(-38, 43, Math.toRadians(-90)))
//                .build();
//        robot.drive.roadRunnerDrive.followTrajectorySequence(toBoardCenter);
//        TrajectorySequence toBoardLeft = robot.drive.roadRunnerDrive.trajectorySequenceBuilder(getCurrentPose())
//                .lineToLinearHeading(new Pose2d(-42, 48, Math.toRadians(-90)))
//                .build();
//        TrajectorySequence toBoard = robot.drive.roadRunnerDrive.trajectorySequenceBuilder(getCurrentPose())
//                .lineToLinearHeading(new Pose2d(-26, 49, Math.toRadians(-90)))
//                .build();
//        robot.drive.roadRunnerDrive.followTrajectorySequence(toBoard);
//
//
//        robot.drive.moveBackwardsTouchSensor();
//
//        robot.timer.wait(500);
//
//        robot.lift.liftToPosition(-800);
//        while(opModeIsActive() && !robot.lift.hasReachedTarget(10)) {
//            telemetry.addData("Current Height: ", robot.lift.getCurrentPosition());
//            telemetry.update();
//        }
//
//        robot.timer.wait(200);
//
//        robot.lift.autoDeposit();
//        robot.timer.wait(1000);
//        robot.lift.autoClose();
//
//        robot.lift.liftToPosition(-100);
//        while(opModeIsActive() && !robot.lift.hasReachedTarget(10)) {
//            telemetry.addData("Current Height: ", robot.lift.getCurrentPosition());
//            telemetry.update();
//        }
//
//        TrajectorySequence toPark = robot.drive.roadRunnerDrive.trajectorySequenceBuilder(getCurrentPose())
//                .strafeLeft(24)
//                .build();
//        robot.drive.roadRunnerDrive.followTrajectorySequence(toPark);
//
//    }
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        robot = new Robot(this, this.telemetry, getInitialPose());
//        setup();
//        waitForStart();
//        run();
//    }
//}
