package org.firstinspires.ftc.teamcode.opModes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.vision.BlueFilter;
import org.firstinspires.ftc.teamcode.vision.BlueFilterFar;
import org.firstinspires.ftc.teamcode.opModes.AutoOpMode;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.vision.FrameGrabberBlue2;


@Autonomous(name = "Blue Far")
public class Blue2 extends AutoOpMode {
    Vector2d zoneRight = new Vector2d(-43, -45.5);
    Vector2d zoneMiddle = new Vector2d(-38, -36);
    Vector2d zoneLeft = new Vector2d(-43, -30);
    BlueFilterFar.State position = BlueFilterFar.State.NOT_FOUND;


    public Pose2d getInitialPose() {
        return new Pose2d(-64, -30.25, Math.toRadians(0));
    }

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

            position = fg.blueFilterFar.position;

            telemetry.addData("position", position);
            telemetry.update();
        }

        fg.closeCamera();
    }


    @Override
    public void run() {
        TrajectorySequence toAlign = robot.drive.roadRunnerDrive.trajectorySequenceBuilder(getCurrentPose())
                .strafeRight(5)
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

        TrajectorySequence toReachBoardCenter = robot.drive.roadRunnerDrive.trajectorySequenceBuilder(getCurrentPose())
                //strafe right to get away from pixel
                .lineToLinearHeading(new Pose2d(-51, -36, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(-51, -14, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(-12, -14, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(-12, 30, Math.toRadians(-90)))
                .build();

        TrajectorySequence toReachBoardLeft = robot.drive.roadRunnerDrive.trajectorySequenceBuilder(getCurrentPose())
                //back up 4 in away from pixel
                .lineToLinearHeading(new Pose2d(-48, -30, Math.toRadians(0)))
                //strafeleft to get away from pixel
                .lineToLinearHeading(new Pose2d(-48, -34, Math.toRadians(0)))
                //forward 30 before turning to lifting door
                .lineToLinearHeading(new Pose2d(-14, -34, Math.toRadians(0)))
                //reach center of the field(truss)
                .lineToLinearHeading(new Pose2d(-14, -14, Math.toRadians(-90)))
                //go to the other side of the field
                .lineToLinearHeading(new Pose2d(-14, 30, Math.toRadians(-90)))
                .build();

        TrajectorySequence toReachBoardRight = robot.drive.roadRunnerDrive.trajectorySequenceBuilder(getCurrentPose())
                //back up 4 in away from pixel
                .lineToLinearHeading(new Pose2d(-48, -45.5, Math.toRadians(0)))
                //strafeleft to get away from pixel
                .lineToLinearHeading(new Pose2d(-48, -34, Math.toRadians(0)))
                //forward 30 before turning to lifting door
                .lineToLinearHeading(new Pose2d(-14, -34, Math.toRadians(0)))
                //reach center of the field(truss)
                .lineToLinearHeading(new Pose2d(-14, -14, Math.toRadians(-90)))
                //go to the other side of the field
                .lineToLinearHeading(new Pose2d(-14, 30, Math.toRadians(-90)))
                .build();
// Depending on the location of the team prop the robot will follow the corresponding path, which are stated above
        if (position == BlueFilterFar.State.LEFT) {
            robot.drive.roadRunnerDrive.followTrajectorySequence(toReachBoardLeft);
        } else if (position == BlueFilterFar.State.CENTER) {
            robot.drive.roadRunnerDrive.followTrajectorySequence(toReachBoardCenter);
        } else {
            robot.drive.roadRunnerDrive.followTrajectorySequence(toReachBoardRight);
        }
//robot goes to the center of the board so that both touch sensors are pressed
        TrajectorySequence toCenterRobot = robot.drive.roadRunnerDrive.trajectorySequenceBuilder(getCurrentPose())
                .lineToLinearHeading(new Pose2d(-37, 49, Math.toRadians(-90)))
                .build();
        robot.drive.roadRunnerDrive.followTrajectorySequence(toCenterRobot);

//make sure the robot is straight using touch sensors
        robot.drive.moveBackwardsTouchSensor();

        robot.timer.wait(500);

//lift the slide to the correct position
        robot.lift.liftToPosition(-800);
        while(opModeIsActive() && !robot.lift.hasReachedTarget(10)) {
            telemetry.addData("Current Height: ", robot.lift.getCurrentPosition());
            telemetry.update();
        }
        //coordinates to correct board positioning
        TrajectorySequence toBoardCenter = robot.drive.roadRunnerDrive.trajectorySequenceBuilder(getCurrentPose())
                .lineToLinearHeading(new Pose2d(-37, 50.5, Math.toRadians(-90)))
                .build();

        TrajectorySequence toBoardLeft = robot.drive.roadRunnerDrive.trajectorySequenceBuilder(getCurrentPose())
              //  .back(1)
                .lineToLinearHeading(new Pose2d(-42, 49, Math.toRadians(-90)))
               // .forward(2)
                .build();

        TrajectorySequence toBoardRight = robot.drive.roadRunnerDrive.trajectorySequenceBuilder(getCurrentPose())
                .lineToLinearHeading(new Pose2d(-35, 52, Math.toRadians(-90)))
                .build();
//executes the commands from above
        if (position == BlueFilterFar.State.LEFT) {
            robot.drive.roadRunnerDrive.followTrajectorySequence(toBoardLeft);
        } else if (position == BlueFilterFar.State.CENTER) {
            robot.drive.roadRunnerDrive.followTrajectorySequence(toBoardCenter);
        } else {
            robot.drive.roadRunnerDrive.followTrajectorySequence(toBoardRight);
        }

        //     robot.drive.roadRunnerDrive.followTrajectorySequence(toBoard);


        robot.timer.wait(200);
//opens block servo to drop pixel and closes it back up when finished
        robot.lift.autoDeposit();
        robot.timer.wait(1000);
        robot.lift.autoClose();
//brings slide down
        robot.lift.liftToPosition(-100);
        while(opModeIsActive() && !robot.lift.hasReachedTarget(10)) {
            telemetry.addData("Current Height: ", robot.lift.getCurrentPosition());
            telemetry.update();
        }
//strafes right to be in the parking zone
        TrajectorySequence toPark = robot.drive.roadRunnerDrive.trajectorySequenceBuilder(getCurrentPose())
                .strafeLeft(20)
                .build();
        robot.drive.roadRunnerDrive.followTrajectorySequence(toPark);

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
