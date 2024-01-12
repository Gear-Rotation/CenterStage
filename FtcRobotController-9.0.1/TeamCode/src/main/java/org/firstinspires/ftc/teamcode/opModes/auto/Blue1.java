//package org.firstinspires.ftc.teamcode.opModes.auto;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//
//import org.firstinspires.ftc.teamcode.opModes.AutoOpMode;
//import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
//import org.firstinspires.ftc.teamcode.subsystems.Robot;
//import org.firstinspires.ftc.teamcode.vision.BlueFilterFar;
//import org.firstinspires.ftc.teamcode.vision.FrameGrabberBlue;
//import org.firstinspires.ftc.teamcode.vision.BlueFilter;
//
//@Autonomous(name = "Blue Close")
//public class Blue1 extends AutoOpMode {
//    Vector2d zoneLeft = new Vector2d(-45, 21);
//    Vector2d zoneMiddle = new Vector2d(-40, 12);
//    Vector2d zoneRight = new Vector2d(-45, 6);
//    BlueFilter.State position = BlueFilter.State.NOT_FOUND;
//
//
//    public Pose2d getInitialPose() {
//        return new Pose2d(-64, 10.25, Math.toRadians(0));
//    }
//
//    @Override
//    public void setup() {
//        FrameGrabberBlue fg = new FrameGrabberBlue(this, this.robot);
//
//        while (!isStarted()) {
//            if (gamepad1.dpad_up) {
//                fg.blueFilter.offset1 = new Vector2d(fg.blueFilter.offset1.getX(),fg.blueFilter.offset1.getY() + 0.001);
//            }
//            if (gamepad1.dpad_down) {
//                fg.blueFilter.offset1 = new Vector2d(fg.blueFilter.offset1.getX(),fg.blueFilter.offset1.getY() - 0.001);
//            }
//
//            if (gamepad1.dpad_left) {
//                fg.blueFilter.offset1 = new Vector2d(fg.blueFilter.offset1.getX() - 0.001,fg.blueFilter.offset1.getY());
//            }
//
//            if (gamepad1.dpad_right) {
//                fg.blueFilter.offset1 = new Vector2d(fg.blueFilter.offset1.getX() + 0.001,fg.blueFilter.offset1.getY());
//            }
//
//            position = fg.blueFilter.position;
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
//        //robot goes to corresponding zone as sensed above
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
////implements the code from above
//        if (position == BlueFilter.State.LEFT) {
//            robot.drive.roadRunnerDrive.followTrajectorySequence(left);
//        } else if (position == BlueFilter.State.CENTER) {
//            robot.drive.roadRunnerDrive.followTrajectorySequence(middle);
//        } else {
//            robot.drive.roadRunnerDrive.followTrajectorySequence(right);
//        }
//
//        //deposits pixel
//        robot.intake.depositPixel();
//
//
////robot goes to the center of the board so that both touch sensors are pressed
//        TrajectorySequence toCenterRobot = robot.drive.roadRunnerDrive.trajectorySequenceBuilder(getCurrentPose())
//                .lineToLinearHeading(new Pose2d(-36, 49.5, Math.toRadians(-90)))
//                .build();
//        robot.drive.roadRunnerDrive.followTrajectorySequence(toCenterRobot);
//
////make sure the robot is straight using touch sensors
//        robot.drive.moveBackwardsTouchSensor();
//
//        robot.timer.wait(500);
//
////lift the slide to the correct position
//        robot.lift.liftToPosition(-800);
//        while(opModeIsActive() && !robot.lift.hasReachedTarget(10)) {
//            telemetry.addData("Current Height: ", robot.lift.getCurrentPosition());
//            telemetry.update();
//        }
//        //coordinates to correct board positioning
//        TrajectorySequence toBoardCenter = robot.drive.roadRunnerDrive.trajectorySequenceBuilder(getCurrentPose())
//              //  .lineToLinearHeading(new Pose2d(-37, 50.5, Math.toRadians(-90)))
//                .back(1)
//                .build();
//
//        TrajectorySequence toBoardLeft = robot.drive.roadRunnerDrive.trajectorySequenceBuilder(getCurrentPose())
//                .forward(2)
//                .strafeRight(7)
//                .back(4)
//                //.lineToLinearHeading(new Pose2d(-42, 49, Math.toRadians(-90)))
//                // .forward(2)
//                .build();
//
//        TrajectorySequence toBoardRight = robot.drive.roadRunnerDrive.trajectorySequenceBuilder(getCurrentPose())
//                .forward(1)
//                .strafeLeft(10)
//                .back(3)
////                .lineToLinearHeading(new Pose2d(-34.5, 52, Math.toRadians(-90)))
////                .back(6)
//                .build();
////executes the commands from above
//        if (position == BlueFilter.State.LEFT) {
//            robot.drive.roadRunnerDrive.followTrajectorySequence(toBoardLeft);
//        } else if (position == BlueFilter.State.CENTER) {
//            robot.drive.roadRunnerDrive.followTrajectorySequence(toBoardCenter);
//        } else {
//            robot.drive.roadRunnerDrive.followTrajectorySequence(toBoardRight);
//        }
//
//        //     robot.drive.roadRunnerDrive.followTrajectorySequence(toBoard);
//
//
//        robot.timer.wait(200);
////opens block servo to drop pixel and closes it back up when finished
//        robot.lift.autoDeposit();
//        robot.timer.wait(1000);
//        robot.lift.autoClose();
////brings slide down
//        robot.lift.liftToPosition(-100);
//        while(opModeIsActive() && !robot.lift.hasReachedTarget(10)) {
//            telemetry.addData("Current Height: ", robot.lift.getCurrentPosition());
//            telemetry.update();
//        }
////strafes right to be in the parking zone
//        TrajectorySequence toPark = robot.drive.roadRunnerDrive.trajectorySequenceBuilder(getCurrentPose())
//                .forward(2)
//                .strafeLeft(25)
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







package org.firstinspires.ftc.teamcode.opModes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opModes.AutoOpMode;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.vision.BlueFilterFar;
import org.firstinspires.ftc.teamcode.vision.FrameGrabberBlue;
import org.firstinspires.ftc.teamcode.vision.BlueFilter;

@Autonomous(name = "Blue Close")
public class Blue1 extends AutoOpMode {
    //positions of spike marks
    Vector2d zoneLeft = new Vector2d(-45, 22);
    Vector2d zoneMiddle = new Vector2d(-40, 12);
    Vector2d zoneRight = new Vector2d(-45, 6);
    BlueFilter.State position = BlueFilter.State.NOT_FOUND;


    public Pose2d getInitialPose() {
        //starting position
        return new Pose2d(-64, 10.25, Math.toRadians(0));
    }

    @Override
    public void setup() {
        FrameGrabberBlue fg = new FrameGrabberBlue(this, this.robot);
        //way to adjust camera during init before running, last minute

        while (!isStarted()) {

            if (gamepad1.a) {
                fg.blueFilter.threshold += 0.001;
            }
            if (gamepad1.b) {
                fg.blueFilter.threshold -= 0.001;
            }


            if (gamepad1.dpad_up) {
                fg.blueFilter.offset = new Vector2d(fg.blueFilter.offset.getX(),fg.blueFilter.offset.getY() + 0.001);
            }
            if (gamepad1.dpad_down) {
                fg.blueFilter.offset = new Vector2d(fg.blueFilter.offset.getX(),fg.blueFilter.offset.getY() - 0.001);
            }

            if (gamepad1.dpad_left) {
                fg.blueFilter.offset = new Vector2d(fg.blueFilter.offset.getX() - 0.001,fg.blueFilter.offset.getY());
            }

            if (gamepad1.dpad_right) {
                fg.blueFilter.offset = new Vector2d(fg.blueFilter.offset.getX() + 0.001,fg.blueFilter.offset.getY());
            }

            if (gamepad2.dpad_up) {
                fg.blueFilter.offset1 = new Vector2d(fg.blueFilter.offset1.getX(),fg.blueFilter.offset1.getY() + 0.001);
            }
            if (gamepad2.dpad_down) {
                fg.blueFilter.offset1 = new Vector2d(fg.blueFilter.offset1.getX(),fg.blueFilter.offset1.getY() - 0.001);
            }

            if (gamepad2.dpad_left) {
                fg.blueFilter.offset1 = new Vector2d(fg.blueFilter.offset1.getX() - 0.001,fg.blueFilter.offset1.getY());
            }

            if (gamepad2.dpad_right) {
                fg.blueFilter.offset1 = new Vector2d(fg.blueFilter.offset1.getX() + 0.001,fg.blueFilter.offset1.getY());
            }

            position = fg.blueFilter.position;

            telemetry.addData("position", position);

            telemetry.addData("offset", fg.blueFilter.offset);
            telemetry.addData("offset1", fg.blueFilter.offset1);
            telemetry.addData("threshold", fg.blueFilter.threshold);
            telemetry.update();
        }

        fg.closeCamera();
    }


    @Override
    public void run() {

//        robot.timer.wait(5000);
        int zone = 0;
        TrajectorySequence left = robot.drive.roadRunnerDrive.trajectorySequenceBuilder(getCurrentPose())
                .lineToConstantHeading(zoneLeft)
                .build();
        TrajectorySequence middle = robot.drive.roadRunnerDrive.trajectorySequenceBuilder(getCurrentPose())
                .lineToConstantHeading(zoneMiddle)
                .build();
        TrajectorySequence right = robot.drive.roadRunnerDrive.trajectorySequenceBuilder(getCurrentPose())
                .lineToConstantHeading(zoneRight)
                .turn(Math.toRadians(-50))
                .forward(4)
                .build();

        if (position == BlueFilter.State.LEFT) {
            robot.drive.roadRunnerDrive.followTrajectorySequence(left);
        } else if (position == BlueFilter.State.CENTER) {
            robot.drive.roadRunnerDrive.followTrajectorySequence(middle);
        } else {
            robot.drive.roadRunnerDrive.followTrajectorySequence(right);
        }

        robot.intake.depositPixel();


        TrajectorySequence toBoardCenter = robot.drive.roadRunnerDrive.trajectorySequenceBuilder(getCurrentPose())
                .lineToLinearHeading(new Pose2d(-36, 49, Math.toRadians(-90)))
                .build();

        TrajectorySequence toBoardLeft = robot.drive.roadRunnerDrive.trajectorySequenceBuilder(getCurrentPose())
                .lineToLinearHeading(new Pose2d(-41, 49, Math.toRadians(-90)))
                .build();

        TrajectorySequence toBoard = robot.drive.roadRunnerDrive.trajectorySequenceBuilder(getCurrentPose())
                .lineToLinearHeading(new Pose2d(-22.4, 50.5, Math.toRadians(-90)))
                .build();

        if (position == BlueFilter.State.LEFT) {
            robot.drive.roadRunnerDrive.followTrajectorySequence(toBoardLeft);
        } else if (position == BlueFilter.State.CENTER) {
            robot.drive.roadRunnerDrive.followTrajectorySequence(toBoardCenter);
        } else {
            robot.drive.roadRunnerDrive.followTrajectorySequence(toBoard);
        }

        robot.intake.disarmIntake();

        //     robot.drive.roadRunnerDrive.followTrajectorySequence(toBoard);


        robot.drive.moveBackwardsTouchSensor();

        robot.timer.wait(500);

        robot.lift.liftToPosition(-800);
        while (opModeIsActive() && !robot.lift.hasReachedTarget(10)) {
            telemetry.addData("Current Height: ", robot.lift.getCurrentPosition());
            telemetry.update();
        }

        robot.timer.wait(200);

        robot.lift.autoDeposit();
        robot.timer.wait(1000);
        robot.lift.autoClose();

        robot.lift.liftToPosition(-100);
        while (opModeIsActive() && !robot.lift.hasReachedTarget(10)) {
            telemetry.addData("Current Height: ", robot.lift.getCurrentPosition());
            telemetry.update();
        }


        //strafes right to be in the parking zone
        TrajectorySequence parkCenter = robot.drive.roadRunnerDrive.trajectorySequenceBuilder(getCurrentPose())
                .strafeRight(28.5)
                .build();

        TrajectorySequence parkLeft = robot.drive.roadRunnerDrive.trajectorySequenceBuilder(getCurrentPose())
                .strafeRight(23)
                .build();

        TrajectorySequence parkRight = robot.drive.roadRunnerDrive.trajectorySequenceBuilder(getCurrentPose())
                .strafeRight(39)
                .build();

        //executes the commands from above
        if (position == BlueFilter.State.LEFT) {
            robot.drive.roadRunnerDrive.followTrajectorySequence(parkLeft);
        } else if (position == BlueFilter.State.CENTER) {
            robot.drive.roadRunnerDrive.followTrajectorySequence(parkCenter);
        } else {
            robot.drive.roadRunnerDrive.followTrajectorySequence(parkRight);
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this, this.telemetry, getInitialPose());
        setup();
        waitForStart();
        run();
    }
}

