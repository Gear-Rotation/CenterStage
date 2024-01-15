package org.firstinspires.ftc.teamcode.opModes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opModes.AutoOpMode;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.vision.BlueFilter;
import org.firstinspires.ftc.teamcode.vision.FrameGrabberRed;
import org.firstinspires.ftc.teamcode.vision.RedFilter;

@Autonomous(name = "Red Close")
public class Red1Close extends AutoOpMode {
    //spike mrk positions
    Vector2d zoneRight = new Vector2d(45, 24.25);
    Vector2d zoneMiddle = new Vector2d(38, 14);
    Vector2d zoneLeft = new Vector2d(45, 12);
    RedFilter.State position = RedFilter.State.NOT_FOUND;


    public Pose2d getInitialPose() {
        //robot starting position
        return new Pose2d(64, 10.25, Math.toRadians(180));
    }

    @Override
    public void setup() {
        FrameGrabberRed fg = new FrameGrabberRed(this, this.robot);
//way to adjust camera before match
        while (!isStarted()) {
            if (gamepad1.dpad_up) {
                fg.redFilter.offset = new Vector2d(fg.redFilter.offset.getX(), fg.redFilter.offset.getY() + 0.001);
            }
            if (gamepad1.dpad_down) {
                fg.redFilter.offset = new Vector2d(fg.redFilter.offset.getX(), fg.redFilter.offset.getY() - 0.001);
            }

            if (gamepad1.dpad_left) {
                fg.redFilter.offset = new Vector2d(fg.redFilter.offset.getX() - 0.001, fg.redFilter.offset.getY());
            }

            if (gamepad1.dpad_right) {
                fg.redFilter.offset = new Vector2d(fg.redFilter.offset.getX() + 0.001, fg.redFilter.offset.getY());
            }

            if (gamepad2.dpad_up) {
                fg.redFilter.offset1 = new Vector2d(fg.redFilter.offset1.getX(), fg.redFilter.offset1.getY() + 0.001);
            }
            if (gamepad2.dpad_down) {
                fg.redFilter.offset1 = new Vector2d(fg.redFilter.offset1.getX(), fg.redFilter.offset1.getY() - 0.001);
            }

            if (gamepad2.dpad_left) {
                fg.redFilter.offset1 = new Vector2d(fg.redFilter.offset1.getX() - 0.001, fg.redFilter.offset1.getY());
            }

            if (gamepad2.dpad_right) {
                fg.redFilter.offset1 = new Vector2d(fg.redFilter.offset1.getX() + 0.001, fg.redFilter.offset1.getY());
            }

            position = fg.redFilter.position;

            telemetry.addData("position", position);

            telemetry.addData("offset", fg.redFilter.offset);
            telemetry.addData("offset1", fg.redFilter.offset1);
            telemetry.update();
        }

        fg.closeCamera();
    }


    @Override
    public void run() {
        //        robot.timer.wait(5000);
        int zone = 0;
        //move to the middle of the tile
        TrajectorySequence toMove = robot.drive.roadRunnerDrive.trajectorySequenceBuilder(getCurrentPose())
                .strafeRight(4)
                .build();
        robot.drive.roadRunnerDrive.followTrajectorySequence(toMove);
        //goign to different positions based off of the webcam

        TrajectorySequence left = robot.drive.roadRunnerDrive.trajectorySequenceBuilder(getCurrentPose())
                .lineToConstantHeading(zoneLeft)
                .turn(Math.toRadians(50))
                .forward(4.5)
                .build();
        TrajectorySequence middle = robot.drive.roadRunnerDrive.trajectorySequenceBuilder(getCurrentPose())
                .lineToConstantHeading(zoneMiddle)
                .build();
        TrajectorySequence right = robot.drive.roadRunnerDrive.trajectorySequenceBuilder(getCurrentPose())
                .lineToConstantHeading(zoneRight)
                .build();
//executes the precious code
        if (position == RedFilter.State.LEFT) {
            robot.drive.roadRunnerDrive.followTrajectorySequence(left);
        } else if (position == RedFilter.State.CENTER) {
            robot.drive.roadRunnerDrive.followTrajectorySequence(middle);
        } else {
            robot.drive.roadRunnerDrive.followTrajectorySequence(right);
        }
//deposits pixel
        robot.intake.depositPixel();

        //board positions to drop pixel
        TrajectorySequence toBoardCenter = robot.drive.roadRunnerDrive.trajectorySequenceBuilder(getCurrentPose())
                .lineToLinearHeading(new Pose2d(36, 49, Math.toRadians(-90)))
                .build();

        TrajectorySequence toBoardRight = robot.drive.roadRunnerDrive.trajectorySequenceBuilder(getCurrentPose())
                .lineToLinearHeading(new Pose2d( 43, 49, Math.toRadians(-90)))
                .build();

        TrajectorySequence toBoardLeft = robot.drive.roadRunnerDrive.trajectorySequenceBuilder(getCurrentPose())
                .lineToLinearHeading(new Pose2d(25, 50.5, Math.toRadians(-90)))
                .build();

        //executes the previous block

        if (position == RedFilter.State.RIGHT) {
            robot.drive.roadRunnerDrive.followTrajectorySequence(toBoardRight);
        } else if (position == RedFilter.State.CENTER) {
            robot.drive.roadRunnerDrive.followTrajectorySequence(toBoardCenter);
        } else {
            robot.drive.roadRunnerDrive.followTrajectorySequence(toBoardLeft);
        }

        robot.intake.disarmIntake();

        //     robot.drive.roadRunnerDrive.followTrajectorySequence(toBoard);

//aligns to the backdrop
        robot.drive.moveBackwardsTouchSensor();

        //wait
        robot.timer.wait(500);

        //lifts slide
        robot.lift.liftToPosition(-800);
        while(opModeIsActive() && !robot.lift.hasReachedTarget(10)) {
            telemetry.addData("Current Height: ", robot.lift.getCurrentPosition());
            telemetry.update();
        }

        robot.timer.wait(200);

        //opens block servo to drop and closes t again after dropped
        robot.lift.autoDeposit();
        robot.timer.wait(1000);
        robot.lift.autoClose();

//lowers slide
        robot.lift.liftToPosition(-100);
        //lowers until it is within 10 encoder ticks of position
        while(opModeIsActive() && !robot.lift.hasReachedTarget(10)) {
            telemetry.addData("Current Height: ", robot.lift.getCurrentPosition());
            telemetry.update();
        }

        //parks the robot
        TrajectorySequence parkCenter = robot.drive.roadRunnerDrive.trajectorySequenceBuilder(getCurrentPose())
                .forward(3)
                //park in the corner near the wall
                //.strafeLeft(27.5)
                //park in the diagnoal center spot
                .strafeRight(28)
                .back(14)
                .build();

        TrajectorySequence parkLeft = robot.drive.roadRunnerDrive.trajectorySequenceBuilder(getCurrentPose())
                .forward(3)
                //park in the corner near the wall
                //.strafeLeft(39)
                //park in the diagnoal center spot
                .strafeRight(20)
                .back(14)
                .build();

        TrajectorySequence parkRight = robot.drive.roadRunnerDrive.trajectorySequenceBuilder(getCurrentPose())
                .forward(3)
                //park in the corner near the wall
              //  .strafeLeft(20)
                //park in the diagnoal center spot
                .strafeRight(37)
                .back(14)
                .build();

        //executes the commands from above
        if (position == RedFilter.State.LEFT) {
            robot.drive.roadRunnerDrive.followTrajectorySequence(parkLeft);
        } else if (position == RedFilter.State.CENTER) {
            robot.drive.roadRunnerDrive.followTrajectorySequence(parkCenter);
        } else {
            robot.drive.roadRunnerDrive.followTrajectorySequence(parkRight);
        }

    }

    //plays the code
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this, this.telemetry, getInitialPose());
        setup();
        waitForStart();
        run();
    }
}
