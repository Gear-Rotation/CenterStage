package org.firstinspires.ftc.teamcode.opModes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.vision.RedFilter;
import org.firstinspires.ftc.teamcode.vision.RedFilterFar;
import org.firstinspires.ftc.teamcode.opModes.AutoOpMode;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.vision.FrameGrabberRed2;

@Autonomous(name = "Red Far")
public class Red2 extends AutoOpMode {
    Vector2d zoneRight = new Vector2d(43, -30);
    Vector2d zoneMiddle = new Vector2d(39, -36);
    Vector2d zoneLeft = new Vector2d(43, -46);
    RedFilterFar.State position = RedFilterFar.State.NOT_FOUND;


    public Pose2d getInitialPose() {
        return new Pose2d(64, -34.25, Math.toRadians(180));
    }

    @Override
    public void setup() {
        FrameGrabberRed2 fg = new FrameGrabberRed2(this, this.robot);

        while (!isStarted()) {
            if (gamepad1.dpad_up) {
                fg.redFilterFar.offset = new Vector2d(fg.redFilterFar.offset.getX(), fg.redFilterFar.offset.getY() + 0.001);
            }
            if (gamepad1.dpad_down) {
                fg.redFilterFar.offset = new Vector2d(fg.redFilterFar.offset.getX(), fg.redFilterFar.offset.getY() - 0.001);
            }

            if (gamepad1.dpad_left) {
                fg.redFilterFar.offset = new Vector2d(fg.redFilterFar.offset.getX() - 0.001, fg.redFilterFar.offset.getY());
            }

            if (gamepad1.dpad_right) {
                fg.redFilterFar.offset = new Vector2d(fg.redFilterFar.offset.getX() + 0.001, fg.redFilterFar.offset.getY());
            }

            if (gamepad2.dpad_up) {
                fg.redFilterFar.offset1 = new Vector2d(fg.redFilterFar.offset1.getX(), fg.redFilterFar.offset1.getY() + 0.001);
            }
            if (gamepad2.dpad_down) {
                fg.redFilterFar.offset1 = new Vector2d(fg.redFilterFar.offset1.getX(), fg.redFilterFar.offset1.getY() - 0.001);
            }

            if (gamepad2.dpad_left) {
                fg.redFilterFar.offset1 = new Vector2d(fg.redFilterFar.offset1.getX() - 0.001, fg.redFilterFar.offset1.getY());
            }

            if (gamepad2.dpad_right) {
                fg.redFilterFar.offset1 = new Vector2d(fg.redFilterFar.offset1.getX() + 0.001, fg.redFilterFar.offset1.getY());
            }

            position = fg.redFilterFar.position;

            telemetry.addData("position", position);

            telemetry.addData("offset", fg.redFilterFar.offset);
            telemetry.addData("offset1", fg.redFilterFar.offset1);
            telemetry.update();
        }

        fg.closeCamera();
    }


    @Override
    public void run() {
//        TrajectorySequence toAlign = robot.drive.roadRunnerDrive.trajectorySequenceBuilder(getCurrentPose())
//                .strafeLeft(5)
//                .build();
//        robot.drive.roadRunnerDrive.followTrajectorySequence(toAlign);
         //     robot.timer.wait(5000);
        int zone = 0;
        TrajectorySequence right = robot.drive.roadRunnerDrive.trajectorySequenceBuilder(getCurrentPose())
                .lineToConstantHeading(zoneRight)
                .turn(Math.toRadians(-57))
                .forward(3)
                .build();
        TrajectorySequence middle = robot.drive.roadRunnerDrive.trajectorySequenceBuilder(getCurrentPose())
                .lineToConstantHeading(zoneMiddle)
                .build();
        TrajectorySequence left = robot.drive.roadRunnerDrive.trajectorySequenceBuilder(getCurrentPose())
                .lineToConstantHeading(zoneLeft)
                .build();

        if (position == RedFilterFar.State.RIGHT) {
            robot.drive.roadRunnerDrive.followTrajectorySequence(right);
        } else if (position == RedFilterFar.State.CENTER) {
            robot.drive.roadRunnerDrive.followTrajectorySequence(middle);
        } else {
            robot.drive.roadRunnerDrive.followTrajectorySequence(left);
        }

        robot.intake.depositPixel();

        TrajectorySequence toReachBoardCenter = robot.drive.roadRunnerDrive.trajectorySequenceBuilder(getCurrentPose())
                .back(4)
                .addDisplacementMarker(() -> {
                    robot.intake.raiseIntake();
                })
                .strafeLeft(20)
                .forward(32)
                .turn(Math.toRadians(90))
                .build();

        TrajectorySequence toReachBoardLeft = robot.drive.roadRunnerDrive.trajectorySequenceBuilder(getCurrentPose())
                .back(3)
                //as moving away we disarm the intake
                .addDisplacementMarker(() -> {
                    robot.intake.raiseIntake();
                })
                .strafeRight(16)
                .forward(30)
                .turn(Math.toRadians(90))
                .build();
        TrajectorySequence toReachBoardRight = robot.drive.roadRunnerDrive.trajectorySequenceBuilder(getCurrentPose())
//                .back(9)
//                .addDisplacementMarker(() -> {
//                    robot.intake.raiseIntake();
//                })
//                .turn(Math.toRadians(147))
//                .strafeRight(55)
//                .build();
                .back(7)
                .addDisplacementMarker(() -> {
                    robot.intake.raiseIntake();
                })
                .turn(Math.toRadians(59))
                .forward(28)
                .turn(Math.toRadians(90))
                .build();

//
        // Depending on the location of the team prop the robot will follow the corresponding path, which are stated above
        if (position == RedFilterFar.State.LEFT) {
            robot.drive.roadRunnerDrive.followTrajectorySequence(toReachBoardLeft);
        } else if (position == RedFilterFar.State.CENTER) {
            robot.drive.roadRunnerDrive.followTrajectorySequence(toReachBoardCenter);
        } else {
            robot.drive.roadRunnerDrive.followTrajectorySequence(toReachBoardRight);
        }

        robot.intake.disarmIntake();
//
        //robot aligns to go under stage door
        TrajectorySequence toCenterRobot = robot.drive.roadRunnerDrive.trajectorySequenceBuilder(getCurrentPose())
                .lineToLinearHeading(new Pose2d(10, 30, Math.toRadians(-90)))
//                .back(20)
                .build();
        robot.drive.roadRunnerDrive.followTrajectorySequence(toCenterRobot);
        //coordinates to correct board positioning
        TrajectorySequence toBoardCenter = robot.drive.roadRunnerDrive.trajectorySequenceBuilder(getCurrentPose())
                .lineToLinearHeading(new Pose2d(41, 50.5, Math.toRadians(-90)))
                .build();

        TrajectorySequence toBoardLeft = robot.drive.roadRunnerDrive.trajectorySequenceBuilder(getCurrentPose())
                //  .back(1)
                .lineToLinearHeading(new Pose2d(32, 51.5, Math.toRadians(-90)))
                // .forward(2)
                .build();

        TrajectorySequence toBoardRight = robot.drive.roadRunnerDrive.trajectorySequenceBuilder(getCurrentPose())
//                .forward(2)0
//                .strafeLeft(3)
//                .back(2)
                .lineToLinearHeading(new Pose2d(49, 53, Math.toRadians(-90)))
                //   .back(6)
                .build();

        //executes the commands from above
        if (position == RedFilterFar.State.LEFT) {
            robot.drive.roadRunnerDrive.followTrajectorySequence(toBoardLeft);
        } else if (position == RedFilterFar.State.CENTER) {
            robot.drive.roadRunnerDrive.followTrajectorySequence(toBoardCenter);
        } else {
            robot.drive.roadRunnerDrive.followTrajectorySequence(toBoardRight);
        }

//
        //make sure the robot is straight using touch sensors
        robot.drive.moveBackwardsTouchSensor();
//
        robot.timer.wait(500);

        // lift the slide to the correct position
      //  robot.lift.liftToPosition(-1200);
        robot.lift.liftToPosition(-800);
        while (opModeIsActive() && !robot.lift.hasReachedTarget(10)) {
            telemetry.addData("Current Height: ", robot.lift.getCurrentPosition());
            telemetry.update();
        }
//
//
//        //      robot.drive.roadRunnerDrive.followTrajectorySequence(toBoard);
//
//
        robot.timer.wait(200);
        //opens block servo to drop pixel and closes it back up when finished
        robot.lift.autoDeposit();
        robot.timer.wait(1000);
        robot.lift.autoClose();
//
        //brings slide down
        robot.lift.liftToPosition(-100);
        while (opModeIsActive() && !robot.lift.hasReachedTarget(10)) {
            telemetry.addData("Current Height: ", robot.lift.getCurrentPosition());
            telemetry.update();
        }

        //parks the robot
        TrajectorySequence parkCenter = robot.drive.roadRunnerDrive.trajectorySequenceBuilder(getCurrentPose())
                .forward(3)
                //park in the corner near the wall
                //.strafeLeft(27.5)
                //park in the diagnoal center spot
                .strafeRight(26)
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
                .strafeRight(35)
                .back(14)
                .build();

        //executes the commands from above
        if (position == RedFilterFar.State.LEFT) {
            robot.drive.roadRunnerDrive.followTrajectorySequence(parkLeft);
        } else if (position == RedFilterFar.State.CENTER) {
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

//
//
//
//
//



