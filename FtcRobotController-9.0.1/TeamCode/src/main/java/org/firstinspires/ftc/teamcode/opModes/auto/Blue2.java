package org.firstinspires.ftc.teamcode.opModes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opModes.AutoOpMode;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Autonomous(name = "Blue Far")
public class Blue2 extends AutoOpMode {
    Vector2d zoneZero = new Vector2d(45, 17);


    public Pose2d getInitialPose() {
        return new Pose2d(-64, 10.25, Math.toRadians(0));
    }
    @Override
    public void setup() {

    }

    @Override
    public void run() {
        int zone = 0;
        TrajectorySequence forward = robot.drive.roadRunnerDrive.trajectorySequenceBuilder(getCurrentPose())
                .lineToConstantHeading(zoneZero)
                .build();
        robot.drive.roadRunnerDrive.followTrajectorySequence(forward);

        robot.timer.wait(1000);
        TrajectorySequence toBoard = robot.drive.roadRunnerDrive.trajectorySequenceBuilder(getCurrentPose())
                .lineToLinearHeading(new Pose2d(-36, 48, Math.toRadians(-90)))
                .build();
        robot.drive.roadRunnerDrive.followTrajectorySequence(toBoard);

        robot.timer.wait(500);

        robot.lift.liftToPosition(-1000);
        while(opModeIsActive() && !robot.lift.hasReachedTarget(10)) {
            telemetry.addData("Current Height: ", robot.lift.getCurrentPosition());
            telemetry.update();
        }

        robot.timer.wait(200);

        robot.lift.liftToPosition(-100);
        while(opModeIsActive() && !robot.lift.hasReachedTarget(10)) {
            telemetry.addData("Current Height: ", robot.lift.getCurrentPosition());
            telemetry.update();
        }

        TrajectorySequence toPark = robot.drive.roadRunnerDrive.trajectorySequenceBuilder(getCurrentPose())
                .strafeLeft(24)
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
