package org.firstinspires.ftc.teamcode.opModes.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp(name="Motor Test", group="Linear Opmode")
public class MotorTest extends LinearOpMode {
    private Robot robot;

    @Override
    public void runOpMode() {
        Robot robot = new Robot(this, this.telemetry);

        waitForStart();
        //robot.lift.targetSlidePosition = robot.lift.IDLE_HEIGHT;
        robot.lift.currentSlideState = Lift.SlideStates.RUN_POSITION;

        while (opModeIsActive()) {
            robot.drive.forwardTest();
        }
    }
}