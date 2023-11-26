package org.firstinspires.ftc.teamcode.opModes.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp(name="Teleop", group="Linear Opmode")
public class Teleop extends LinearOpMode {
    private Robot robot;

    @Override
    public void runOpMode() {
        Robot robot = new Robot(this, this.telemetry);

        waitForStart();
       //robot.lift.targetSlidePosition = robot.lift.IDLE_HEIGHT;
       robot.lift.currentSlideState = Lift.SlideStates.RUN_POSITION;

        while (opModeIsActive()) {
            robot.loop();

        }
    }
}