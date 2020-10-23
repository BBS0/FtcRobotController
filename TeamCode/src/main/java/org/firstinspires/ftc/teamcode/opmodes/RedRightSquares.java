package org.firstinspires.ftc.teamcode.opmodes;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.BotBuildersMecBot;
import org.firstinspires.ftc.teamcode.robot.OpenCvDetector;

import static android.icu.lang.UCharacter.DecompositionType.SQUARE;

@Autonomous(name = "RedRightSquares", group = "drive")
public class RedRightSquares extends OpenCvDetector {

    @Override
    public void runOpMode() throws InterruptedException {
        BotBuildersMecBot drive = new BotBuildersMecBot(hardwareMap);

        Trajectory moveForward = new TrajectoryBuilder(new Pose2d(), drive.constraints)
                .forward(20)
                .build();

        Trajectory locateA = new TrajectoryBuilder(new Pose2d(), drive.constraints)
                .splineTo(new Vector2d(70, -10), 0)
                .build();

        Trajectory locateB = new TrajectoryBuilder(new Pose2d(), drive.constraints)
                .splineTo(new Vector2d(70, -10), 0)
                .splineTo(new Vector2d(100, 10), 0)
                .build();

        Trajectory locateC = new TrajectoryBuilder(new Pose2d(), drive.constraints)
                .splineTo(new Vector2d(120, -10), 0)
                .build();

        Trajectory park = new TrajectoryBuilder(new Pose2d(), drive.constraints)
                .splineTo(new Vector2d(70, -10), 0)
                .build();

        super.runOpMode(); //runs OpenCV program and sets rings to the number of rings it sees

        if (isStopRequested()) return;

        telemetry.addData("Rings", rings);
        telemetry.update();

        drive.followTrajectory(moveForward);
        drive.turn(Math.toRadians(45));
        //todo: pauses and other actions are done via markers.
        //sleep(1000);


        drive.turn(Math.toRadians(-45));

        if (rings == GoalDeterminationPipeline.RingPosition.NONE)
            drive.followTrajectory(locateA);
        else if (rings == GoalDeterminationPipeline.RingPosition.ONE)
            drive.followTrajectory(locateB);
        else if (rings == GoalDeterminationPipeline.RingPosition.FOUR)
            drive.followTrajectory(locateC);

        sleep(1000);
        drive.followTrajectory(park);
    }
}

