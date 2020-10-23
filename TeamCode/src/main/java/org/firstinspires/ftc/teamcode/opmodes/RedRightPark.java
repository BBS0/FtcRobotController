package org.firstinspires.ftc.teamcode.opmodes;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.BotBuildersMecBot;

@Config
@Autonomous(name = "RedRightPark", group = "drive")
public class RedRightPark extends LinearOpMode {
    public static double DISTANCE = 70; // in

    @Override
    public void runOpMode() throws InterruptedException {
        BotBuildersMecBot drive = new BotBuildersMecBot(hardwareMap);

        sleep(25000);

        Trajectory trajectory = new TrajectoryBuilder(new Pose2d(), drive.constraints)
                .splineTo(new Vector2d(DISTANCE, -10), 0)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(trajectory);
    }
}