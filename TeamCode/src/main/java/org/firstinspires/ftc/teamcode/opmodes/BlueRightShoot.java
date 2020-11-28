package org.firstinspires.ftc.teamcode.opmodes;

//import com.acmerobotics.dashboard.config.Config;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.BotBuildersMecBot;
//import org.firstinspires.ftc.teamcode.rr_quickstart_examples.drive.SampleMecanumDrive;

//@Config
//@Autonomous(name = "BlueRightShoot", group = "drive")
public class BlueRightShoot extends LinearOpMode {
    public static double DISTANCE = 60; // in

    @Override
    public void runOpMode() throws InterruptedException {
        BotBuildersMecBot drive = new BotBuildersMecBot(hardwareMap);

        Trajectory moveToLine = new TrajectoryBuilder(new Pose2d(), drive.constraints)
                .splineTo(new Vector2d(60, -10), 0)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(moveToLine);
    }
}
