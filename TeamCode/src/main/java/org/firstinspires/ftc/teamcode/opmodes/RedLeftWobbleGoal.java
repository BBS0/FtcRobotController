package org.firstinspires.ftc.teamcode.opmodes;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.BotBuildersMecBot;

@Autonomous(name = "RedLeftWobbleGoal", group = "drive")
public class RedLeftWobbleGoal extends LinearOpMode {

    public static double DISTANCE = 60;


    @Override
    public void runOpMode() throws InterruptedException {
        BotBuildersMecBot drive =  new BotBuildersMecBot(hardwareMap);
        drive.opMode = this;


        //moveForward Variable
        Trajectory moveForward = new TrajectoryBuilder(new Pose2d(), drive.constraints)
                .forward(100)
                .build();

        //moveBack Variable
        Trajectory moveBack = new TrajectoryBuilder(moveForward.end(), drive.constraints)
                .back(25)
                .build();

        drive.followTrajectory(moveForward);
        drive.followTrajectory(moveBack);
        waitForStart();

        if (isStopRequested()) return;

        sleep(1000);
    }


}