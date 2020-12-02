package org.firstinspires.ftc.teamcode.opmodes;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.BotBuildersMecBot;

@Autonomous(name = "BlueLeftWobbleGoal", group = "drive")
public class BlueLeftWobbleGoal extends LinearOpMode {

    public static double DISTANCE = 60;


    @Override
    public void runOpMode() throws InterruptedException {
        BotBuildersMecBot drive =  new BotBuildersMecBot(hardwareMap);
        drive.opMode = this;


        //moveForward Variable
        Trajectory moveForward = new TrajectoryBuilder(new Pose2d(), drive.constraints)
                .forward(115)
                .build();


        //inchBack Variable
        Trajectory inchBack = new TrajectoryBuilder(moveForward.end(), drive.constraints)
                .back(5)
                .build();

        //moveRight Variable
        Trajectory moveRight = new TrajectoryBuilder(moveForward.end(), drive.constraints)
                .strafeRight(20)
                .build();

        //moveBack Variable
        Trajectory moveBack = new TrajectoryBuilder(moveRight.end(), drive.constraints)
                .back(35)
                .build();

        drive.followTrajectory(moveForward);
        drive.followTrajectory(inchBack);
        drive.followTrajectory(moveRight);
        drive.followTrajectory(moveBack);
        waitForStart();

        if (isStopRequested()) return;

        sleep(1000);
    }


}
