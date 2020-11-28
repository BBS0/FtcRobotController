package org.firstinspires.ftc.teamcode.tests;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.robot.BotBuildersMecBot;

@Autonomous(name = "Sample", group = "drive")
@Disabled
  public class SampleOpMode extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException{

        //any setup...
       BotBuildersMecBot robot = new BotBuildersMecBot(hardwareMap);

        Trajectory moveForward = robot.trajectoryBuilder(new Pose2d(10, 10))
                .forward(20)
                .build();

        Trajectory splineTest = robot.trajectoryBuilder((moveForward.end()))
                .splineTo(new Vector2d(50, 50), 0)
                .build();
        telemetry.addData("Ready", "");


        waitForStart();
    }
}
