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
@Autonomous(name = "BlueRightPark", group = "drive")
public class BlueRightPark extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        BotBuildersMecBot drive = new BotBuildersMecBot(hardwareMap);
        drive.opMode = this;
        drive.AutoLockGoal();

        //sleep(25000);

        Trajectory trajectory = new TrajectoryBuilder(new Pose2d(), drive.constraints)
                .splineTo(new Vector2d(80, -10), 0)
                .build();

        Trajectory strafe = new TrajectoryBuilder(new Pose2d(), drive.constraints)
                .strafeLeft(15)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(trajectory);

        sleep(500);
        drive.turn(Math.toRadians(105));
        sleep(100);
        drive.AutoArmPickUp();
        sleep(100);
        drive.AutoArmDropOff();
        drive.followTrajectory(strafe);
        sleep(500);
        drive.turn(Math.toRadians(-105));
    }
}