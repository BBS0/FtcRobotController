package org.firstinspires.ftc.teamcode.opmodes;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robot.BotBuildersMecBot;
import org.firstinspires.ftc.teamcode.robot.OpenCvDetector;

@Autonomous(name = "StraferAuto", group = "drive")
public class StrafeAuto extends OpenCvDetector {

    @Override
    public void runOpMode() throws InterruptedException {
        BotBuildersMecBot drive = new BotBuildersMecBot(hardwareMap);
        drive.opMode = this;

        //drive a little bit forward
        Trajectory forwardOntoLine = new TrajectoryBuilder(new Pose2d(), drive.constraints)
                .forward(10)
                .build();

        Trajectory moveForward = new TrajectoryBuilder(new Pose2d(), drive.constraints)
                .forward(45)
                .build();

        Trajectory movesmallForward = new TrajectoryBuilder(new Pose2d(), drive.constraints)
                .forward(10)
                .build();

        Trajectory strafeLeft1 = new TrajectoryBuilder(new Pose2d(), drive.constraints)
                .strafeLeft(35)
                .build();


        Trajectory strafeRight1 = new TrajectoryBuilder(new Pose2d(), drive.constraints)
                .strafeRight(20)
                .build();

        Trajectory strafeRight2 = new TrajectoryBuilder(new Pose2d(), drive.constraints)
                .strafeRight(15)
                .build();

        Trajectory strafeAwayLeft = new TrajectoryBuilder(new Pose2d(), drive.constraints)
                .strafeLeft(15)
                .build();

        Trajectory locateB = new TrajectoryBuilder(new Pose2d(), drive.constraints)
                .splineTo(new Vector2d(35, -5), 0)
                .build();

        Trajectory locateC = new TrajectoryBuilder(new Pose2d(), drive.constraints)
                .splineTo(new Vector2d(53, -25), 0)
                .build();

        Trajectory strafeRightToWall = new TrajectoryBuilder(new Pose2d(), drive.constraints)
                .strafeRight(43)
                .build();

        Trajectory revOne = new TrajectoryBuilder(new Pose2d(), drive.constraints)
                .back(15)
                .build();

        Trajectory revC = new TrajectoryBuilder(new Pose2d(), drive.constraints)
                .back(35)
                .build();
        drive.LockArm();


        drive.AutoLockGoal();
        super.runOpMode(); //runs OpenCV program and sets rings to the number of rings it sees

        if (isStopRequested()) return;

        drive.AutoArmPickUp();


        Trajectory pickedTraj = null;


        telemetry.addData("Rings", rings);
        telemetry.update();

        drive.followTrajectory(movesmallForward);
        drive.followTrajectory(strafeLeft1);
        drive.followTrajectory(moveForward);
        drive.turn(Math.toRadians(20));

        drive.TurnJHopShooterOn(205); //was 215
        drive.FullAutoShoot();

        drive.followTrajectory(strafeRight1);

        sleep(200);
        drive.turn(Math.toRadians(-10));
        drive.FullAutoShoot();

        drive.followTrajectory(strafeRight2);

        drive.turn(Math.toRadians(-10));
        sleep(200);
        drive.FullAutoShoot();
        drive.TurnJHopShooterOff();

        if (rings == GoalDeterminationPipeline.RingPosition.NONE) {


            drive.followTrajectory(forwardOntoLine);

            drive.followTrajectory(strafeRightToWall);

            drive.turn(Math.toRadians(10));
            drive.AutoArmDropOff();

            //wait for the servo to disengage
            sleep(1000);

            drive.followTrajectory(strafeAwayLeft);

            drive.AutoArmTop();

            drive.LockArm();
            sleep(1000);
        }
        else if (rings == GoalDeterminationPipeline.RingPosition.ONE) {
            drive.followTrajectory(locateB);
            pickedTraj = locateB;

        }
        else if (rings == GoalDeterminationPipeline.RingPosition.FOUR) {
            drive.followTrajectory(locateC);
            pickedTraj = locateC;
        }



        if(pickedTraj != null) {
            if( rings == GoalDeterminationPipeline.RingPosition.FOUR) {

                drive.turn(Math.toRadians(25));
                drive.AutoArmDropOff();

                //wait for the servo to disengage
                sleep(550);

                drive.followTrajectory(strafeAwayLeft);

                drive.AutoArmTop();

                sleep(500);

                drive.turn(Math.toRadians(-25));

                drive.LockArm();
                drive.followTrajectory(revC);


            }else if( rings == GoalDeterminationPipeline.RingPosition.ONE){

                drive.AutoArmDropOff();

                //wait for the servo to disengage
                sleep(1200);

                drive.followTrajectory(strafeAwayLeft);

                drive.AutoArmTop();

                sleep(500);


                drive.LockArm();
                drive.followTrajectory(revOne);


            }
        }
    }
}