/*
This is a modified version of StraightTest.java from the ACME Robotics Road-Runner-Quickstart project.
Included with permission from ACME Robotics.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
 */

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

/*
 * This is a simple routine to test translational drive capabilities.
 */
//@Config
//@Autonomous(name = "BlueRightSquares", group = "drive")
public class BlueRightSquares extends LinearOpMode {
    public static double DISTANCE = 60; // in
    public static int SQUARE = 3;

    @Override
    public void runOpMode() throws InterruptedException {
        BotBuildersMecBot drive = new BotBuildersMecBot(hardwareMap);

        Trajectory moveForward = new TrajectoryBuilder(new Pose2d(), drive.constraints)
                .forward(20)
                .build();

        Trajectory locateA = new TrajectoryBuilder(new Pose2d(), drive.constraints)
                .splineTo(new Vector2d(70, 35), 0)
                .build();

        Trajectory locateB = new TrajectoryBuilder(new Pose2d(), drive.constraints)
                .splineTo(new Vector2d(40, -10), 0)
                .splineTo(new Vector2d(100, 10), 0)
                .build();

        Trajectory locateC = new TrajectoryBuilder(new Pose2d(), drive.constraints)
                .splineTo(new Vector2d(40, -10), 0)
                .splineTo(new Vector2d(120, 35), 0)
                .build();

        Trajectory park = new TrajectoryBuilder(new Pose2d(), drive.constraints)
                .splineTo(new Vector2d(70, 35), 0)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(moveForward);
        drive.turn(Math.toRadians(45));
        sleep(1000);

        // add in the camera code here

        drive.turn(Math.toRadians(-45));

        if (SQUARE == 1)
            drive.followTrajectory(locateA);
        else if (SQUARE == 2)
            drive.followTrajectory(locateB);
        else if (SQUARE == 3)
            drive.followTrajectory(locateC);

        sleep(1000);
        drive.followTrajectory(park);
    }
}

