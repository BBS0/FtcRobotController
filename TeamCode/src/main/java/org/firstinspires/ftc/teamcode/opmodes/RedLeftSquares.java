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


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.BotBuildersMecBot;
import org.firstinspires.ftc.teamcode.robot.OpenCvDetector;

import static android.icu.lang.UCharacter.DecompositionType.SQUARE;

/*
 * This is a simple routine to test translational drive capabilities.
 */
//@Config
@Autonomous(name = "RedLeftSquares", group = "drive")
public class RedLeftSquares extends OpenCvDetector {
    public static double DISTANCE = 60; // in
    public int SQUARE;

    @Override
    public void runOpMode() throws InterruptedException {
        BotBuildersMecBot drive = new BotBuildersMecBot(hardwareMap);

        Trajectory moveForward = new TrajectoryBuilder(new Pose2d(), drive.constraints)
                .forward(20)
                .build();

        Trajectory locateA = new TrajectoryBuilder(new Pose2d(), drive.constraints)
                .splineTo(new Vector2d(70, -35), 0)
                .build();

        Trajectory locateB = new TrajectoryBuilder(new Pose2d(), drive.constraints)
                .splineTo(new Vector2d(40, 10), 0)
                .splineTo(new Vector2d(100, -15), 0)
                .build();

        Trajectory locateC = new TrajectoryBuilder(new Pose2d(), drive.constraints)
                .splineTo(new Vector2d(40,10), 0)
                .splineTo(new Vector2d(120, -35), 0)
                .build();

        Trajectory parkFromB = new TrajectoryBuilder(locateB.end(), drive.constraints)
                .splineTo(new Vector2d(70, -35), 0)
                .build();

        Trajectory parkFromC = new TrajectoryBuilder(locateC.end(), drive.constraints)
                .splineTo(new Vector2d(70, -35), 0)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        //drive.followTrajectory(moveForward);
        //drive.turn(Math.toRadians(-45));

        super.runOpMode(); //runs OpenCV program and sets rings to the number of rings it sees

        //drive.turn(Math.toRadians(45));

        //Pose2d endpose = new Pose2d();

        if (rings == GoalDeterminationPipeline.RingPosition.NONE) {
            drive.followTrajectory(locateA);
        } else if (rings == GoalDeterminationPipeline.RingPosition.ONE) {
            drive.followTrajectory(locateB);
            sleep(1000);
            drive.followTrajectory(parkFromB);
        } else if (rings == GoalDeterminationPipeline.RingPosition.FOUR) {
            drive.followTrajectory(locateC);
            sleep(1000);
            drive.followTrajectory(parkFromC);
        }
    }
}

