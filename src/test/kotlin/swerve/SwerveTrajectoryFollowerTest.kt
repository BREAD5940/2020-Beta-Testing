package swerve

import com.ctre.phoenix.CANifier
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.trajectory.Trajectory
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator
import frc.robot.Constants
import frc.robot.subsystems.drive.swerve.Mk2SwerveModule
import frc.robot.subsystems.drive.swerve.SwerveTrajectoryController
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.trajectory.FalconTrajectoryConfig
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.*
import org.ghrobotics.lib.mathematics.units.feet
import org.ghrobotics.lib.mathematics.units.kFeetToMeter
import org.ghrobotics.lib.physics.MotorCharacterization
import org.ghrobotics.lib.utils.setLEDOutput
import org.junit.Test
import org.knowm.xchart.SwingWrapper
import org.knowm.xchart.XYChartBuilder
import java.awt.Color
import java.awt.Font
import java.text.DecimalFormat
import kotlin.math.absoluteValue

class SwerveTrajectoryFollowerTest {

    @Test fun testTrajectory() {

        val trajectory = TrajectoryGenerator.generateTrajectory(
                listOf(Pose2d(2.feet, 2.feet, 0.degrees), Pose2d(10.feet, 10.feet, 0.degrees)),
                FalconTrajectoryConfig(5.feet.velocity, 5.feet.acceleration)
        )

        // setting up kinematics
        val kinematics = Constants.kinematics
        val odometry = SwerveDriveOdometry(kinematics)
        odometry.resetPosition(Pose2d(4.feet, 10.feet, 45.degrees))
        val controller = SwerveTrajectoryController(kinematics,
                MotorCharacterization(
                        SIUnit(2.25),
                        SIUnit(0.8),
                        SIUnit(0.0)))

        // Sample the trajectory
        val samples = mutableListOf<Trajectory.State>()
        var i = 0.0
        while(i < trajectory.totalTimeSeconds) {
            samples += trajectory.sample(i)
            i += 0.020
        }

        val xList = mutableListOf<Double>()
        val yList = mutableListOf<Double>()
        val refxList = mutableListOf<Double>()
        val refyList = mutableListOf<Double>()

        val targetHeading = Rotation2d()

        var time = 0.0
        for(state in samples) {

            // Add references
            refxList += state.poseMeters.translation.x / kFeetToMeter
            refyList += state.poseMeters.translation.y / kFeetToMeter
            xList += odometry.poseMeters.translation.x / kFeetToMeter
            yList += odometry.poseMeters.translation.y / kFeetToMeter

            // advance controller
            val nextState = controller.calculate(time, state,
                    targetHeading, odometry.poseMeters)

            // update odometry
            val inverse = kinematics.toChassisSpeeds(
                    nextState.flState.toSwerveModuleState(),
                    nextState.frState.toSwerveModuleState(),
                    nextState.blState.toSwerveModuleState(),
                    nextState.brState.toSwerveModuleState()
            )
            val lastAngle = odometry.poseMeters.rotation
            odometry.updateWithTime(time,
                    lastAngle + inverse.omegaRadiansPerSecond.radians.toRotation2d() * 0.020,
                    nextState.flState.toSwerveModuleState(),
                    nextState.frState.toSwerveModuleState(),
                    nextState.blState.toSwerveModuleState(),
                    nextState.brState.toSwerveModuleState()
            )

            time += 0.020
        }

        val fm = DecimalFormat("#.###").format(trajectory.totalTimeSeconds)

        val chart = XYChartBuilder().width(1800).height(1520).title("$fm seconds.")
                .xAxisTitle("X").yAxisTitle("Y").build()

        chart.styler.markerSize = 8
        chart.styler.seriesColors = arrayOf(Color.ORANGE, Color(151, 60, 67))

        chart.styler.chartTitleFont = Font("Kanit", 1, 40)
        chart.styler.chartTitlePadding = 15

        chart.styler.xAxisMin = -1.0
        chart.styler.xAxisMax = refxList.max()!! + 2.0
        chart.styler.yAxisMin = -1.0
        chart.styler.yAxisMax = refyList.max()!! + 2.0

        chart.styler.chartFontColor = Color.WHITE
        chart.styler.axisTickLabelsColor = Color.WHITE

        chart.styler.legendBackgroundColor = Color.GRAY

        chart.styler.isPlotGridLinesVisible = true
        chart.styler.isLegendVisible = true

        chart.styler.plotGridLinesColor = Color.GRAY
        chart.styler.chartBackgroundColor = Color.DARK_GRAY
        chart.styler.plotBackgroundColor = Color.DARK_GRAY

        chart.addSeries("Trajectory", refxList.toDoubleArray(), refyList.toDoubleArray())
        chart.addSeries("Robot", xList.toDoubleArray(), yList.toDoubleArray())

//        SwingWrapper(chart).displayChart()
//        Thread.sleep(1000000)

        assert((refxList.last() - xList.last()).absoluteValue < 0.5)
        assert((refyList.last() - yList.last()).absoluteValue < 0.5)
        val headingError = (targetHeading.degrees - odometry.poseMeters.rotation.degrees).absoluteValue
        assert(headingError < 10.0)

    }

}

private fun Mk2SwerveModule.Output.Velocity.toSwerveModuleState() =
        SwerveModuleState(this.velocity.value, this.angle)
