package frc.robot.auto

import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.robot.Network
import frc.robot.Robot
import frc.robot.subsystems.drive.DriveSubsystem
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.utils.Source
import org.ghrobotics.lib.utils.and
import org.ghrobotics.lib.utils.monitor
import org.ghrobotics.lib.utils.onChangeToTrue
import org.ghrobotics.lib.wrappers.FalconTimedRobot

/**
 * Manages the autonomous mode of the game.
 */
object Autonomous {

    // Auto mode to run
    private val autoMode = { Network.autoModeChooser.selected }

    // Starting position of the robot
    val startingPosition = { Network.startingPositionChooser.selected }

    // Stores whether the current config is valid.
    private var configValid = Source(true)

    val isStartingOnLeft = { val position = startingPosition()
        position == StartingPositions.LEFT ||
                position == StartingPositions.LEFT_REVERSED
    }

    // Stores if we are ready to send it.
    private val isReady =
            { Robot.currentMode == FalconTimedRobot.Mode.AUTONOMOUS && Robot.isEnabled } and configValid

    // Update the autonomous listener.
    fun update() {
        // Update localization.
//        startingPositionMonitor.onChange { if (!Robot.isEnabled) DriveSubsystem.odometry.resetPosition(it.pose, DriveSubsystem.gyro()) }

        // update our selected auto mode
        selectedAutonomous = possibleAutos[autoMode()] ?: doNothing

        @Suppress("UNUSED_ANONYMOUS_PARAMETER")
        robotModeMonitor.onChange { newValue ->
            // maybe stop auto on change to enabled?
        }

        isReadyMonitor.onChangeToTrue {
            startAuto()
        }
    }

    val possibleAutos = hashMapOf(
            Mode.DO_NOTHING to InstantCommand()
    )
    var selectedAutonomous: CommandBase = InstantCommand()
    val doNothing = selectedAutonomous

    fun startAuto() {
        selectedAutonomous.schedule()
    }

    @Suppress("LocalVariableName")
    private val IT = ""

    private val startingPositionMonitor = startingPosition.monitor
    private val isReadyMonitor = isReady.monitor
    private val robotModeMonitor = { Robot.currentMode }.monitor

    enum class StartingPositions(val pose: Pose2d) {
        LEFT(Pose2d()), // TODO poses
        CENTER(Pose2d()),
        RIGHT(Pose2d()),
        LEFT_REVERSED(Pose2d()),
        RIGHT_REVERSED(Pose2d())
    }

    enum class Mode { BOTTOM_ROCKET_2, FORWARD_CARGO_SHIP, SIDE_CARGO_SHIP, HYBRID, DO_NOTHING }
}
