package frc.robot.subsystems.drive

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.geometry.Translation2d
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.trajectory.Trajectory
import frc.robot.subsystems.drive.swerve.Mk2SwerveModule
import frc.robot.subsystems.drive.swerve.SwerveTrajectoryController
import lib.PidController
import lib.mirror
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.twodim.trajectory.mirror
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.utils.BooleanSource
import org.ghrobotics.lib.utils.Source
import org.ghrobotics.lib.utils.map
import java.util.function.BooleanSupplier

/**
 * Follow a trajectory
 * @param trajectorySupplier the trajectory to follow, paired with the end heading of the drivetrain
 */
class SwerveTrajectoryFollowerCommand(val trajectorySupplier: Source<Trajectory>,
                                      val headingSupplier: Source<Rotation2d>) : FalconCommand(DriveSubsystem) {

    constructor(trajectory: Trajectory, targetHeading: Rotation2d, pathMirrored: BooleanSource) : this(
            pathMirrored.map(trajectory.mirror(), trajectory),
            pathMirrored.map(targetHeading.mirror(), targetHeading)
    )

    constructor(trajectory: Trajectory, targetHeading: Rotation2d) : this(
            { trajectory }, { targetHeading }
    )

    private var prevStates = listOf<SwerveModuleState>()
    private lateinit var trajectory: Trajectory
    private lateinit var targetHeading: Rotation2d
    private val timer = Timer()

    private val controller = SwerveTrajectoryController(DriveSubsystem.kinematics, DriveSubsystem.feedForward)

    override fun initialize() {
        trajectory = trajectorySupplier()
        targetHeading = headingSupplier()

        timer.reset()
        timer.start()
        prevStates = listOf(SwerveModuleState(), SwerveModuleState(), SwerveModuleState(), SwerveModuleState())
    }

    override fun execute() {

        // update the controller
        val time = timer.get()
        val state = trajectory.sample(time)
        DriveSubsystem.periodicIO.output = controller.calculate(time, state, targetHeading, DriveSubsystem.periodicIO.pose)

    }
}

fun Rotation2d.toTranslation2d() = Translation2d(this.cos, this.sin)
