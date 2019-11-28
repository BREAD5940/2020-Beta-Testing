package frc.robot.subsystems.drive

import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
import frc.robot.Constants
import frc.robot.autonomous.paths.TrajectoryWaypoints
import frc.robot.autonomous.paths.transformBy
import kotlin.math.PI
import kotlin.math.absoluteValue
import lib.PidController
import lib.normalize
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.derived.radians
import org.ghrobotics.lib.mathematics.units.derived.toRotation2d
import org.ghrobotics.lib.mathematics.units.feet
import org.ghrobotics.lib.mathematics.units.inMeters

class VisionDriveCommand : FalconCommand(DriveSubsystem) {

    lateinit var targetHeading: Rotation2d

    override fun runsWhenDisabled() = true

    private val rotationController = PidController(10.0, 0.0) // rad per sec per radian of error
            .apply {
                setInputRange(0.0, 2.0 * PI)
                setContinuous(true)
                setOutputRange(-2 * PI, 2 * PI)
            }
    private val translationController = PidController(100.0, 0.0)
            .apply {
                setOutputRange(-4.feet.inMeters(), 4.feet.inMeters())
            }

    override fun initialize() {
        targetHeading = (importantAngles.minBy { (it - DriveSubsystem.robotPosition.rotation.radians).absoluteValue })!!.radians.toRotation2d()
        rotationController.setSetpoint(targetHeading.radians)
        translationController.setSetpoint(0.0)
    }

    override fun execute() {
        val turn = rotationController.calculate(DriveSubsystem.robotPosition.rotation.radians, 0.020)

        val targetPose = TrajectoryWaypoints.kRocketN.transformBy(Constants.kIntakeToCenter) // TODO get from limelight
        val currentPose = DriveSubsystem.robotPosition
        // we only care about the translational error, so
        var error = (targetPose.translation - currentPose.translation)

        println("target $targetPose current $currentPose error $error")

        val targetVelocity = translationController.calculate(error.norm, 0.020)
        error = error.normalize()
        val vX = -error.x * targetVelocity
        val vY = -error.y * targetVelocity

        DriveSubsystem.periodicIO.output = SwerveDriveOutput.Velocity(
                ChassisSpeeds.fromFieldRelativeSpeeds(vX, vY, turn, DriveSubsystem.robotPosition.rotation)
        )
//        println("Target Heading ${targetHeading.degrees} turn $turn")
    }

    companion object {

        val importantAngles = listOf(
                // rocket n, bay and f (mirrored and not)
                TrajectoryWaypoints.kRocketN.rotation // ,
//                -TrajectoryWaypoints.kRocketN.rotation,
//                TrajectoryWaypoints.kRocketF.rotation,
//                -TrajectoryWaypoints.kRocketF.rotation,
//                TrajectoryWaypoints.kRocketBay.rotation,
//                -TrajectoryWaypoints.kRocketBay.rotation,
//                TrajectoryWaypoints.kCargoShipFL.rotation,
//                -TrajectoryWaypoints.kCargoShipFL.rotation,
//                TrajectoryWaypoints.kCargoShipS1.rotation,
//                -TrajectoryWaypoints.kCargoShipS1.rotation
        ).map { it.radians }
    }
}
