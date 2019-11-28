package frc.robot.subsystems.drive

import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
import frc.robot.autonomous.paths.TrajectoryWaypoints
import lib.PidController
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.twodim.geometry.mirror
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.derived.radians
import org.ghrobotics.lib.mathematics.units.derived.toRotation2d
import kotlin.math.PI
import kotlin.math.absoluteValue

class VisionDriveCommand : FalconCommand(DriveSubsystem) {

    lateinit var targetHeading: Rotation2d

    override fun runsWhenDisabled() = true

    private val rotationController = PidController(100.0, 0.0) // rad per sec per radian of error
            .apply {
                setInputRange(0.0, 2.0 * PI)
                setContinuous(true)
                setOutputRange(-0.5, 0.5)
            }

    override fun initialize() {
        targetHeading = (importantAngles.minBy { (it - DriveSubsystem.robotPosition.rotation.radians).absoluteValue })!!.radians.toRotation2d()
        rotationController.setSetpoint(targetHeading.radians)
    }

    override fun execute() {
        val turn = rotationController.calculate(DriveSubsystem.robotPosition.rotation.radians, 0.020);
        DriveSubsystem.periodicIO.output = DriveSubsystem.Output.Velocity(
                ChassisSpeeds(0.0, 0.0, turn)
        )
        println("Target Heading ${targetHeading.degrees} turn $turn")
    }

    companion object {

        val importantAngles = listOf(
                // rocket n, bay and f (mirrored and not)
                TrajectoryWaypoints.kRocketN.rotation,
                -TrajectoryWaypoints.kRocketN.rotation,
                TrajectoryWaypoints.kRocketF.rotation,
                -TrajectoryWaypoints.kRocketF.rotation,
                TrajectoryWaypoints.kRocketBay.rotation,
                -TrajectoryWaypoints.kRocketBay.rotation,
                TrajectoryWaypoints.kCargoShipFL.rotation,
                -TrajectoryWaypoints.kCargoShipFL.rotation,
                TrajectoryWaypoints.kCargoShipS1.rotation,
                -TrajectoryWaypoints.kCargoShipS1.rotation
        ).map { it.radians }

    }

}