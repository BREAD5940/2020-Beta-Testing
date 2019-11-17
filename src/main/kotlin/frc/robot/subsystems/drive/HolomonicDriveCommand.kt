package frc.robot.subsystems.drive

import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.geometry.Translation2d
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics
import frc.robot.Controls
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.wrappers.hid.getX
import org.ghrobotics.lib.wrappers.hid.getY
import kotlin.math.abs
import kotlin.math.absoluteValue
import kotlin.math.hypot
import kotlin.math.sign

class HolomonicDriveCommand: FalconCommand(DriveSubsystem) {

    override fun execute() {
        val forward = xSource()
        val strafe = zSource()
        val rotation = rotSource()

        var translation = Translation2d(forward, strafe)
        val magnitude = translation.norm

        // snap translation power to poles
        if((translation.toRotation2d().distance(translation.toRotation2d().nearestPole())).radians.absoluteValue
                < Math.toRadians(10.0)) {
            translation = translation.toRotation2d().nearestPole().toTranslation() * magnitude
        }

        val speeds = ChassisSpeeds.fromFieldRelativeSpeeds(translation.x, translation.y, rotation, DriveSubsystem.gyro())
        val moduleStates = DriveSubsystem.kinematics.toSwerveModuleStates(speeds, centerOfRotation)
        // volts per meter per second times meters per seconds gives volts
        val maxAttainableSpeed = 12.0 -
                moduleStates.map { it.speedMetersPerSecond }.max()!! * DriveSubsystem.feedForward.kV.value
        SwerveDriveKinematics.normalizeWheelSpeeds(moduleStates, maxAttainableSpeed)

        DriveSubsystem.periodicIO.output = DriveSubsystem.Output.KinematicsVelocity(moduleStates.toList())

    }

    companion object {
        val xSource by lazy { Controls.driverFalconXbox.getY(GenericHID.Hand.kLeft) }
        val zSource by lazy { Controls.driverFalconXbox.getX(GenericHID.Hand.kLeft) }
        val rotSource by lazy { Controls.driverFalconXbox.getX(GenericHID.Hand.kRight) }

        var centerOfRotation = Translation2d()
    }
}

private fun Rotation2d.toTranslation() = Translation2d(this.cos, this.sin)

fun Translation2d.toRotation2d(): Rotation2d {
    var x = this.x
    var y = this.y
    val hypo = hypot(x, y)
    x /= hypo
    y /= hypo
    return Rotation2d(x, y)
}

fun Rotation2d.nearestPole(): Rotation2d {
    var pole_sin = 0.0
    var pole_cos = 0.0
    if (abs(cos) > abs(sin)) {
        pole_cos = sign(cos)
        pole_sin = 0.0
    } else {
        pole_cos = 0.0
        pole_sin = sign(sin)
    }
    return Rotation2d(pole_cos, pole_sin)
}

fun Rotation2d.distance(other: Rotation2d): Rotation2d {
    val inverse = Rotation2d(cos, -sin)
    return inverse + other
}