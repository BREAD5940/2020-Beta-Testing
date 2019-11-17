package frc.robot.subsystems.drive

import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Translation2d
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics
import frc.robot.Constants
import frc.robot.Controls
import lib.*
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.wrappers.hid.getX
import org.ghrobotics.lib.wrappers.hid.getY
import kotlin.math.absoluteValue
import kotlin.math.sign

class HolomonicDriveCommand : FalconCommand(DriveSubsystem) {

    private var wasEvading = false
    private var clockwiseCenter = Translation2d()
    private var counterClockwiseCenter = Translation2d()

    override fun execute() {
        val forward = xSource()
        val strafe = zSource()
        val rotation = rotSource()


        var translation = Translation2d(forward, strafe) * (12.0 * DriveSubsystem.feedForward.kV.value) // this will have a norm of 1
        val magnitude = translation.norm

        // snap translation power to poles
        if ((translation.toRotation2d().distance(translation.toRotation2d().nearestPole())).radians.absoluteValue
                < Math.toRadians(10.0)) {
            translation = translation.toRotation2d().nearestPole().toTranslation() * magnitude
        }

        // check evasion
        if(evadingButton()) {
            if(!wasEvading) {
                wasEvading = true
                determineEvasionWheels(translation, DriveSubsystem.periodicIO.pose)
            }
            val sign = sign(rotation)
            if (sign > 0.5) {
                centerOfRotation = clockwiseCenter
            } else if (sign < -0.5) {
                centerOfRotation = counterClockwiseCenter
            }
        } else if (wasEvading) centerOfRotation = Translation2d()

        val speeds = ChassisSpeeds.fromFieldRelativeSpeeds(translation.x, translation.y, rotation, DriveSubsystem.periodicIO.pose.rotation)
        val moduleStates = DriveSubsystem.kinematics.toSwerveModuleStates(speeds, centerOfRotation)
        // volts per meter per second times meters per seconds gives volts
        val maxAttainableSpeed = 12.0 -
                moduleStates.map { it.speedMetersPerSecond }.max()!! * DriveSubsystem.feedForward.kV.value
        SwerveDriveKinematics.normalizeWheelSpeeds(moduleStates, maxAttainableSpeed)

        DriveSubsystem.periodicIO.output = DriveSubsystem.Output.KinematicsVelocity(moduleStates.toList())
    }

    private fun determineEvasionWheels(driveVector: Translation2d, robotPosition: Pose2d) {
        val here: Translation2d = driveVector.rotateBy(robotPosition.rotation.inverse())
        val wheels: List<Translation2d> = Constants.kModulePositions
        clockwiseCenter = wheels[0]
        counterClockwiseCenter = wheels[wheels.size - 1]
        for (i in 0 until wheels.size - 1) {
            val cw = wheels[i]
            val ccw = wheels[i + 1]
            if (here.isWithinAngle(cw, ccw)) {
                clockwiseCenter = ccw
                counterClockwiseCenter = cw
            }
        }
    }

    companion object {
        val xSource by lazy { Controls.driverFalconXbox.getY(GenericHID.Hand.kLeft) }
        val zSource by lazy { Controls.driverFalconXbox.getX(GenericHID.Hand.kLeft) }
        val rotSource by lazy { Controls.driverFalconXbox.getX(GenericHID.Hand.kRight) }

        val evadingButton by lazy { Controls.driverFalconXbox.getRawButton(11) } // TODO check

        var centerOfRotation = Translation2d()
    }
}

