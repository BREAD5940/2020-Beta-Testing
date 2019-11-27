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
import org.ghrobotics.lib.mathematics.units.inFeet
import org.ghrobotics.lib.mathematics.units.meters
import org.ghrobotics.lib.utils.withDeadband
import org.ghrobotics.lib.wrappers.hid.getX
import org.ghrobotics.lib.wrappers.hid.getY
import kotlin.math.absoluteValue
import kotlin.math.sign

class HolomonicDriveCommand : FalconCommand(DriveSubsystem) {

    private var wasEvading = false
    private var clockwiseCenter = Translation2d()
    private var counterClockwiseCenter = Translation2d()

    override fun runsWhenDisabled(): Boolean {
        return true
    }

    override fun execute() {
        val forward = -xSource()
        val strafe = -zSource()
        val rotation = -rotSource() * 6.0

        println("commanding forward $forward strafe $strafe rotation $rotation")

        // calculate translation vector (with magnitude of the max speed
        // volts divided by volts per meter per second is meters per second
        println("Scaler ${(12.0 / DriveSubsystem.feedForward.kV.value)}")
        var translation = Translation2d(forward, strafe) * (12.0 / DriveSubsystem.feedForward.kV.value) // this will have a norm of 1
        val magnitude = translation.norm

        println("chassis velocity: ${magnitude.meters.inFeet()}")

        // snap translation power to poles if we're close to them
        if ((translation.toRotation2d().distance(translation.toRotation2d().nearestPole())).radians.absoluteValue
                < Math.toRadians(10.0)) {
            translation = translation.toRotation2d().nearestPole().toTranslation() * magnitude
        }

        // check evasion and determine wheels if necessary
        val wantsEvasion = evadingButton()
        if(wantsEvasion) {
            if(!wasEvading) { // determine evasion wheels if we weren't previously evading. Doing this twice is a Bad Idea
                wasEvading = true
                determineEvasionWheels(translation, DriveSubsystem.periodicIO.pose)
            }
            val sign = sign(rotation)
            if (sign > 0.5) {
                centerOfRotation = clockwiseCenter
            } else if (sign < -0.5) {
                centerOfRotation = counterClockwiseCenter
            }
        } else  { // we don't want evasion
            if (wasEvading) { // toggle out if we were previously evading
                centerOfRotation = Translation2d()
                wasEvading = false
            }
        }

        // calculate wheel speeds from field oriented chassis state
        val speeds = ChassisSpeeds.fromFieldRelativeSpeeds(translation.x, translation.y, rotation, DriveSubsystem.periodicIO.pose.rotation)
        val moduleStates = DriveSubsystem.kinematics.toSwerveModuleStates(speeds, centerOfRotation)

        // Normalize wheel speeds
        // volts per meter per second times meters per seconds gives volts
        SwerveDriveKinematics.normalizeWheelSpeeds(moduleStates,
                12.0 / DriveSubsystem.feedForward.kV.value)

        DriveSubsystem.periodicIO.output = DriveSubsystem.Output.KinematicsVelocity(moduleStates.toList())
    }

    /** Determine which wheels to use to evade. */
    private fun determineEvasionWheels(driveVector: Translation2d, robotPosition: Pose2d) {
        val here: Translation2d = driveVector.rotateBy(robotPosition.rotation.inverse())
        val wheels = Constants.kModulePositions
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
        val xSource by lazy { Controls.driverFalconXbox.getY(GenericHID.Hand.kLeft).withDeadband(0.02) }
        val zSource by lazy { Controls.driverFalconXbox.getX(GenericHID.Hand.kLeft).withDeadband(0.02) }
        val rotSource by lazy { Controls.driverFalconXbox.getX(GenericHID.Hand.kRight).withDeadband(0.06) }

        val evadingButton by lazy { Controls.driverFalconXbox.getRawButton(10) } // TODO check

        var centerOfRotation = Translation2d()
    }
}

