package frc.robot.subsystems.drive

import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Translation2d
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics
import frc.robot.Constants
import frc.robot.Controls
import kotlin.math.absoluteValue
import kotlin.math.sign
import lib.* // ktlint-disable no-wildcard-imports
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.utils.withDeadband
import org.ghrobotics.lib.wrappers.hid.getX
import org.ghrobotics.lib.wrappers.hid.getY

class HolomonicDriveCommand : FalconCommand(DriveSubsystem) {

    private var wasEvading = false
    private var clockwiseCenter = Translation2d()
    private var counterClockwiseCenter = Translation2d()

    override fun execute() {
        val forward = -xSource() / 1.0
        val strafe = -zSource() / 1.0
        val rotation = -rotSource() * 2.0 / 1.0

        // calculate translation vector (with magnitude of the max speed
        // volts divided by volts per meter per second is meters per second
        val translation = Translation2d(forward, strafe) // this will have a norm of 1, or 100% power

        if(forward.absoluteValue < 0.01 && strafe.absoluteValue < 0.01 && rotation.absoluteValue < 0.01) {
            DriveSubsystem.periodicIO.output = SwerveDriveOutput.Nothing
            return
        }

        // calculate wheel speeds from field oriented chassis state
        val speeds = ChassisSpeeds.fromFieldRelativeSpeeds(translation.x, translation.y, rotation, DriveSubsystem.periodicIO.pose.rotation)
//        val moduleStates = DriveSubsystem.kinematics.toSwerveModuleStates(speeds, centerOfRotation)

        // Normalize wheel speeds
        // Max duty cycle is 1.0
//        SwerveDriveKinematics.normalizeWheelSpeeds(moduleStates,1.0)

        println("speeds ${speeds.toString2()}")

        DriveSubsystem.periodicIO.output = SwerveDriveOutput.Percent(speeds)
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
        val xSource by lazy { Controls.driverFalconXbox.getY(GenericHID.Hand.kRight).withDeadband(0.1) }
        val zSource by lazy { Controls.driverFalconXbox.getX(GenericHID.Hand.kRight).withDeadband(0.1) }
        val rotSource by lazy { Controls.driverFalconXbox.getX(GenericHID.Hand.kLeft).withDeadband(0.06) }

        val evadingButton by lazy { Controls.driverFalconXbox.getRawButton(11) } // TODO check

        var centerOfRotation = Translation2d()
    }
}

private fun ChassisSpeeds.toString2(): String {
    return "Speeds vx $vxMetersPerSecond vy $vyMetersPerSecond omega $omegaRadiansPerSecond"
}
