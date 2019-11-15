package frc.robot.subsystems.drive

import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
import frc.robot.Controls
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.wrappers.hid.getX
import org.ghrobotics.lib.wrappers.hid.getY

class HolomonicDriveCommand: FalconCommand(DriveSubsystem) {

    override fun execute() {
        val forward = xSource()
        val strafe = zSource()
        val rotation = rotSource()

        val speeds = ChassisSpeeds.fromFieldRelativeSpeeds(forward, strafe, rotation, DriveSubsystem.gyro())

        DriveSubsystem.periodicIO.output = DriveSubsystem.Output.Percent(speeds)

    }

    companion object {
        val xSource by lazy { Controls.driverFalconXbox.getY(GenericHID.Hand.kLeft) }
        val zSource by lazy { Controls.driverFalconXbox.getX(GenericHID.Hand.kLeft) }
        val rotSource by lazy { Controls.driverFalconXbox.getX(GenericHID.Hand.kRight) }
    }
}