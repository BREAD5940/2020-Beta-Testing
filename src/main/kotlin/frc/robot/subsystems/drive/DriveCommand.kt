package frc.robot.subsystems.drive

import edu.wpi.first.wpilibj.GenericHID
import frc.robot.Controls
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.utils.withDeadband
import org.ghrobotics.lib.wrappers.hid.getX
import org.ghrobotics.lib.wrappers.hid.getY

class DriveCommand {
    fun isFinished() = false
    fun execute() {
        // negative because xbox is negative Y when pushed forward
        val forward = speedSource() // same as -1 * speedSource.invoke()
        val turn = rotationSource()
        val isQuickTurn = false
        DriveSubsystem.curvatureDrive(forward, turn, isQuickTurn)
    }

    fun end(interrupted: Boolean) {
        DriveSubsystem.leftMotor.setNeutral()
        DriveSubsystem.rightMotor.setNeutral()
    }

    companion object {

        private const val kDeadband = 0.05
        //TODO check this bit I (max) think I fixed it
        //val speedSource by lazy { Controls.driverFalconXbox.getY(GenericHID.Hand.kLeft).withDeadband(kDeadband) }
        val speedSource by lazy { Controls.XboxController().driverFalconXbox.getY(GenericHID.Hand.kLeft).withDeadband(kDeadband) }
        val rotationSource by lazy { Controls.XboxController().driverFalconXbox.getX(GenericHID.Hand.kRight).withDeadband(kDeadband) } }
}
