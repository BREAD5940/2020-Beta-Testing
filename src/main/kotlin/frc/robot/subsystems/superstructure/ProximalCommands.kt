package frc.robot.subsystems.superstructure

import edu.wpi.first.wpilibj.controller.ArmFeedforward
import edu.wpi.first.wpilibj.controller.PIDController
import edu.wpi.first.wpilibj.controller.ProfiledPIDController
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.*

class ClosedLoopProximalMove(val targetPosition: SIUnit<Radian> = 0.radians): FalconCommand(Proximal) {

    private val controller = ProfiledPIDController(25.0, 0.0, 0.48, constraints)

    companion object {
        private val feedforward = ArmFeedforward(0.3, 0.3, 3.9, 0.08)
        private val constraints = TrapezoidProfile.Constraints(30.degrees.inRadians(), 60.degrees.inRadians())
    }

    override fun initialize() {
        controller.setGoal(targetPosition.inRadians())
    }

    override fun execute() {
        val position = Proximal.position
        Proximal.voltageOutput = controller.calculate(position.inRadians(), targetPosition.inRadians()).volts
            + feedforward.calculate(Proximal.position.inRadians(), controller.setpoint.velocity)
    }

    override fun isFinished() = controller.atGoal()

}