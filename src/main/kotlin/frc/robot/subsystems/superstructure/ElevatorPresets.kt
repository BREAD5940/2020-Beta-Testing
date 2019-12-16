package frc.robot.subsystems.superstructure
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.inches
import org.ghrobotics.lib.mathematics.units.Meter
import org.ghrobotics.lib.mathematics.units.SIUnit
//TODO check no motors are fighting
object ElevatorPresets {
    class elevatorGoToPreset(val height : SIUnit <Meter>) {
        //TODO Change to real presets
         fun initialize() {
            //super.initialize()
            Elevator.master.setPosition(height)
        }
         fun isFinished(): Boolean {
            return (Elevator.master.encoder.position -height).absoluteValue < 0.5.inches
        }
    }
}