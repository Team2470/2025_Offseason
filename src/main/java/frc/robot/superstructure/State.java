package frc.robot.superstructure;

public interface State {
    void onEnter();
    void onExit();
    void periodic(); // called every robot loop
    boolean isFinished(); // signals ready to transition
    State nextState(); // return next state (or null if finished)
}
