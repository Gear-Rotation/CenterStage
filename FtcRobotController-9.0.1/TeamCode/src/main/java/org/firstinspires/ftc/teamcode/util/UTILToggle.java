package org.firstinspires.ftc.teamcode.util;

public final class UTILToggle {
    public enum Status
    {
        NOT_BEGUN ,
        IN_PROGRESS ,
        COMPLETE
    }

    private Status _status = Status.NOT_BEGUN;      // Current status of the toggle

    final public Status status(boolean buttonStatus)
    {
        // If the button is being held
        if(buttonStatus && _status == Status.NOT_BEGUN)
            _status = Status.IN_PROGRESS;

            // If the button is not being pressed and the toggle was in progress
        else if(!buttonStatus && _status == Status.IN_PROGRESS)
            _status = Status.COMPLETE;

            // If the toggle is finished
        else if(_status == Status.COMPLETE)
            _status = Status.NOT_BEGUN;

        return _status;
    }
}