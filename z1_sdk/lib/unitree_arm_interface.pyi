from typing import Any, List, Tuple, Union, overload
from enum import Enum
import numpy as np
import numpy.typing as npt

class Error(Enum):
    joint_position_limits_violation: ...
    """True if the robot exceeeded joint velocity limits."""
    joint_velocity_violation: ...
    """True if the robot exceeded joint velocity limits."""
    colision_detected: ...
    """True if a collision was detected."""
    repeat_file_invalid: ...
    """True if the trajectory file is not avaliable when TeachRepeat"""

class MotorError(Enum):
    disconnection: ...
    """True if the motor has lost connection."""
    phase_current_large: ...
    """True if the motor phase current is too large."""
    phase_leakage: ...
    """True if the motor has phase leakage."""
    over_temperature: ...
    """True if the motor temperature is larger than 80 degrees centigrade."""
    wind_overheat: ...
    """True if the motor winds overheat."""
    parameters_jumped: ...
    """True if the motor parameters jumped."""

class ArmMode(Enum):
    Invalid: ...
    Passive: ...
    LowCmd: ...
    JointSpeedCtrl: ...
    JointPositionCtrl: ...
    Teach: ...
    TeachRepeat: ...
    Calibration: ...
    ClearError: ...

class GripperCmd:
    """This class models the command sent to the gripper."""

    angle: float
    """Target gripper angle. [-1, 0], the negative direction indicates opening."""

    speed: float
    """Closing speed in rad/s. Must be a positive value."""

    maxTau: float
    """Grasping max tau in Nm. Must be a positive value."""

    epsilon_inner: float
    """Maximum tolerated deviation when the actual grasped angle is smaller than the commanded grasp angle."""

    epsilon_outer: float
    """Maximum tolerated deviation when the actual grasped angle is larger than the commanded grasp angle."""

class GripperState:
    """This class models the state of the gripper."""

    angle: float
    """Measured gripper angle. Unit: rad"""

    speed: float
    """Measured gripper speed. Unit: rad/s"""

    tau: float
    """Measured gripper output torque. Unit: Nm"""

    reached_goal: bool
    """True if the gripper reaches target angle."""

    stalled: bool
    """True if the gripper doesn't move."""

    exist: bool
    """True if the z1 robot has UnitreeGripper."""

class ArmCmd:
    """Class representing arm command structure."""

    version: List[int]
    """3 elements, The z1 controller version."""

    mode: int
    """Desired running state.
    See: ArmMode
    """

    mass_ee: float
    """Configured mass of the end effector."""

    com_ee: List[float]
    """3 elements, Configured center of mass of the end effector."""

    inertia_ee: List[float]
    """9 elements, Configured rotational inertia matrix of the end effector load with respect to center of mass."""

    F_T_EE: List[float]
    """16 elements, Configured the end effector in flange frame.
    Pose is represented as a 4x4 matrix in column-major format.
    """

    mass_load: float
    """Configured mass of the external load."""

    com_load: List[float]
    """3 elements, Configured center of mass of the external load with respect to flange frame."""

    inertia_load: List[float]
    """9 elements, Configured inertia matrix of the external load with respect to center of mass."""

    gripperCmd: "GripperCmd"
    """Desired gripper command."""

    Kp: List[float]
    """6 elements, Joint position gains."""

    Kd: List[float]
    """6 elements, Joint velocity gains."""

    q_d: List[float]
    """6 elements, Desired joint position. Unit: rad"""

    dq_d: List[float]
    """6 elements, Desired joint velocity. Unit: rad/s"""

    tau_d: List[float]
    """6 elements, Desired joint feedforward torque. Unit: Nm"""

    tau_d_1: List[float]
    """6 elements, Placeholder for desired joint feedforward torque. Unit: Nm"""

    label: str
    """Teach & TeachRepeat file name."""

    def setLabel(self, str:str) -> None:
        """Set label for the command. Will truncate to 10 characters if exceeds."""
        ...

    def setF_T_EE(self, T: npt.NDArray[np.float64]) -> None:
        """Input `T` should be a 4*4 matrix."""
        ...
    
    def set_inertia_ee(self, I: npt.NDArray[np.float64]) -> None:
        """Input `I` should be a 3*3 matrix"""
        ...

class ArmState:
    """Represents the state of an arm."""

    version: List[int]
    """3 elements, The z1 controller version."""

    mode: int
    """Desired running state. See ArmMode."""

    gripperState: "GripperState"
    """Measured gripper state."""

    gripperCmd: "GripperCmd"
    """Last desired gripper command."""

    q: List[float]
    """6 elements, Measured joint position. Unit: rad."""

    q_d: List[float]
    """6 elements, Last desired joint position. Unit: rad."""

    dq: List[float]
    """6 elements, Measured joint speed. Unit: rad/s."""

    dq_d: List[float]
    """6 elements, Last desired joint speed. Unit: rad/s."""

    tau: List[float]
    """6 elements, Measured joint output torque. Unit: Nm."""

    tau_d: List[float]
    """6 elements, Last desired joint feedforward torque. Unit: Nm."""

    motor_temperature: List[int]
    """8 elements, Measured motor temperature."""

    F_T_EE: List[float]
    """16 elements, Configured the end effector in flange frame."""

    errors: List[bool]
    """ErrorNum elements, Current robot error state."""

    motor_errors: List[List[bool]]
    """8 elements, Current motor error state, each with 8 elements."""



    def getF_T_EE() -> npt.NDArray[np.float64]:
        """Return mapped F_T_EE data."""
        ...

    def hasError() -> bool:
        """Check if any error exists."""
        ...

    def hasMotorError() -> bool:
        """Check if any motor error exists."""
        ...



class UnitreeArm:
    """Class representing a Unitree Arm"""

    def __init__(self, controllerIP: str, ownPort: int = 0) -> None:
        """
        Construct a new Unitree Arm object.
        
        :param controllerIP: IP/hostname of the z1_controller
        :param ownPort: Udp port to bind for this program. Use 0 by default.
        """
        ...

    def init(self) -> None:
        """Send an initialization message. Verify that the connection is valid."""
        ...

    def sendRecv(self) -> None:
        """Send and receive UDP packets."""
        ...

    def clearErrors(self) -> None:
        """Clear the running error. Currently available for clear collision detection signs."""
        ...

    def printlog(self) -> None:
        """Prints the logs based on state and errors."""
        ...

    armCmd: 'ArmCmd'
    """Placeholder for ArmCmd object"""

    armState: 'ArmState'
    """Placeholder for ArmState object"""

    dt: float
    """Time delta, set to 0.004. Read only."""

