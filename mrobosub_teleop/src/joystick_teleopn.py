#!/usr/bin/env python

from calendar import c
from enum import Enum
import string
from tokenize import String
from typing import List, cast

from matplotlib import axis, scale
from matplotlib.pyplot import flag
from numpy import rate
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Header, Float64
from std_srvs.srv import Trigger
from typing import Callable, Dict, Union, Tuple
from time import sleep
from mrobosub_lib.lib import Node, Param

AXES = ['sway', 'surge', 'heave', 'yaw', 'roll', 'pitch']

class ButtonCommand(int, Enum):
    INCREASE = 1
    DECREASE = -1
    TOGGLE = 0

class Button:
    """ Store commands tied to buttons """
    def __init__(self, idx : int, commandstr : str) -> None:
        self.idx = idx
        self.commandstr = commandstr
        self._ispressed = False
        self._isrising_edge = False
        self.name, self._action = self.__split_command(commandstr)
        
    @staticmethod
    def __split_command(comstr : str) -> Tuple[str, ButtonCommand]:
        """ Split command string into (axis, command) """
        if comstr.endswith(".increase"):
            return (comstr[:-9], ButtonCommand.INCREASE)
        elif comstr.endswith(".decrease"):
            return (comstr[:-9], ButtonCommand.DECREASE)
        elif comstr.endswith(".toggle"):
            return (comstr[:-7], ButtonCommand.TOGGLE)
        # Special commands
        elif comstr.endswith("estop") or comstr.endswith("switch"):
            return (comstr[4:], ButtonCommand.TOGGLE)
        else:
            raise ValueError(f"Button command {comstr} not recognised")
        
    def update(self, ispress : bool) -> None:
        self._isrising_edge = (not self.pressed) and ispress
        self._ispressed = ispress
        
    @property
    def rise_edge(self) -> bool:
        return self._isrising_edge
    
    @property
    def pressed(self) -> bool:
        return self._ispressed
    
    @property
    def command(self) -> Tuple[str, ButtonCommand]:
        return self.name, self._action
   
class DOF:
    """ Store state of a degree of freedom """
    class DOFState(int, Enum):
        POSE = 0 
        TWIST = 1
    
    def __init__(self, id : int, name : str, scale : float) -> None:
        self.idx = id
        self.name = name
        self.state = DOF.DOFState.POSE
        self.scale = scale
        
    def update(self, command : ButtonCommand, scale : float) -> float:
        """ Command is decided by button press, scale is from axis """
        self.state ^= int(not bool(abs(command))) # toggle if command is TOGGLE
        self.scale = scale * command
        return self.scale

class Joystick_teleopn(Node):
    """
    Publishers
    - /target_pose/heave
    - /target_twist/heave
    - /output_wrench/heave
    - /target_twist/surge
    - /output_wrench/surge
    - /target_twist/sway
    - /output_wrench/sway
    - /target_pose/yaw
    - /target_twist/yaw
    - /output_wrench/yaw
    - /target_pose/roll
    - /target_twist/roll
    - /output_wrench/roll
    - /target_pose/pitch
    - /target_twist/pitch
    - /output_wrench/pitch

    Subscribers
    - /joy
    - /pose/heave
    - /pose/yaw
    """
    axes : Dict[str, Dict[str, Union[str,float]]]
    buttons : Dict[str, Dict[str, str]]
    rate : float # Hz
    
    def __init__(self) -> None:
        super().__init__("joystick_teleopn")
        self.input_subscriber = rospy.Subscriber("/joy", Joy, self.joystick_callback)
        self.wrench_pubs = [rospy.Publisher(f'/output_wrench/{axis}', Float64, queue_size=1) for axis in AXES]
        # Globals:
        self.buttonsInstance : List[Button] = []
        self.DOFInstance : Dict[str, DOF] = {}
        self.stateMachineMode = False
        
    def init(self) -> None:
        for i in range(6):
            if (DOFConfig := self.axes.get(str(i))):
                name = str(DOFConfig['use'])
                self.DOFInstance[name]= DOF(i, name, float(DOFConfig['scale']))
            else:
                raise ValueError(f"Axis {i} not configured in params")
        for i in range(10):
            if (ButtonConfig := self.buttons.get(str(i))):
                self.buttonsInstance.append(Button(i, str(ButtonConfig['use'])))
            else:
                raise ValueError(f"Button {i} not configured in params")
        return
       
    def joystick_callback(self, msg: Joy):
        for button in self.buttonsInstance:
            button.update(msg.buttons[button.idx] == 1)
            if button.rise_edge:
                name, command = button.command
                if name == "estop":
                    self.hard_stop()
                    return
                elif name == "switch":
                    self.stateMachineMode ^= True
                else: 
                    if tar := self.DOFInstance.get(name):
                        tar.update(command, msg.axes[tar.idx])
                    else: 
                        raise ValueError(f"Button command {name} not recognised")
            
    def hard_stop(self):
        for pub in self.wrench_pubs:
            pub.publish(Float64(0.0))
        return
    
    def run(self):
        self.init() # load globals
        while not rospy.is_shutdown():
            
            sleep(1/self.rate)
        return 0
            
if __name__ == "__main__":
    Joystick_teleopn().run()