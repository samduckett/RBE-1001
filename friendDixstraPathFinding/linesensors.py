from XRPLib.reflectance import Reflectance

class Linesensors:

    def __init__(self) -> None:

        # initiate a default reflectance object 
        self.reflectance = Reflectance.get_default_reflectance()

    
    def get_error(self) -> float:

        # return error values
        # print error for debug (uncomment if needed)
        # print('getting error')
        return self.reflectance.get_right() - self.reflectance.get_left()
    

    def at_cross(self) -> bool: 

        # check if the robot is at an intersection
        # return true if both sensors are at an intersection
        return self.reflectance.get_right() > 0.85 and self.reflectance.get_left() > 0.85


    
    def either_on_line(self):

        # check if either sensor is on on the line (used to confirm turning)
        # return true if either sensor is on the line
        return self.reflectance.get_right() > 0.85 or self.reflectance.get_left() > 0.85