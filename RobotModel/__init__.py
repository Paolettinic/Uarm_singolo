from pyrep import VRep
from pyrep.vrep import vrep as v
import functions
import numpy as np
import threading

class RobotState():
    def __init__(self):
        self.block_env = []
        self.Moving = False
        self.isHolding = False
        self.nextEFPosition = (0,0,0)
        self.goingToSharedSpace = False
        self.slot_env = [] # Only contains free slots

class RobotModel ():
    def __init__(self, name: str, api: VRep):
        self._api = api
        self._name = name
        self._sensors = None
        self._actuators = None
        self._state = None

    def process_commands(self,cmd):
        pass

class Uarm(RobotModel):
    def __init__(self, name, api):
        RobotModel.__init__(self, name, api)
        self._actuators = {}
        self._sensors = {}
        self._baseSize = 106.5
        self._arm1 = 149.7
        self._arm2 = 160.1
        self._endEffectorDisplacement = (0 , -25, 0)
        self._thethaDisplacementRad = (0, -19.3*np.pi/180, -4*np.pi/180)
        self._state = RobotState()
        self._actuators['base_motor'] = api.joint.with_position_control(name+"_motor1")
        self._actuators['first_motor'] = api.joint.with_position_control(name+"_motor2")
        self._actuators['second_motor'] = api.joint.with_position_control(name+"_motor3")
        self._actuators['endEffector_motor'] = api.joint.with_position_control(name+"_motor4")
        self._suctionHandler = name+"_suctionCup"
        self._sensors['cameraTop'] = api.sensor.vision(name+"_visionSensorTop")
        self._sensors['cameraFront'] = api.sensor.vision(name+"_visionSensorFront")

        self._cameraResX = 128
        self._cameraResY = 128

        # perm S7 (1653)(24)
        self._env_slots = [
            (74,225),
            (0,225),
            (-74,225),
            (149,225),
            (-153,225),
            (223,225),
            (-223,225)
        ]


    def setStateEnv(self,object_list):
        self._state.block_env = object_list

    def setHolding(self,isHolding):
        self._state.isHolding = isHolding

    def isHolding(self):
        return self._state.isHolding

    def getSizes(self):
        return self._baseSize, self._arm1, self._arm2, self._endEffectorDisplacement, self._thethaDisplacementRad

    def goHome(self):
        self.placeEnd((530 / 2, 0, 50))

    # TR
    def pickup(self,block):
        bColor = functions.index_to_color(block)
        x,y,z = self._state.block_env[bColor]
        scheme = [0,0,61,130]
        self.placeEnd((int(x),int(y),130))
        self.placeEnd((int(x),int(y),scheme[z]))
        self.enableSuction()
        self.placeEnd((int(x), int(y), 130))
        self.goHome()
        self.setHolding(block)

    def put_on_table(self):
        x,y = self._state.slot_env[0] #Use the first free slot
        self.placeEnd((x, y, 130))
        self.placeEnd((x,y,0))
        self.disableSuction()
        self.placeEnd((x, y, 130))
        self.goHome()
        self.setHolding(0)
        self._state.slot_env.remove((x,y)) # the slot has just been used so it's no longer free

    def placeEnd(self, coordinates: tuple):
        x, y, z = coordinates
        theta = functions.getMotorsTetha(self, x, y, z)
        self._state.Moving = True
        self.rotateMotors(*theta)

    def rotateMotors(self, theta1, theta2, theta3 ):
        self._state.Moving = True

        t1 = int(theta1 * 100) / 100
        t2 = int(theta2 * 100) / 100
        t3 = int(theta3 * 100) / 100

        self._actuators['base_motor'].set_target_position(theta1)
        self._actuators['first_motor'].set_target_position(theta2)
        self._actuators['second_motor'].set_target_position(theta3)
        self._actuators['endEffector_motor'].set_target_position(theta1)
        post1 = int(self._actuators['base_motor'].get_position() * 100) / 100
        post2 = int(self._actuators['first_motor'].get_position() * 100) / 100
        post3 = int(self._actuators['second_motor'].get_position() * 100) / 100

        while(t1 != post1 or t2 != post2 or t3 != post3): #
            post1 = int(self._actuators['base_motor'].get_position() * 100) / 100
            post2 = int(self._actuators['first_motor'].get_position() * 100) / 100
            post3 = int(self._actuators['second_motor'].get_position() * 100) / 100

        print("FINISHED MOVING")
        self._state.Moving = False

    def enableSuction(self):
        self._state.pickedUp = True
        self.setIntegerSignal(self._suctionHandler,1)

    def disableSuction(self):
        self.setIntegerSignal(self._suctionHandler, 0)

    def getState(self):
        with self._state as state:
            return {
                "is_moving":state.Moving,
                "is_holding":state.isHolding if state.isHolding else "nothing",
                "next_position":state.nextEFPosition,
                "going_to_shared":state.goingToSharedSpace
            }

    def get_percepts(self):
        """
        Reads the top and front Vision Sensors.
        :return array containing, for each detected blob
                (color, x_position, y_position, width, height):
        """

        #value gathered from the filters
        codeTop, stateTop, imageTop = self._sensors["cameraTop"].read()
        codeFront, stateFront, imageFront = self._sensors["cameraFront"].read()
        img = self._sensors["cameraTop"].raw_image()
        rawTop = np.array(img, dtype=np.uint8)
        rawTop.resize(128,128,3)

        rawFront = np.array(self._sensors["cameraFront"].raw_image(), dtype=np.uint8)
        rawFront.resize(128, 128, 3)
        object_list = functions.readVisionData(imageTop, imageFront,rawTop ,rawFront)
        self._state.block_env = object_list
        self._state.slot_env = functions.getFreeSlots(object_list,self._env_slots)
        print(object_list)
        return object_list



    def process_commands(self, commands):
        for cmd in commands:
            if not cmd == None:
                self.invoke(cmd['cmd'], cmd['args'])

    def invoke(self, cmd, args):
        print('invoke', cmd, args)
        if cmd != 'illegal_command':
            try:
                getattr(self.__class__, cmd)(self, *args)
            except AttributeError:
                raise NotImplementedError("Class `{}` does not implement `{}`".format(self.__class__.__name__, cmd))

    def setIntegerSignal(self,name: str,value: int):
        v.simxSetIntegerSignal(self._api._id,name, value,v.simx_opmode_oneshot)


class RobotTask(threading.Thread):
    def __init__(self,robot: RobotModel,cmd):
        threading.Thread.__init__(self)
        self.cmd = cmd
        self.robot = robot

    def run(self):
        self.robot.process_commands(self.cmd)
