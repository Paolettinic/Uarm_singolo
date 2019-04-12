from RobotModel import Uarm, VRep
import pedroclient
import queue
import threading
import time
import functions as f

class Control(object):
    def __init__(self, host, port, sleep_time):
        self._host = host
        self._port = port
        self._sleep_time = sleep_time
        try:
            self._api = VRep.connect(self._host, self._port)
        except:
            print('V-REP not responding')
            exit(-1)
        self.robot = None
        self._api.simulation.start()

    def run(self):
        while True:
            self.process_percepts(self.robot.get_percepts(),self.robot.isHolding())
            self.robot.process_commands(self.get_commands())

    def make_robot(self, api):
        return None

    def process_initialize(self):
        pass

    def process_percepts(self, block_percept,arm_percept):
        pass

    def get_commands(self):
        pass


# Handling messages from the TR program
class MessageThread(threading.Thread):
    def __init__(self, client, q):
        self.running = True
        self.client = client
        self.queue = q
        threading.Thread.__init__(self)
        self.daemon = True

    def run(self):
        while self.running:
            p2pmsg = self.client.get_term()[0]
            print("-----------P2P MESSAGE_",p2pmsg)
            self.queue.put(p2pmsg)

    def stop(self):
        self.running = False

class PedroControl(Control):
    def __init__(self, host='127.0.0.1', port=19997, sleep_time=1):
        super().__init__(host, port, sleep_time)
        self.client = pedroclient.PedroClient()
        self.client.register("robot_sim")
        self.queue = queue.Queue(0)
        self.message_thread = MessageThread(self.client, self.queue)
        self.message_thread.start()
        self._tr_client_addr = 0


    def process_percepts(self, block_percepts, arm_holding):
        msg = []
        if not arm_holding == 0:
            msg.clear()
            msg.append('holding({0})'.format(arm_holding))
        else:
            for color,(x,y,z) in block_percepts.items():
                #print("color: {0}".format(color))
                #print("X: {0} | Y: {1} | Z: {2}".format(x,y,z))
                index_color_on = f.color_to_index(color)
                if z == 1:
                    msg.append('on_table({0})'.format(index_color_on))
                else:
                    for c,(px,py,pz) in block_percepts.items():
                        if px == x and py == y and pz == z - 1:
                            index_color_under = f.color_to_index(c)
                            msg.append('on({0},{1})'.format(index_color_on,index_color_under))

        if not arm_holding == 0:
            msg.clear()
            msg.append('holding({0})'.format(arm_holding))
        # else:
        #     msg.append('[f_(holding(1))]')

        self.send_percept('['+','.join(msg)+']')


    def send_percept(self, percepts_string):
            print("send_percept", str(self.percepts_addr), percepts_string)
            if self.client.p2p(self.percepts_addr, percepts_string) == 0:
                print("Error", percepts_string)


    def get_commands(self):
        cmds = []
        while not self.queue.empty():
            p2pmsg = self.queue.get()
            print("P2PMSG=",p2pmsg)
            msg = p2pmsg.args[2]
            print("MESSAGE=", msg)
            actions = msg
            for a in actions.toList():
                cmds.append(self.action_to_command(a))
        return cmds

    def process_initialize(self):
        # Block unitil message arrives
        self.robot = self.make_robot(self._api)
        block_percept = self.robot.get_percepts()
        arm_percept = self.robot.isHolding()

        p2pmsg = self.queue.get()
        print(p2pmsg)
        message = p2pmsg.args[2]
        if str(message) == 'initialise_':
            percepts_addr = p2pmsg.args[1]
            print("percepts_addr", str(percepts_addr))
            self.set_client(percepts_addr)
            self.process_percepts(block_percept,arm_percept)
        else:
            print("Didn't get the initialize message")

    def set_client(self, addr):
        self.percepts_addr = addr

    def make_robot(self, api) -> Uarm:
        return Uarm('uarmR', api)

    def action_to_command(self, a):
        cmd_type = a.functor.val
        cmd = a.args[0]
        if cmd_type == 'start_':
            print("------------------------START CMD:\t\t\t",cmd.functor.val," | ",cmd.args[0])
            if cmd.functor.val == "pickup":
                return {'cmd': cmd.functor.val, 'args': [cmd.args[0].val]}
            if cmd.functor.val == "put_on_table":
                print("PUT ON TABLE")
                return {'cmd': "put_on_table", 'args': []}
        else:
            print("--------------------------STOP CMD:\t\t\t", cmd)

        #else return {'cmd':'action', 'args':[]}

    def moveTo(self,position):
        pass

    def pickup(self,block,arm):
        pass

    def placeAoverB(self,block,block_or_tower):
        pass


def pedro_control():
    '''
        Out of process robot control by a teleor AI
    :return:
    '''
    vrep_pedro = PedroControl()

    # wait for and process initialize_ message
    vrep_pedro.process_initialize()
    vrep_pedro.run()


class DemoControl(Control):

    def __init__(self, host='127.0.0.1', port=19997, sleep_time=2):
        super().__init__(host, port, sleep_time)
        self.i = 0


    def make_robot(self, api)-> Uarm:
        return Uarm('uarmR', api)

    def process_percepts(self, object_percepted,arm_status):
        for i in range((object_percepted)):
            print (i)

    def get_commands(self):
        return [{'cmd': 'placeEnd', 'args': [(-74,231,50)]}]

    def process_initialize(self):
         return [{'cmd': 'placeEnd', 'args': [(0, 100, 100)]}]




def demo_control():
    vrep_demo = DemoControl()
    #vrep_demo.process_initialize()
    #time.sleep(1)
    vrep_demo.run()