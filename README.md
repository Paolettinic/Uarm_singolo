# Teleor single arm example 

Python controlled robot (Uarm simulated in VRep) that performs simple actions sent by Teleor. Perceptions
and actions are sent and received using Pedro.

## Requirements
-   Python3
-   [VRep](http://www.coppeliarobotics.com/downloads.html)
-   [vrep-api-python](https://github.com/Troxid/vrep-api-python)
-   [Qulog/Teleor Interpreter](http://staff.itee.uq.edu.au/pjr/HomePages/QulogHome.html) (requires [Qu-Prolog](http://staff.itee.uq.edu.au/pjr/HomePages/QuPrologHome.html) to be installed)
-   [Pedro server](http://staff.itee.uq.edu.au/pjr/HomePages/PedroHome.html)

## Starting the simulation

-   Open a terminal and run:

        pedro -L stdout

    to start Pedro server.
-   Start the simulation in Vrep
-   In another terminal run:
        
        python3 robot_interface.py
        
-   In another terminal move to AI directory:

        cd [path-to-project]/Uarm-singolo/AI
        
    then:
    
        teleor -Acontroller
       
    to start the interpreter, registering an agent called 'controller'
-   In the interpreter:

        [dismantle].
        go().
