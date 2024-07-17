|License|

*******
SOLO-motor-controllers-CPP-library
*******

SOLO Motor Controller's C++ Library can be used with Windows OS to SETor READ all the parameters that are stored or existing in the command set of SOLO, for more information on that please visit `the SOLO website <https://www.solomotorcontrollers.com/>`_.

This library supports all the communications offered by SOLO including UART, USB, or CANopen communication using KVASER products.  

Kvaser works on top of  `Kvaser Drivers for Windows V5.39  <https://www.kvaser.com/download/>`_


Branch Info
=======

MAIN BRANCH: The Library is tested with Visual Studio 2022 
(or can be compiled using GCC supporting Visual Studio Code (`C/C++ for Visual Studio Code extension  <https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools>`_))


Start up Info
=======

Open the Project:
1. double click on SOLO-motor-controllers-CPP-library.sln to start Visual Studio 2022 as a Visual studio project

Run an Example:
1. In the Visual Studio go to the Solution Explorer Menu
2. right click on the example you want to execute
3. select "include In Project"
4. press the run icon

Note: in case you want to run a different file you need to remove the file from the project:
1. In the Visual Studio go to the Solution Explorer Menu
2. right click on the example you want to esclude
3. select "Exclude From Project"


Authors
=======

SOLO Motor Controller CPP Library is created by SOLO Motor Controllers team


Folders and Files Hierarchy
=======

src/ = Source File folder

  SOLOMotorControllersKvaser.* = implementation of SOLOMotorControllers using Kvaser Drivers

  SOLOMotorControllersSerial.* = implementation of SOLOMotorControllers using USB 

examples/ = Examples File folder

  serial/ = Examples over Serial 

  Canopen/ = Examples over CANopen
   
inc/ = File used by Kvaser implementation

lib/ = .Lib used by Kvaser implementation

vscode example/ = reference over VScode set-up (reference for Build/Debug using VScode)

License
=======

MIT License
