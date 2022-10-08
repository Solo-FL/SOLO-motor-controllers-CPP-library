|License|

*******
SOLO-motor-controllers-CPP-library
*******

SOLO Motor Controller's C++ Library can be used with Windows OS to Set or Read all the parameters that are stored or existing in command set of SOLO, for more information on that please visit `the SOLO website <https://www.solomotorcontrollers.com/>`_.

The library support USB or Kvaser communication. 

The Library is compiled using GCC, Visual Studio Code (`C/C++ for Visual Studio Code extension  <https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools>`)

Kvaser work on top of `Kvaser Drivers for Windows V5.39  <https://www.kvaser.com/download/>`


Authors
=======

SOLO Motor Controller CPP Library is created by SOLO Motor Controllers team


Folders and Files Hierarchy
=======

src/ = Source File folder

  SOLOMotorControllersKvaser.* = implementation of SOLOMotorControllers using Kvaser Drivers

  SOLOMotorControllersSerial.* = implementation of SOLOMotorControllers using USB 

examples/ = Examples File folder

  serial/ = Examples over SOLOMotorControllersSerial 

  kvaser/ = Examples over SOLOMotorControllersKvaser
   
inc/ = File used by Kvaser implementation

lib/ = .Lib used by Kvaser implementation

tests/ = Test folder, reference for testing unit

vscode example/ = reference over VScode set-up (reference for Build/Debug using VScode)

buildAll.bat = spript for complie all the cpp file in examples and tests (in the main folder, write in the terminal: .\buildAll  )

License
=======

GNU General Public License v3.0 or later

See `COPYING <COPYING>`_ to see the full text.

.. |License| image:: https://img.shields.io/badge/license-GPL%20v3.0-brightgreen.svg
   :target: COPYING
   :alt: Repository License

