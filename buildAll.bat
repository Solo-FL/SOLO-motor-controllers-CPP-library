@REM EXAMPLE OF SINGLE COMPILE COMMAND:
@REM g++ -g -I./src -I./inc ./src/*.cpp ./examples/serial/ReadingBusVoltage.cpp -L./Lib -lcanlib32 -o ./examples/serial/ReadingBusVoltage.exe
@REM g++ -g -I./src -I./inc ./src/*.cpp ./examples/serial/ReadingBusVoltage2.cpp -L./Lib -lcanlib32 -o ./examples/serial/ReadingBusVoltage2.exe

@REM EXAMPLE OF LOOPS:
@REM FOR %%I in (.\*.*) DO echo %%I
@REM FOR %%I in (./examples/serial/*.cpp) DO echo %%~nI

@REM COMMANDS LIST:
FOR %%I in (./examples/serial/*.cpp) DO g++ -g -I./src -I./inc ./src/*.cpp ./examples/serial/%%~nI.cpp -L./Lib -lcanlib32 -o ./examples/serial/%%~nI.exe
FOR %%I in (./examples/kvaser/*.cpp) DO g++ -g -I./src -I./inc ./src/*.cpp ./examples/kvaser/%%~nI.cpp -L./Lib -lcanlib32 -o ./examples/kvaser/%%~nI.exe
FOR %%I in (./tests/*.cpp) DO g++ -g -I./src -I./inc ./src/*.cpp ./tests/%%~nI.cpp -L./Lib -lcanlib32 -o ./tests/%%~nI.exe