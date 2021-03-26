<a href="https://luos.io"><img src="https://uploads-ssl.webflow.com/601a78a2b5d030260a40b7ad/602f8d74abdf72db7f5e3ed9_Luos_Logo_animation_Black.gif" alt="Luos logo" title="Luos" align="right" height="60" /></a>

# Luos Analyzer Project

This folder contains the source code for modifying, building and installing your Luos Analyzer. You can use these files for rebuilding the Luos Analyzer, though it is necessary only if you want to make changes in its functioning. Otherwise, you can use the precompiled executables outside of this folder.

In order to compile the analyzer for Windows, build the visual studio project contained in the Visual Studio folder. For Linux and Mac OS, run the buld_analyzer.py script. 

For further information on how to compile, debug and setup the analyzer visit the Saleae support page: https://support.saleae.com/saleae-api-and-sdk/protocol-analyzer-sdk/build

# Debugging on Windows

1. Open the Logic software.
2. In Visual Studio, open the Debug menu and select "attach to process"
3. In the available processes search box, search for Logic. You should see 5 results for Logic.exe. (Saleae uses the electron framework for Logic 2, which creates a handful of processes)
4. To figure out which one to use, open Task Manager (right click the task bar and select "task manager")
5. Expand the entry for Logic (5) in the Processes tab.
6. Locate the entry that is using the most ram, and right click it and select "go to details". Note the process ID you see there.
7. In visual studio, select the process that has the same ID, and attach.
8. Set a break point in your code (e.g. the top of "WorkerThread"). Or, just run your analyzer. Your crash will now just pause the debugger where the code crashed.
