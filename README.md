# Flight Log Converter

Technical Specs:

1. Windows OS
2. Telemetry log files to load
3. An excel program to open the exported .csv.
4. No installation needed. Use the program via running the .exe
file.
5. The Flight Log Converter is named as such because it converts
Flight log formats into a .csv format.

Installation:
1. Download the Executable folder in the GitHub repository.
2. Run the FlightLogConverter.exe inside the Release folder.

Program Instructions for .tlog files:

1. Load a telemetry log (.tlog). To know when your file is fully
loaded, the progress bar will become fully green.

2. "Select All" and "Remove All Selected" will be enabled
and become green and red respectively.

"Select All" will select all the data parameters.

"Remove All Selected" will remove all the selected data
parameters.

3. "Clear Loaded Log" and ".CSV Export" will be enabled.
"Clear Loaded Log" will remove the current telemetry log file
loaded.

".CSV Export" will export the selected data paramters
(including time and date) in to a .CSV file and be put in your
Downloads folder. It will then clear the current loaded file.

Program Instructions for .bin files:

1. Click on the "Load" button.

2. Locate your .bin file and press "OK"

3. A message box will display and ask if you are okay with just converting
to a .csv format since the data parameters cannot be displayed.

If you select "yes", it will convert it the .bin to a .csv and display
a success message to alert you when it is done. If you select "no", 
it will do nothing.
