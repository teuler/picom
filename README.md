# picom
 A file synchronisation tool for the [PicoMite](https://geoffg.net/picomite.html) family, Raspberry Pico Pico(2) microcontrollers running MMBasic by Geoff Graham and Peter Mather.

## Installation
### As a Python script
1) Clone the repository and create an environment.
   ```
   git clone https://github.com/teuler/picom.git
   python -m venv picom
   ```
2) Activate environment and install requirements.    
   Under Windows:
   ```
   .\picom\Scripts\activate
   pip install pyserial
   pip install xmodem
   pip install psutil
   cd picom
   ```
   Under Linux:
   ```
   source picom/bin/activate
   pip install pyserial
   pip install xmodem
   pip install psutil
   cd picom
   ```
### As an `.exe` file (Windows)
The archive `picom_v0_1_8_Windows.zip` contains an `.exe` file of the Python program together with the necessary DLLs. Copy the `picom` folder to your drive and execute `picom` from within that folder.
   
## Configuration `picom.toml`
This file contains some basic settings, such as the preferred serial port and the baudrate.

Here, the program can be tuned. For example, for `com_baudrate = 921600`, a wait time for finishing an XMODEM transfer of `xmodem_wait_s = 1.5` is sufficient.
At `com_baudrate = 115200`, the XMODEM wait time needs to be longer (`xmodem_wait_s = 3.0`), otherwise the program may fail.


## Usage
Enter `python picom.py -h` (or, in case of the executable `picom -h`) to get help:
```
PicoM v0.1.8 (beta)
usage: PicoM v0.1.8 (beta) [-h] [-s SERIAL] [-d DRIVE] [-p PATH] [-f FILES] [-n NAME] command

A file synchronisation tool for PicoMite.
Note that the quotes are only required for multiple-word commands. All file
names, path, and drive parameters need to be given in quotes.

commands:
  c, "check"         Check if PicoMite present and get version information
  p, "ports"         List available serial ports (Windows only)
  ol, "option list"  Print options set on PicoMite
  f, "files"         List files on given drive and current folder
                     (considers options `-d`, `-p`)
  ft, "filetree"     List all files
                     (considers option `-d`)
  xs, "xmodem s"     Send file(s) to PicoMite
                     (requires option `-f`, considers `-d`, `-p`)
  xr, "xmodem r"     Retrieve file(s) from Picomite
                     (requires option `-f`, considers `-d`, `-p`)
  b, "backup"        Create complete backup of the PicoMite's given drive
                     (requires option `-n`, considers `-d`)
  r, "restore"       Restore a backup from a local folder to the PicoMite
                     (requires option `-n`, considers `-d`)

positional arguments:
  command

options:
  -h, --help            show this help message and exit
  -s SERIAL, --serial SERIAL
                        Serial port, defaults to `COM5`
  -d DRIVE, --drive DRIVE
                        Drive `A:` or `B:`, defaults to `A:`
  -p PATH, --path PATH  File path, defaults to an empty path
  -f FILES, --files FILES
                        Comma-separated list of file name(s)
  -n NAME, --name NAME  Name of backup to generate or restore
  -t, --terminal        End terminal program if one is still running and restart it after the command
```

## Trouble shooting
- Note that if your file names contain `$` characters, you cannot use the Windows powershell, because there variable names start with `$`. Use `cmd.exe` instead. Same is true for (some?) Linux shells. In Linux, you can "escape" the `$` by using `\$`.
- To restore a backup, use the name (folder name) that `picom` generated - don't change the name, because `picom` uses the timestamp (separated by `_`) to manage its files and to restore options and libraries.

## Release notes
- v0.1.8
  - Fixed a bug when using B: drive
  - Added `-t` option (experimental), which kills the terminal program defined in `picom.toml` on command start and tries to restart that terminal program when the command is finished. Needs a full path to the program in `picom.toml`. 
- v0.1.7
  - Some changes to account for Python 3.13
  - With the newest firmware, saving a library on the PicoMite seems to take a bit more time. Therefore,  `sendCommand` has now an additional parameter that allows to wait between sending a command to the PicoMite and looking for the reply.
  - Give a warning if the name of the backup does not contain a time stamp.
- v0.1.6
  - Bug fixed when using file names with capital characters
- v0.1.5
  - Bug fixed in filetree representation
  - Now user is asked if to proceed, if file that is transferred already exists
  - `picom.toml` has a new parameter `ask_questions`; if set to 0, most questions (e.g., overwrite target file)
    are suppressed.
- v0.1.4
  - Filetree command added
- v0.1.3
  - Send `Ctr-C` after opening a serial port to interrupt any running program
- v0.1.2
  - First release

## Open issues
- Linux: Reconnecting to Pico after sending a command that resets the serial connection (e.g., `option reset`)
  does not yet work, because the Pico does not necessarily connects to the same device address.
- `xmodem r`: when receiving files from subfolders on the PicoMite, the local target folder needs to exist (is not automatically created)
