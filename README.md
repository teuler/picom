# picom
 A file synchronisation tool for PicoMite

## Installation
1) Clone the repository and create an environment.
   ```
   git clone https://github.com/teuler/picom.git
   python -m venv picom
   ```
2) Activate environment and install requirements.    
   Under Windows:
   ```
   .\picomenv_name\Scripts\activate
   pip install pyserial
   pip install xmodem
   cd picom
   ```
   Under Linux:
   ```
   source picom/bin/activate
   pip install pyserial
   pip install xmodem
   cd picom
   ```

## Usage
Enter `python picom.py -h` to get help:
```
PicoM v0.1.2 (beta)
usage: PicoM v0.1.2 (beta) [-h] [-s SERIAL] [-d DRIVE] [-p PATH] [-f FILES] [-n NAME] command

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
```

## Open issues
- Linux: Reconnecting to Pico after sending a command that resets the serial connection (e.g., `option reset`)
  does not yet work, because the Pico does not necessarily connects to the same device address.

