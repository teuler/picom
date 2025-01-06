##!/usr/bin/env python
# -*- coding: utf-8 -*-
# ---------------------------------------------------------------------------
# picom.py
# ...
#
# The MIT License (MIT)
# Copyright (c) 2025 Thomas Euler
# ---------------------------------------------------------------------------
import serial
import io
import sys
import time
import textwrap
import argparse
import tomllib
import logging
import datetime
import platform
from pathlib import Path
from xmodem import XMODEM

PROG_NAME      = "PicoM"
PROG_VER       = "0.1.2 (beta)"

# ---------------------------------------------------------------------------
# Default configuration (replaced by content of `picom.toml`)
#
COM_PORT        = "COM1"
COM_BAUDRATE    = 921_600
COM_TOUT_S      = 0.2

XMODEM_RETRY    = 8
XMODEM_WAIT_S   = 1.5
XMODEM_PKG_SIZE = 128

VERBOSE         = True

# ---------------------------------------------------------------------------
class ErrCode:
    OK              = 0
    INVALID_DRIVE   = 10
    DRIVE_NOT_READY = 11
    B_NOT_ENABLED   = 12
    INVALID_PATH    = 13
    FOLDER_EXISTS   = 14
    INVALID_CMD     = 20
    PARAM_MISSING   = 21
    NO_PICO_FOUND   = 30
    # ...

class ElementType:
    IS_FILE         = 1
    IS_FOLDER       = -1
    IS_NONE         = 0    

# ---------------------------------------------------------------------------
# Global variables and callbacks for XMODEM 
# ---------------------------------------------------------------------------
xmodem_n_pkgs = 0
xmodem_pre = ""   
xmodem_post = ""  
xmodem_pkg_size = XMODEM_PKG_SIZE

def _getc(size, timeout=1):
    return Ser.read(size) or None

def _putc(data, timeout=1):
    return Ser.write(data) 

# ---------------------------------------------------------------------------
# Definitions of command line arguments
# ---------------------------------------------------------------------------
def getCmdLineArgs() -> list:
    """ Setup argument parser and get arguments
    """
    log("Getting command line arguments ... ", noLF=True)
    _parser = argparse.ArgumentParser(
        prog=f"{PROG_NAME} v{PROG_VER}",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        description=textwrap.dedent('''\
            A file synchronisation tool for PicoMite. 
            Note that the quotes are only required for multiple-word commands. All file
            names, path, and drive parameters need to be given in quotes. 
                                    
            commands:
              c, "check"         Check if PicoMite present and get version information
              p, "ports"         List available serial ports (Windows only)
              ol, "option list"  Print options set on PicoMite
              f, "files"         List files on given drive and current folder
                                 (considers options `-d`, `-p`)   
              xs, "xmodem s"     Send file(s) to PicoMite
                                 (requires options `-f`, considers `-d`, `-p`)   
              xr, "xmodem r"     Retrieve file(s) from Picomite
                                 (requires options `-f`, considers `-d`, `-p`)   
              b, "backup"        Create complete backup of the PicoMite's given drive
                                 (requires options `-n`, considers `-d`)   
        '''), 
        epilog=''
    )
    _parser.add_argument("command")
    _parser.add_argument(
        "-s", "--serial", 
        nargs=1, default=COM_PORT,  
        help=f"Serial port, defaults to `{COM_PORT}`"
    ) 
    _parser.add_argument(
        "-d", "--drive", 
        nargs=1, default="A:",  
        help="Drive `A:` or `B:`, defaults to `A:`"
    ) 
    _parser.add_argument(
        "-p", "--path", 
        nargs=1, default="",  
        help="File path, defaults to an empty path"
    )     
    _parser.add_argument(
        "-f", "--files", 
        nargs=1, default=[],  
        help="Comma-separated list of file name(s)"
    )     
    _parser.add_argument(
        "-n", "--name", 
        nargs=1, default="",  
        help="Name of backup to generate or restore"
    )     
    log("done.", noHeader=True)
    return _parser.parse_args()

# ---------------------------------------------------------------------------
# Handling of serial port
# ---------------------------------------------------------------------------
def createSerialIO(_port :str, _baudrate :int) -> tuple:
    """ Open and return a serial port instance
    """ 
    try:
        ser = serial.Serial(_port, baudrate=_baudrate, timeout=COM_TOUT_S)
        serIO = io.TextIOWrapper(io.BufferedRWPair(ser, ser), newline=None)
    except serial.serialutil.SerialException:
        ser = serIO = None   
    return ser, serIO


def _listSerialPorts(lastPort=10) -> list:
    """ Return list of serial ports that can be opened
        NOTE: Currently, Windows only
    """
    assert platform.system().lower() == "windows"
    ports = []
    for i in range(lastPort):
        try:
            port = f"COM{i}"
            ser = serial.Serial(port)
            ser.close()
            ports.append(port)
        except serial.SerialException:
            pass   
    return ports
    

def cleanUp(errC :ErrCode, noClose :bool =False):
    """ Close ports etc.
    """
    if not noClose:
        Ser.close()
    if errC == ErrCode.OK:
        log("Done.") 

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
def sendCommand(_cmd :str, doPrint=False) -> list:
    """ Sends a command via the serial port to the Pico and returns the 
        REPL output as a list of strings; prints the reply, if `doPrint`
    """
    # Send command
    cmd = _cmd.lower()
    SerIO.write(cmd +"\n")
    SerIO.flush() 
    
    # Retrieve output
    done = False
    repl = []
    while not(done):
        #t = time.monotonic()
        res = SerIO.readline()
        #print("dt=", time.monotonic() -t, " res=", res)
        if not(done := len(res) == 0):
            # Filter out lines starting with `>` and the first line (which
            # mirrors the command), and add the lines to a list
            res = res[:-1]

            if len(res) > 0 and res[0] != ">" and res.lower() != cmd:
                repl.append(res)

    # Print output, if requested
    if doPrint:
        _print(repl)           
    return repl

# ---------------------------------------------------------------------------
# Logging and printing
# ---------------------------------------------------------------------------
def log(msg :str, noLF=False, noHeader=False):
    if VERBOSE:
        msg = "# "+msg if not noHeader else msg
        post = "" if noLF else "\n"
        print(msg, end=post)

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
def _print(repl :list):
    """ Print output
    """
    #print("-->")
    for ln in repl:
        print("| " +ln)
    #print("<--")
    

def _print_progress_bar(index :int, total :int, pre :str, post :str):
    n_bar = 50  
    progress = index /total
    sys.stdout.write('\r')
    sys.stdout.write(
        f"{pre} [{'=' *int(n_bar *progress):{n_bar}s}] {int(100 *progress)}% {post}"
    )
    sys.stdout.flush()

# ---------------------------------------------------------------------------
# Retrieving general information from the PicoMite
# ---------------------------------------------------------------------------
def checkPicoMite(doDetailed=False) -> dict:
    """ Check if PicoMite is responding and get version information
    """
    log("Checking for PicoMite and retrieving key data ... ", noLF=True)
    ver = dict()
    res = sendCommand("option list")
    if len(res) > 0:
        tmp = res[0].split()
        if "picomite" in tmp[0].lower():
            ver.update({
                "firmware": tmp[0], 
                "chip": tmp[2], "version": tmp[4]
            })
    if doDetailed:
        res = sendCommand("?MM.INFO$(CPUSPEED)")
        ver.update({"cpu_speed": int(res[0])}) 
        res = sendCommand("?MM.DEVICE$")
        ver.update({"device": res[0]}) 
        res = sendCommand("?MM.INFO$(DRIVE)")
        ver.update({"drive": res[0]}) 
        res = sendCommand("?MM.INFO(FREE SPACE)")
        ver.update({"free_disk_space": int(res[0])}) 
        res = sendCommand("?MM.INFO(DISK SIZE)")
        ver.update({"total_disk_space": int(res[0])})             
        res = sendCommand("?MM.INFO(ID)")
        ver.update({"ID": res[0]})             

    log("done.", noHeader=True)
    return ver

# ---------------------------------------------------------------------------
# Functions to handling the file system on the PicoMite
# ---------------------------------------------------------------------------
def checkDrive(_drive: str, doChange :bool =True) -> tuple:
    """ Check if drive is valid and available
    """
    if _drive.upper() not in ["A:", "B:"]:
        return (ErrCode.INVALID_DRIVE, _drive, []) 
    else:
        if doChange:
            repl = sendCommand(_drive)
            if len(repl) > 0 and "error" in repl[0].lower():
                return (ErrCode.B_NOT_ENABLED, repl[0][8:], [])
        return (ErrCode.OK, "", [])


def checkFileExists(fname :str) -> ElementType:
    """ Returns True if file exists
    """
    repl = sendCommand(f'?MM.INFO(EXISTS FILE "{fname}")')
    assert len(repl) > 0
    if int(repl[-1].strip()) == 1:
        return ElementType.IS_FILE
    repl = sendCommand(f'?MM.INFO(EXISTS DIR "{fname}")')
    if int(repl[-1].strip()) == 1:    
        return ElementType.IS_FOLDER
    return ElementType.IS_NONE


def getFileSize(fname :str) -> int:
    """ Returns size of given file in bytes or -1, if file was not found
    """
    if checkFileExists(fname) != ElementType.IS_FILE:
        return -1
    repl = sendCommand(f'?MM.INFO(FILESIZE "{fname}")')
    return int(repl[-1])
 

def getAbsLocalPath(path_local :str) -> str:
    """ Return a complete local path terminating with "\" 
    """  
    if len(path_local) == 0:
        return ""
    assert platform.system().lower() == "windows"      
    path_local_abs = Path(path_local).resolve().__str__() +"\\"
    return path_local_abs.replace("\\", "/") # +"/"

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
def getFileTree(drive :str) -> list:
    """ Returns the complete file tree on `drive` as a list of paths in the
        form  `[errCode, msg, pathlist]`
    """
    def _getSubTree(drive :str, path :str) -> list:
        ftree = []
        dlist = []
        cmd = f'files "{drive +path}"'
        log(f"Sending `{cmd}` ...")
        repl = sendCommand(cmd)    
        for i, ln in enumerate(repl):
            if i < 3 or i > len(repl) -2:
                # Skip the first three lines with the drive, `.`, and `..`
                # and last line
                continue
            
            tmp = ln.split()
            if len(tmp) == 2 and "DIR" in tmp[0]:
                # Is folder
                dir = path +tmp[1] +"/"
                ftree.append([dir, ElementType.IS_FOLDER])
                dlist.append(dir)

            elif len(tmp) == 4:
                # Is file
                ftree.append([path +tmp[-1], ElementType.IS_FILE])

            else:
                print(f"Warning: Entry #{i} (`{ln}`) not recognized.")    
        return ftree, dlist

    # Check if drive ok
    res = checkDrive(drive)
    if res[0] is not ErrCode.OK:
        return res
    
    # Initialise
    print(f"Retrieving file tree from `{drive}` :")
    ftree = []
    drive += "/" if drive[-1] != "/" else ""
    path = ""
    dlist = []
    isDone = False

    # Generate tree
    while not(isDone):
        print(f"  Parsing `{drive +path}` ...")
        fsubtree, dsublist = _getSubTree(drive, path)
        ftree += fsubtree
        dlist += dsublist
        if isDone := len(dlist) == 0:
            break
        path = dlist.pop(0)

    # Count folders and subfolders
    ndir = 0
    ntotal = len(ftree)
    for ln in ftree:    
        ndir += 1 if ln[1] is ElementType.IS_FOLDER else 0
    print(f"{ntotal -ndir} file(s) and {ndir} folder(s) found.")
    return [ErrCode.OK, "", ftree]
    
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
def _xmodemSend(fname :str) -> ErrCode:
    """ Send local file `fname` to the PicoMite using XMODEM. Returns True
        if transfer complete
    """            
    global xmodem_n_pkgs, xmodem_post    
    
    # Calculate number of packages
    fsize = Path(fname).stat().st_size
    xmodem_n_pkgs = round(fsize /XMODEM_PKG_SIZE)
    log(f"`{fname}` : Size: {fsize} bytes (= {xmodem_n_pkgs} packages)")

    try:    
        # Open local file and trigger transfer on PicoMite 
        stream = open(fname, "rb")
        cmd = f'xmodem r "{fname}'
        log(f'Send `{cmd}` ...')
        _ = sendCommand(cmd)
        time.sleep(XMODEM_WAIT_S)

        # Start transfer
        modem = XMODEM(_getc, _putc)
        modem.log.setLevel(logging.CRITICAL)
        xmodem_post = f"`{fname}`"
        _ = modem.send(
            stream, 
            retry=XMODEM_RETRY, quiet=0, 
            callback=_progressSend
        )
    finally:
        modem = None
        stream.close()

    print("\nGive PicoMite time to finish ... ", end="")
    time.sleep(XMODEM_WAIT_S)
        
    # Check if transferred file has the correct size
    fsize_pico = getFileSize(fname)
    print("incomplete" if fsize_pico < fsize else "complete", end="")
    print(f" ({fsize_pico} of {fsize} bytes).")
    return fsize_pico >= fsize


def _xmodemReceive(fname :str, path_local :str ="") -> ErrCode:
    """ Receive file `fname` on PicoMite using XMODEM. Returns True if
        transfer complete       
    """    
    global xmodem_n_pkgs, xmodem_post    
    
    # Calculate number of packages
    fsize = getFileSize(fname)
    assert fsize > 0
    xmodem_n_pkgs = round(fsize /XMODEM_PKG_SIZE)
    log(f"`{fname}` : Size: {fsize} bytes (= {xmodem_n_pkgs} packages)")

    # Extend path, if `path_local` is defined
    path = getAbsLocalPath(path_local)

    # Open local target file and trigger transfer from PicoMite 
    try:
        stream = open(path +fname, "wb")
    except FileNotFoundError:
        print(f"Error: file `{path +fname}` cannot be opened for writing.")
        return False   
    
    try:
        cmd = f'xmodem s "{fname}'
        log(f'Send `{cmd}` ...')
        _ = sendCommand(cmd)
        time.sleep(XMODEM_WAIT_S)

        # Start transfer
        modem = XMODEM(_getc, _putc)
        modem.log.setLevel(logging.CRITICAL)
        xmodem_post = f"`{fname}`"
        _ = modem.recv(
            stream, 
            retry=XMODEM_RETRY, quiet=1, 
            callback=_progressRecv
        )
    finally:
        modem = None
        stream.close()

    print("\nGive PicoMite time to finish ... ", end="")
    time.sleep(XMODEM_WAIT_S)
        
    # Check if transferred file has the correct size
    fsize_local = Path(fname).stat().st_size
    print("incomplete" if fsize_local < fsize else "complete", end="")
    print(f" ({fsize_local} of {fsize} bytes).")
    return fsize_local >= fsize


def _progressSend(total_packets :int, success_count :int, error_count :int):
    """ Callback for XMODEM send function
    """
    global xmodem_n_pkgs, xmodem_post
    #print(total_packets, success_count, error_count)    
    if xmodem_n_pkgs > 0:
        _print_progress_bar(success_count, xmodem_n_pkgs, xmodem_pre, xmodem_post)
    if success_count == xmodem_n_pkgs:
        xmodem_n_pkgs = 0


def _progressRecv(total_packets :int, success_count :int, error_count :int, packet_size: int):
    """ Callback for XMODEM recv function
    """
    global xmodem_n_pkgs, xmodem_post, xmodem_pkg_size
    assert xmodem_pkg_size == packet_size
    #print(total_packets, success_count, error_count, packet_size)
    if xmodem_n_pkgs > 0:
        _print_progress_bar(success_count, xmodem_n_pkgs, xmodem_pre, xmodem_post)
    if success_count == xmodem_n_pkgs:
        xmodem_n_pkgs = 0
    
# ---------------------------------------------------------------------------
# Commands
# ---------------------------------------------------------------------------
def _files(_args :list) -> tuple:
    """ List directory of given drive
    """
    # Check if drive is available and ready
    errMsg = checkDrive(_args.drive, doChange=False)
    if errMsg[0] != ErrCode.OK:
        return errMsg
    
    # Check if path exists
    path = _args.drive +"/" +_args.path
    if len(_args.path) > 0:
        res = checkFileExists(path)
        if res is not ElementType.IS_FOLDER:
            return (ErrCode.INVALID_PATH, path, [])

    # List files on that drive
    path += "/" if path[-1] != "/" else ""
    print(f"Content of `{path}` :")
    cmd = f'files "{path}"'
    log(f"Sending `{cmd}` ...")
    repl = sendCommand(cmd)
    if len(repl) > 0 and "error" in repl[0].lower():
        return (ErrCode.DRIVE_NOT_READY, repl[0][8:], [])

    _print(repl)
    return (ErrCode.OK, "", repl)


def _kill(_args :list) -> tuple:
    """ Kill file(s) on given drive
    """
    # Check if drive is available and ready
    errMsg = checkDrive(_args.drive, doChange=False)
    if errMsg[0] != ErrCode.OK:
        return errMsg

    # Kill file(s)
    pass


def _option_list() -> tuple:
    """ List options
    """
    print("Options :")
    cmd = "option list"
    log(f"Sending `{cmd}` ...")    
    repl = sendCommand(cmd)
    _print(repl[1:])
    return (ErrCode.OK, "", repl)

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
def _xmodem(_args :list, folder_local :str ="", doSend=True) -> tuple:
    """ Send or receive a file or files via XModem
    """
    global xmodem_pre 
    nFail = 0

    if len(_args.files) == 0:
        print("No file name(s) defined, nothing to do.")
        return (ErrCode.OK, "", [])

    if doSend:
        # Send files
        print("Transfering file(s) using XMODEM :")
        xmodem_pre = "Transfer"

        for fname in _args.files:
            # Check if file exists
            fname = _args.path +fname
            fobj = Path(fname)
            if not(fobj.is_file()):
                print(f"Error: File `{fname}` not found ... skipped.")
                nFail += 1
                continue

            # Do transfer ...        
            res = _xmodemSend(fname, folder_local)
            nFail += 0 if res else 1    

    else:
        # Retrieve files
        print("Receiving file(s) using XMODEM :")
        xmodem_pre = "Retrieve"    

        for fname in _args.files:
            # Check if file exists
            fname = _args.path +fname
            res = checkFileExists(fname)
            if res is not ElementType.IS_FILE:
                print(f"Error: File `{fname}` not found ... skipped.")
                nFail += 1                
                continue

            # Do transfer ...        
            res = _xmodemReceive(fname, folder_local)
            nFail += 0 if res else 1    

    if nFail > 0:
        print(f"Error : {nFail} of {len(_args.files)} transfers failed.")        

    return (ErrCode.OK, "", [])

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
def _backup(_args :list) -> tuple:
    """ Create a backup of the given drive
    """
    # Check if drive is available and ready
    errMsg = checkDrive(_args.drive, doChange=False)
    if errMsg[0] != ErrCode.OK:
        return errMsg

    # Check if a folder with `name` does not already exist
    if len(_args.name) == 0:
        return (
            ErrCode.PARAM_MISSING, 
            "`-n` as name of backup is required", 
            []
        )
    now = datetime.datetime.now().__str__().split(".")[0][:-2]
    stamp = now.replace(" ", "").replace(":", "").replace("-", "")
    path_local = _args.name +"_" +stamp
    path_local_abs = Path(path_local).resolve()
    if Path(path_local_abs).is_dir():
        return (
            ErrCode.FOLDER_EXISTS, 
            f"`{path_local_abs}` already exists, cannot overwrite existing backup",
            []
        )
    # Getting file tree
    ftree = getFileTree(_args.drive)

    # Getting ready
    print(f"Create backup of PicoMite @ `{_args.serial}` :")
    print(f"  Backup folder : `{path_local_abs}`")
    path = _args.drive
              
    return (ErrCode.OK, "", [])

# ---------------------------------------------------------------------------
if __name__ == "__main__":
  
    # Print version
    print(f"{PROG_NAME} v{PROG_VER}")

    # Read configuration from `.toml` file, if existent
    fname_toml = PROG_NAME.lower() +".toml"
    if Path(fname_toml).is_file():
        with open(fname_toml, "rb") as f:
            data = tomllib.load(f)
        COM_PORT = data["serial"]["com_port"]    
        COM_BAUDRATE = data["serial"]["com_baudrate"]    
        COM_TOUT_S = data["serial"]["com_timeout_s"]    
        XMODEM_RETRY = data["xmodem"]["xmodem_retry"]    
        XMODEM_WAIT_S = data["xmodem"]["xmodem_wait_s"]    
        XMODEM_PKG_SIZE = data["xmodem"]["xmodem_package_size_bytes"]    
        VERBOSE = data["debugging"]["verbose"]    

    # Get command line arguments and pre-process them
    args = getCmdLineArgs()
    args.serial = args.serial[0] if type(args.serial) is list else args.serial
    args.drive = args.drive[0] if type(args.drive) is list else args.drive
    args.name = args.name[0] if type(args.name) is list else args.name
    args.path = args.path[0] if type(args.path) is list else args.path
    if len(args.path) > 0:    
        args.path += "/" if args.path[-1] != "/" else ""
    args.command = args.command.lower()
    assert type(args.files) is list
    if len(args.files) == 0:
        args.files = []
    else:
        args.files = [s.strip() for s in args.files[0].split(",")]
    #print(args)

    # If the list ports command is issued, do this directly
    if args.command in ["p", "ports"]:
        # List available serial ports and exit
        ports = _listSerialPorts()
        print("Serial ports :")
        for p in ports:
            print("  " +p, end="")
        print()    
        cleanUp(ErrCode.OK, noClose=True)
        sys.exit()

    # Get serial port and check if PicoMite is responding
    Ser, SerIO = createSerialIO(args.serial, COM_BAUDRATE)
    if Ser is None:
        print(f"Error: Could not open port `{args.serial}`.")
        sys.exit()

    info = checkPicoMite(doDetailed=args.command in ["c", "check"])
    isConnected = not info == {}
    if not isConnected:
        print(f"Error: No PicoMite at `{args.serial}`.")
        cleanUp(ErrCode.NO_PICO_FOUND)
        sys.exit()
    
    # Process command
    if args.command in ["dummy"]:
        # Dummy command for testing ...
        res = getFileTree(args.drive)

    elif args.command in ["f", "files"]:
        # List directory of given drive
        res = _files(args)

    elif args.command in ["ol", "option list"]:
        # List options
        res = _option_list()    
        '''
    elif args.command in ["k", "kill"]:
        # Kill file(s) 
        res = _kill(args)        
        '''
    elif args.command in ["b", "backup"]:
        # Create backup of connected PicoMite
        res = _backup(args)

    elif args.command in ["xs", "xmodem s"]:
        # Send file (or files) via XModem
        res = _xmodem(args, doSend=True)

    elif args.command in ["xr", "xmodem r"]:
        # Send file (or files) via XModem
        res = _xmodem(args, doSend=False)

    elif args.command in ["c", "check"]:    
        # Return version information
        print(f"  Firmware      : {info['firmware']} @ `{args.serial}`")
        print(f"  Chip          : {info['chip']}")
        print(f"  MMBasic       : {info['version']}")
        print(f"  CPU frequency : {round(info['cpu_speed'] /1E6)} MHz")
        print(f"  Current drive : {info['drive']}")
        diskFree = round(info['free_disk_space'] /1000)
        diskTotal = round(info['total_disk_space'] /1000)
        percFree = round(diskFree /diskTotal *100)
        print(f"  Disk space    : {diskFree} of {diskTotal} kB free ({percFree}%)")
        res = [ErrCode.OK, "", []]

    else:
        # Command not recognized 
        res = [ErrCode.INVALID_CMD]

    # Handle error messages, if any
    if res[0] == ErrCode.INVALID_DRIVE:            
        print(f"Error: Invalid drive parameter (`{res[1]}`)")
    if res[0] == ErrCode.INVALID_PATH:
        print(f"Error: Invalid path (`{res[1]}`)") 
    elif res[0] == ErrCode.INVALID_CMD:
        print(f"Error: Command `{args.command}` not recognized")    
    elif res[0] != ErrCode.OK:            
        print(f"Error: `{res[1]}`")

    # Clean up
    cleanUp(res[0])

# ---------------------------------------------------------------------------