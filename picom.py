##!/usr/bin/env python
# -*- coding: utf-8 -*-
# ---------------------------------------------------------------------------
# picom.py
# A file synchronisation tool for PicoMite. 
# For release notes, see https://github.com/teuler/picom/blob/main/README.md
#
# The MIT License (MIT)
# Copyright (c) 2025 Thomas Euler
# ---------------------------------------------------------------------------
import serial
import io
import os
import sys
import time
import textwrap
import argparse
import tomllib
import logging
import datetime
import platform
import serial.tools.list_ports as serial_p
from pathlib import Path
from xmodem import XMODEM

PROG_NAME      = "PicoM"
PROG_VER       = "0.1.7 (beta)"

MASK_OPT_TXT   = "{0}_options.txt"
MASK_FTREE_TXT = "{0}_filetree.txt"
FILE_EXT_OPT   = ".opt"
FILE_EXT_LIB   = ".lib"

# ---------------------------------------------------------------------------
# Default configuration (replaced by content of `picom.toml`)
#
COM_PORT        = "COM5"
COM_BAUDRATE    = 921_600
COM_TOUT_S      = 0.2

XMODEM_RETRY    = 8
XMODEM_WAIT_S   = 1.5
XMODEM_PKG_SIZE = 128

VERBOSE         = False
ASK_QUESTIONS   = True

# ---------------------------------------------------------------------------
class ErrCode:
    OK              = 0
    USER_ABORT      = 1
    INVALID_DRIVE   = 10
    DRIVE_NOT_READY = 11
    B_NOT_ENABLED   = 12
    INVALID_PATH    = 13
    FOLDER_EXISTS   = 14
    FOLDER_MISSING  = 15
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
    return SerIO.read(size) or None

def _putc(data, timeout=1):
    return SerIO.write(data) 

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
def createSerialIO(_port :str, _baudrate :int, doCtrlC=True) -> tuple:
    """ Open and return a serial port instance
    """ 
    try:
        # Open serial port
        serIO = serial.Serial(_port, baudrate=_baudrate, timeout=COM_TOUT_S)

        # Send Ctrl-C to interrupt any running program
        if doCtrlC:
            serIO.write(0x03)
            serIO.write(0x43)            

    except serial.serialutil.SerialException:
        serIO = None   

    return serIO


def _reopenSerialIO(_args :list):
    global SerIO, picoID
    SerIO.close()
    print("  Re-open serial port ...")
    time.sleep(0.5)

    if platform.system().lower() == "linux":
        # Under Linux, the COM port may change ...
        print(f"  Checking for Pico with ID `{picoID}` ...")
        _dev = _listSerialPorts(verbose=False)
        success = False
        for d in _dev:
            if "pico" in d[1].lower():
                # Is Pico but is it the correct one?
                _port = d[0]
                SerIO = createSerialIO(_port, COM_BAUDRATE, doCtrlC=False)
                res = sendCommand("?MM.INFO(ID)")
                if len(res) > 0:
                    print(f"  Found Pico @ `{_port}` with ID `{res[0]}` ...")
                    success = res[0] == picoID
                    if success:
                        # Pico identified by its ID
                        print("  Successfully reconnected.")
                        _args.serial = d[0]
                        break
                SerIO.close()

        if not success:
            print("Error: Could not reconnect to Pico")           
            sys.exit() 

    else:
        # With Windows, the COM port is usually the same ...
        SerIO = createSerialIO(_args.serial, COM_BAUDRATE)


def _listSerialPorts(verbose :bool =True) -> list:
    """ Return list of serial ports that can be opened
    """
    tmp = serial_p.comports()
    ports = []
    if len(tmp) == 0:
        print("Error: No serial ports found.")
    else:
        if verbose:
            print(f"{len(tmp)} serial port(s) found :")
        for p in tmp:
            print(f"  `{p.device}`, {p.description}")
            ports.append([p.device, p.description])
    return ports
    

def cleanUp(errC :ErrCode, noClose :bool =False):
    """ Close ports etc.
    """
    if not noClose:
        SerIO.close()
    if errC == ErrCode.OK:
        log("Done.") 

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
def sendCommand(
        _cmd :str, doPrint :bool =False, doDot :bool =False, 
        doWait_ms :int = 10
    ) -> list:
    """ Sends a command via the serial port to the Pico and returns the 
        REPL output as a list of strings; prints the reply, if `doPrint`
    """
    # Send command
    cmd = _cmd
    try:
        SerIO.write((cmd +"\n").encode())    
    except serial.serialutil.SerialTimeoutException as err:  
        print(f"Error: {err}")
        return []

    time.sleep(doWait_ms /1_000)
    if doDot:
        print(".", end="")
    
    # Retrieve output
    done = False
    repl = []
    n = m = 0
    while not(done):
        res = SerIO.readline()
        res = res.decode()
        if not(done := len(res) == 0):
            # Filter out lines starting with `>` and the first line (which
            # mirrors the command), and add the lines to a list
            n += 1
            res = res[:-2]
            if len(res) > 0 and res[0] != ">" and res != cmd:
                repl.append(res)
                m += 1

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
    """ Print progress bar
    """
    n_bar = 50  
    progress = index /total
    sys.stdout.write('\r')
    sys.stdout.write(
        f"{pre} [{'=' *int(n_bar *progress):{n_bar}s}] {int(100 *progress)}% {post}"
    )
    sys.stdout.flush()


def _yesno(question :str, doDeleteLn=False):
    """ Ask the user a yes/no question
    """
    repl = input(f"{question} (y/n)? ").lower() == 'y'
    if doDeleteLn:
        print('\033[1A' + '\x1b[2K', end="")
    return repl    

# ---------------------------------------------------------------------------
# Retrieving general information from the PicoMite
# ---------------------------------------------------------------------------
def isPicoMitePresent() -> str:
    """ Check if PicoMite is responding, returns "" if not, else returns
        the unique ID
    """
    log(f"Checking for PicoMite ... ", noLF=True)
    res = sendCommand("?MM.INFO(ID)")
    log("done.", noHeader=True)
    return res[0] if len(res[0]) > 0 else ""


def getPicoMite() -> dict:
    """ Get version information from PicoMIte
    """
    print("Retrieving key data from PicoMite ", end="")
    ver = dict()
    res = sendCommand("option list", doDot=True)
    if len(res) > 0:
        tmp = res[0].split()
        if "picomite" in tmp[0].lower():
            ver.update({
                "firmware": tmp[0], 
                "chip": tmp[2], "version": tmp[4]
            })
    res = sendCommand("?MM.INFO$(CPUSPEED)", doDot=True)
    ver.update({"cpu_speed": int(res[-1])}) 
    res = sendCommand("?MM.DEVICE$", doDot=True)
    ver.update({"device": res[-1]}) 
    res = sendCommand("?MM.INFO$(DRIVE)", doDot=True)
    ver.update({"drive": res[-1]}) 
    res = sendCommand("?MM.INFO(FREE SPACE)", doDot=True)
    ver.update({"free_disk_space": int(res[-1])}) 
    res = sendCommand("?MM.INFO(DISK SIZE)", doDot=True)
    ver.update({"total_disk_space": int(res[-1])})             
    res = sendCommand("?MM.INFO(ID)", doDot=True)
    ver.update({"ID": res[-1]})      
    print(" done.")       
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
    try:
        if int(repl[-1].strip()) == 1:
            return ElementType.IS_FILE
        repl = sendCommand(f'?MM.INFO(EXISTS DIR "{fname}")')
        if int(repl[-1].strip()) == 1:    
            return ElementType.IS_FOLDER
        
    except ValueError:
        log(f"Internal error in `checkFileExists(`{fname}`)` : repl[-1] == `{repl[-1]}`")    

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
    return Path(path_local).resolve().__str__()

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
def getFileTree(drive :str, verbose :bool =True) -> list:
    """ Returns the complete file tree on `drive` as a list of paths in the
        form  `[errCode, msg, pathlist]`
    """
    def _getSubTree(drive :str, path :str) -> list:
        ftree = []
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
                _new_path = path +tmp[1] +"/"
                ftree.append([_new_path, ElementType.IS_FOLDER])
                res = _getSubTree(drive, _new_path)    
                if len(res) > 0:
                    ftree += res

            elif len(tmp) == 4:
                # Is file
                ftree.append([path +tmp[-1], ElementType.IS_FILE])

            else:
                print(f"Warning: Entry #{i} (`{ln}`) not recognized.")    

        return ftree


    # Check if drive ok
    res = checkDrive(drive)
    if res[0] is not ErrCode.OK:
        return res
    
    # Initialise
    print(f"Retrieving file tree from `{drive}` :")
    drive += "/" if drive[-1] != "/" else ""
    path = ""

    # Generate tree
    if verbose:
        print(f"  Parsing `{drive +path}` ...")
    ftree = _getSubTree(drive, path)

    # Count folders and subfolders
    ndir = 0
    ntotal = len(ftree)
    for ln in ftree:    
        ndir += 1 if ln[1] is ElementType.IS_FOLDER else 0
    print(f"{ntotal -ndir} file(s) and {ndir} folder(s) found.")
    return [ErrCode.OK, "", ftree]


def getFileTreeLocal(path_local :str) -> list:
    """ Returns the complete local file tree in `path` as a list of paths 
        in the form  `[errCode, msg, pathlist]`
    """
    def _getSubTreeLocal(_path :str) -> list:
        ftree = []
        dlist = []
        ftree_sub = list(Path(_path).iterdir())
        for ln in ftree_sub:
            if ln.is_dir():
                dir = ln.__str__()
                ftree.append([dir, ElementType.IS_FOLDER])
                dlist.append(dir)
            else:
                ftree.append([ln.__str__(), ElementType.IS_FILE])    
        return ftree, dlist    
    
    # Initialise
    print(f"Retrieving file tree from `{path_local}` :")
    ftree = []
    path = path_local
    dlist = []

    # Generate tree
    while True:
        print(f"  Parsing `{path}` ...")
        fsubtree, dsublist = _getSubTreeLocal(path)
        ftree += fsubtree
        dlist += dsublist
        if len(dlist) == 0:
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
def _xmodemSend(fname :str, _path_local :str ="") -> bool:
    """ Send local file `fname` to the PicoMite using XMODEM. Returns True
        if transfer complete
    """            
    global xmodem_n_pkgs, xmodem_post    

    # Extend path, if `path_local` is defined
    path_local = getAbsLocalPath(_path_local)
    fname_local = Path(path_local).joinpath(Path(fname))
    '''
    print("fname_local", fname_local)
    print("fname", fname)
    '''

    # Calculate number of packages
    fsize = Path(fname_local).stat().st_size
    n_pkgs = round(fsize /XMODEM_PKG_SIZE)
    xmodem_n_pkgs = n_pkgs
    log(f"`{fname}` : Size: {fsize} bytes (= {xmodem_n_pkgs} packages)")

    # Open local file and trigger transfer on PicoMite 
    try:
        stream = open(fname_local, "rb")
    except FileNotFoundError:
        print(f"Error: file `{fname_local.__str__()}` cannot be opened for reading.")
        return False   
    
    try:    
        # Check if `fname` contains folders and if so, if the folders
        # exist on the PicoMite
        tmp = fname.rsplit("/", 1)
        if len(tmp) > 1:
            if not checkFileExists(tmp[0]) == ElementType.IS_FOLDER:
                # Create folder
                log(f"Create folder `{tmp[0]}` ...")
                cmd = f'mkdir "{tmp[0]}"'
                log(f'Send `{cmd}` ...')
                _ = sendCommand(cmd)

        # Prepare transfer ...
        cmd = f'xmodem r "{fname}"'
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

    # Give PicoMite time to finish transfer ...
    _wait_xmodem(n_pkgs, XMODEM_WAIT_S)    
        
    # Check if transferred file has the correct size
    fsize_pico = getFileSize(fname)
    print(" - incomplete" if fsize_pico < fsize else " - complete", end="")
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
    n_pkgs = round(fsize /XMODEM_PKG_SIZE)
    xmodem_n_pkgs = n_pkgs
    log(f"`{fname}` : Size: {fsize} bytes (= {xmodem_n_pkgs} packages)")

    # Extend path, if `path_local` is defined
    #print("path_local", path_local)
    #print("fname", fname)
    path = getAbsLocalPath(path_local)
    #print("path", path)

    # Open local target file and trigger transfer from PicoMite 
    try:
        stream = open(Path(path, fname), "wb")
    except FileNotFoundError:
        print(f"Error: file `{path +fname}` cannot be opened for writing.")
        return False   
    
    try:
        cmd = f'xmodem s "{fname}"'
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

    # Give PicoMite time to finish transfer ...
    _wait_xmodem(n_pkgs, XMODEM_WAIT_S)
     
    # Check if transferred file has the correct size
    fsize_local = Path(path, fname).stat().st_size
    print(" - incomplete" if fsize_local < fsize else " - complete", end="")
    print(f" ({fsize_local} of {fsize} bytes).")
    return fsize_local >= fsize


def _wait_xmodem(n_curr :int, wait_s :float):
    """ Wait while showing dots
    """
    global xmodem_pre, xmodem_post    
    n_max = max(int(n_curr *1.1), 2)
    n_dt = n_max -n_curr +1
    dt = wait_s /n_dt
    for i in range(n_dt):
        time.sleep(dt)    
        _print_progress_bar(
            n_curr +i, n_max, 
            xmodem_pre, xmodem_post
        )


def _progressSend(total_packets :int, success_count :int, error_count :int):
    """ Callback for XMODEM send function
    """
    global xmodem_n_pkgs, xmodem_post
    #print(total_packets, success_count, error_count)    
    if xmodem_n_pkgs > 0:
        _print_progress_bar(
            success_count, max(int(xmodem_n_pkgs *1.1), 2), 
            xmodem_pre, xmodem_post
        )
    if success_count == xmodem_n_pkgs:
        xmodem_n_pkgs = 0


def _progressRecv(total_packets :int, success_count :int, error_count :int, packet_size: int):
    """ Callback for XMODEM recv function
    """
    global xmodem_n_pkgs, xmodem_post, xmodem_pkg_size
    assert xmodem_pkg_size == packet_size
    #print(total_packets, success_count, error_count, packet_size)
    if xmodem_n_pkgs > 0:
        _print_progress_bar(
            success_count, max(int(xmodem_n_pkgs *1.1), 2), 
            xmodem_pre, xmodem_post
        )
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


def _filetree(_args :list) -> tuple:
    """ List filetree of given drive
    """
    # Check if drive is available and ready
    errMsg = checkDrive(_args.drive, doChange=False)
    if errMsg[0] != ErrCode.OK:
        return errMsg
    
    # Retrieve filetree
    repl = getFileTree(_args.drive, verbose=False)
    if repl[0] != ErrCode.OK:
        return repl
    
    # Present filetree
    ftree = repl[2]
    flev = 0
    iLast = len(ftree) -1

    print(f"Content of `{_args.drive}` :")
    for i, ln in enumerate(ftree):
        entry = ln[0].strip("/")
        isLast = i == iLast
        flev = len(entry.split("/")) -1
        endFolder = False
        if i < iLast:
            next_entry = ftree[i+1][0].strip("/")
            #print(next_entry, len(next_entry.split("/")) -1, flev)
            endFolder = (len(next_entry.split("/")) -1) < flev 

        if ln[1] is ElementType.IS_FILE:
            head = "│  " *flev
            head += "└──" if isLast or endFolder else "├──"
            fname = entry.split("/")[-1]
            #print(f"{head}{entry} ({fname})")
            print(f"{head}{fname}")

        elif ln[1] is ElementType.IS_FOLDER:
            head = "│  " *flev
            head += "└──" if isLast else "├──"
            dname = entry.split("/")[-1]
            #print(f"{head}{entry} ({dname})")
            print(f"{head}{dname}")

        else:
            print("?")    

        last_flev = flev
    '''
    "├"
    "─"
    "└"
    '''    

    return (ErrCode.OK, "", [])


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
    nSkipped = 0

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
                # Source file not found, skip
                print(f"Error: File `{fname}` not found ... skipped.")
                nFail += 1
                continue
    
            if ASK_QUESTIONS and checkFileExists(fname) == ElementType.IS_FILE:
                # Target file already exists, overwrite?
                if not(_yesno(f"File `{fname}` already exists - overwrite", doDeleteLn=True)):
                    # Skip file
                    print(f"File `{fname}` skipped by user.")
                    nSkipped += 1
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

            fobj = Path(fname)
            if ASK_QUESTIONS and fobj.is_file():
                # Source file exists, overwrite?
                if not(_yesno(f"File `{fname}` already exists - overwrite", doDeleteLn=True)):
                    # Skip file
                    print(f"File `{fname}` skipped by user.")
                    nSkipped += 1
                    continue

            # Do transfer ...        
            res = _xmodemReceive(fname, folder_local)
            nFail += 0 if res else 1    

    if nFail > 0:
        print(f"Error : {nFail} of {len(_args.files)} transfer(s) failed.")        
    if nSkipped > 0:
        print(f"{nSkipped} of {len(_args.files)} transfer(s) skipped by user.")        

    return (ErrCode.OK, "", [])

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
def _backup(_args :list) -> tuple:
    """ Create a backup of the given drive
    """
    global xmodem_pre

    # Check if drive on PicoMite is available and ready
    log("Check drive ...")
    errMsg = checkDrive(_args.drive, doChange=False)
    if errMsg[0] != ErrCode.OK:
        return errMsg

    # Check if a local folder with `name` does not already exist
    if len(_args.name) == 0:
        return (
            ErrCode.PARAM_MISSING, 
            "`-n` as name of backup is required", 
            []
        )
    
    # Create a unique local folder for the backup
    log("Create unique local folder for backup ...")
    now = datetime.datetime.now().__str__().split(".")[0][:-2]
    stamp = now.replace(" ", "").replace(":", "").replace("-", "")
    path_local = _args.name +"_" +stamp
    path_local_abs_obj = Path(path_local).resolve()
    if path_local_abs_obj.is_dir():
        return (
            ErrCode.FOLDER_EXISTS, 
            f"`{path_local_abs_obj}` already exists, cannot overwrite existing backup",
            []
        )
    path_local_abs_obj.mkdir()

    # Get PicoMite options and save as a file in the backup folder
    log("Get option list ...")
    cmd = "option list"
    log(f"Sending `{cmd}` ...")    
    repl = sendCommand(cmd)
    tmp = path_local_abs_obj.__str__() +"/" +MASK_OPT_TXT.format(stamp)
    with open(tmp, "w") as f:
        for ln in repl[1:]:
            f.write(ln +"\n")

    # Create a options file on the PicoMite
    fname_opt = stamp +FILE_EXT_OPT
    print(f'  Save options to "{fname_opt}" file ...')
    cmd = f'option disk save "{fname_opt}"'
    log(f"Sending `{cmd}` ...")    
    _ = sendCommand(cmd)

    # Check if library is used and if so, create library file on Picomite
    fname_lib = stamp +FILE_EXT_LIB
    print(f"  Save library, if exists, to `{fname_lib}` ...")    
    cmd = f'library disk save "{fname_lib}"'
    log(f"Sending `{cmd}` ...")    
    repl = sendCommand(cmd, doWait_ms=200)
    if len(repl) > 0 and "error" in repl[0].lower():
        log("No library found")

    # Get file tree
    errC, msg, ftree = getFileTree(_args.drive)
    if errC != ErrCode.OK:
        return (errC, msg, [])

    # Write file tree as a file list into a local text file
    tmp = path_local_abs_obj.__str__() +"/" +MASK_FTREE_TXT.format(stamp)
    with open(tmp, "w") as f:
        for ln in ftree:
            f.write(ln[0] +"," +str(ln[1]) +"\n")

    # Getting ready
    print(f"Create backup of PicoMite @`{_args.serial}` :")
    print(f"  Backup folder : `{path_local_abs_obj}`")
    nFail = 0
    nFolders = 0
    nFiles = 0

    # Go through file tree ...
    for ln in ftree:
        if ln[1] == ElementType.IS_FOLDER:
            # Create folder if needed
            pobj = Path(path_local_abs_obj.__str__() +"/" +ln[0])
            pobj.mkdir(exist_ok=False)
            print(f"Create local folder `{pobj.__str__()}`")
            nFolders += 1

        else:
            # Check if file exists
            fname = ln[0]
            res = checkFileExists(fname)
            if res is not ElementType.IS_FILE:
                print(f"Error: File `{fname}` not found ... skipped.")
                nFail += 1                
                continue

            # Do transfer ...        
            xmodem_pre = "Backup"     
            res = _xmodemReceive(fname, path_local_abs_obj.__str__())
            nFail += 0 if res else 1  
            nFiles += 1

    # Finish
    if nFail == 0:
        print(f"Backup `{path_local}` successfully created.")
    print(f"  {nFiles} file(s) and {nFolders} folder(s) saved; {nFail} failed")    

    return (ErrCode.OK, "", [])

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
def _restore(_args :list, info :dict) -> tuple:
    """ Restore a backup of the given drive
    """
    global xmodem_pre

    # Check if drive on PicoMite is available and ready
    log("Check drive ...")
    errMsg = checkDrive(_args.drive, doChange=False)
    if errMsg[0] != ErrCode.OK:
        return errMsg

    # Check if a local backup folder `name` exists
    log("Locate backup ...")
    if len(_args.name) == 0:
        return (
            ErrCode.PARAM_MISSING, 
            "`-n` as name of backup is required", 
            []
        )
    path_local_abs_obj = Path(_args.name).resolve()
    if not(path_local_abs_obj.is_dir()):
        return (
            ErrCode.FOLDER_MISSING, 
            f"`Backup folder {path_local_abs_obj}` not found",
            []
        )
    # Get date-time stamp, assuming it its the end of the backup name;
    # get also names of backup management files for excluding later from
    # the restore ... 
    tmp = _args.name.split("_")
    if len(tmp) > 1:
        stamp = tmp[1]
        ignore_list = [
            MASK_OPT_TXT.format(stamp).lower(),
            MASK_FTREE_TXT.format(stamp).lower()
        ]
    else:
        print("Warning: Backup name does not contain time stamp.")    
        print("  Note that options and library cannot be automatically restored.")    
        ignore_list = []
        stamp = "unknown"

    # Get filetree from local backup folder
    errC, msg, ftree = getFileTreeLocal(path_local_abs_obj.__str__())
    if errC != ErrCode.OK:
        return (errC, msg, [])
        
    # Everything is ready, ask user if to continue ...
    path_local_backup = path_local_abs_obj.__str__()
    print(f"Restoring backup `{path_local_backup}` to `{info['firmware']}` @`{_args.serial}` :")
    if ASK_QUESTIONS:
        if not(_yesno("Continue")):
            return (ErrCode.USER_ABORT, "", [])        

    # Kill everything before backup?
    if _yesno(f"Kill everything on drive `{_args.drive}`"):
        if _yesno("Really?!?", doDeleteLn=True):
            # Kill drive ...
            print(f"  Formatting drive `{_args.drive}` ... ")
            cmd = f'drive "{_args.drive}/FORMAT"'
            log(f"Sending `{cmd}` ...")    
            repl = sendCommand(cmd)
            time.sleep(1.5)

    # Restore ...
    nFail = 0
    nFolders = 0
    nFiles = 0
    for ln in ftree:
        # Get path relative to backup folder, this will be the path on the
        # Picomite
        pobj = Path(ln[0]).relative_to(path_local_abs_obj)
        #pobj_local = Path(ln[0])
        #path_backup = pobj_local.
        fname = pobj.__str__().replace("\\", "/")

        if ln[1] == ElementType.IS_FILE:
            fname_only = pobj.name.lower()
            if fname_only not in ignore_list:
                # Restore file ...
                xmodem_pre = "Restore"     
                res = _xmodemSend(fname, _path_local=path_local_backup)
                nFail += 0 if res else 1  
                nFiles += 1                
            
        elif ln[1] == ElementType.IS_FOLDER:
            # Make folder ...
            nFolders += 1
            if not checkFileExists(fname) == ElementType.IS_FOLDER:
                # Create folder
                    print(f"Make folder `{fname}` ...")
                    cmd = f'mkdir "{fname}"'
                    log(f'Send `{cmd}` ...')
                    _ = sendCommand(cmd)
        else:
            print(f"Error: Unknown element")                     
            nFail += 1

    # Restore PicoMite options, if stored and requested
    fname_opt = stamp +FILE_EXT_OPT
    pobj = path_local_abs_obj.joinpath(Path(fname_opt))
    if pobj.is_file():
        # Option file exists, restore as well?
        if not(ASK_QUESTIONS) or _yesno("Restore PicoMite options"):
            print("  Reset options ...")
            cmd = 'option reset'
            log(f"Sending `{cmd}` ...")    
            _ = sendCommand(cmd)
            
            time.sleep(1.5)
            _reopenSerialIO(_args)    
            time.sleep(1.5)

            print(f"  Restore options from `{fname_opt}` ...")
            cmd = f'option disk load "{fname_opt}"'
            log(f"Sending `{cmd}` ...")    
            _ = sendCommand(cmd, doWait_ms=200)

            time.sleep(1.5)
            _reopenSerialIO(_args)            
            time.sleep(1.5)
            print("done.")

    # Check if library file with backup's name and restore, if requested
    fname_lib = stamp +FILE_EXT_LIB
    if checkFileExists(fname_lib) == ElementType.IS_FILE:
        if not(ASK_QUESTIONS) or _yesno(f"Restore library from `{fname_lib}`"):
            cmd = f'library disk load "{fname_lib}"'
            log(f"Sending `{cmd}` ...")    
            _ = sendCommand(cmd)
            print("done.")

    # Finish
    if nFail == 0:
        print(f"Backup `{path_local_backup}` successfully restored.")
    print(
        f"  {nFiles} file(s) copied, {nFolders} folder(s) created; "
        f"{nFail} failed\n"    
        f"  {len(ignore_list)} backup management files ignored"    
    )
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
        ASK_QUESTIONS = data["debugging"]["ask_questions"]    

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
        _ = _listSerialPorts(args)
        cleanUp(ErrCode.OK, noClose=True)
        sys.exit()

    # Get serial port and check if PicoMite is responding
    SerIO = createSerialIO(args.serial, COM_BAUDRATE)
    if SerIO is None:
        print(f"Error: Could not open port `{args.serial}`.")
        sys.exit()

    picoID = isPicoMitePresent()
    isConnected = len(picoID) > 0
    if not isConnected:
        print(f"Error: No PicoMite at `{args.serial}`.")
        cleanUp(ErrCode.NO_PICO_FOUND)
        sys.exit()


    # Process command
    if args.command in ["dummy"]:
        # Dummy to test new commands
        _reopenSerialIO(args)
        res = _files(args)

    elif args.command in ["ft", "filetree"]:
        # List complete filetree of given drive
        res = _filetree(args)

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
        # Create a backup of connected PicoMite
        res = _backup(args)

    elif args.command in ["r", "restore"]:
        # Restore a backup to the connected PicoMite
        info = getPicoMite()
        res = _restore(args, info)

    elif args.command in ["xs", "xmodem s"]:
        # Send file (or files) via XModem
        res = _xmodem(args, doSend=True)

    elif args.command in ["xr", "xmodem r"]:
        # Send file (or files) via XModem
        res = _xmodem(args, doSend=False)

    elif args.command in ["c", "check"]:    
        # Return version information
        info = getPicoMite()
        print(f"  Firmware      : {info['firmware']} @`{args.serial}`")
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
    if res[0] == ErrCode.USER_ABORT:
        print("Aborted by user")
    elif res[0] == ErrCode.INVALID_DRIVE:            
        print(f"Error: Invalid drive parameter (`{res[1]}`)")
    elif res[0] == ErrCode.INVALID_PATH:
        print(f"Error: Invalid path (`{res[1]}`)") 
    elif res[0] == ErrCode.INVALID_CMD:
        print(f"Error: Command `{args.command}` not recognized")    
    elif res[0] != ErrCode.OK:            
        print(f"Error: `{res[1]}`")

    # Clean up
    cleanUp(res[0])

# ---------------------------------------------------------------------------