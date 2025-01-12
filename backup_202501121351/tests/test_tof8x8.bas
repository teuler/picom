' test_tof8x8.bas
'
Option base 0
Option explicit
Option Escape

Const USE_TOFF         = 1
Const USE_HW_UART      = 1

' - - - - - - - - - - - - - - - - - - - - - - - -  - - - - - - - - - - - -
' CLIENT-SPECIFIC DEFINITIONS
' - - - - - - - - - - - - - - - - - - - - - - - -  - - - - - - - - - - - -
' Pico pins
Const PIN_TOF_RX       = MM.Info(PinNo GP1) ' COM2 as #1
Const PIN_TOF_TX       = MM.Info(PinNo GP0)

' TOF sensor definitions
' Data format (74 bytes):
'   bytes  0..3 : frame index (uint16, hexlified)
'           4,5 : dx, dy (uint8, binary) +DATA_OFFS
'         6..69 : pixels as distance in [cm] +DATA_OFFS (for 8x8)
'        70..79 : to be used later
'         80,81 : `\r\n`, which are remmoved by `Line Input`
'   with DATA_OFFS = 14 to avoid that escape characters (e.g., `\r`)
'   appear in the data section of a package.
' Pixel values of:
'   0      : invalid measurement
'   1..127 : distance in [mm]
'            most significant bit set = extrapolated distance
'
Const TOF_COM          = 1
Const TOF_BAUD         = 115200
Const TOF_FR_DX        = 8
Const TOF_FR_DY        = 8
Const TOF_N_FR         = TOF_FR_DX *TOF_FR_DY
Const TOF_N_DATA       = 6 +TOF_N_FR +10
Const TOF_DATA_OFFS    = 14
Const TOF_TILT_DEG     = 45

' TOF sensor-related variables and setup
Dim integer tof.dx, tof.dy, tof.len, tof.iFr
Dim integer tof.fr(TOF_FR_DX-1, TOF_FR_DY-1)
Dim integer tof.isReady = 0, tof.newData = 0
Dim float tof.t_last = 0
Dim integer running = 1
Dim integer i, j, v, p, ch
Dim integer offs, cx, cy, bdx, colf, colb

' Setup screen
CLS
cx = Int(MM.HRes /2)
cy = Int(MM.VRes /2)
bdx = 15
offs = -Int(TOF_FR_DX /2)
colb = RGB(0,0,0)

' Start TOF sensor ranging
TOF.Open
TOF.start

' Main loop
' ---------
Do While running
  't_loop = Timer

  If tof.t_last = 0 Or Timer -tof.t_last > 3000 Then
    Print "| Warning: No frames received recently, restart ranging ..."
    TOF.start
    tof.t_last = Timer +1000
  EndIf

  If Not(USE_HW_UART) Then TOF._read

  ' Update TOF sensor data
  If tof.newData Then
    'Print tof.iFr
    For i=0 To TOF_FR_DX-1
      For j=0 To TOF_FR_DY-1
        'Print Str$(tof.fr(j,i),4);
        v = tof.fr(j,i)
        If v = 255 Then
          colf = RGB(64,0,0)
        Else
          Inc v, -TOF_DATA_OFFS
          colf = RGB(0, (127 -v)*2, 0)
        EndIf
        Box (i +offs)*bdx +cx, (j +offs)*bdx +cy, bdx,bdx,,colb, colf
      Next
      'Print
    Next
    'Print
    tof.t_last = Timer
    tof.newData = 0
  EndIf

  ch = Asc(LCase$(Inkey$))
  If ch = 27 Then running = 0 : Exit : EndIf
Loop

TOF.Close
End

' ===========================================================================
' TOF sensor subroutines
' ---------------------------------------------------------------------------
Sub TOF.Open
  ' Open serial connection to TOF unit (a Tiny2040 w/ a VL53L5CX)
  If Not(USE_TOFF) Then Exit Sub
  If USE_HW_UART Then
    Local string s$ = "COM" +Str$(TOF_COM), tmp$
    Print "| Opening " +s$ +" to TOF unit ..."
    SetPin PIN_TOF_RX, PIN_TOF_TX, COM1
    tmp$ = s$ +":"+Str$(TOF_BAUD)+",512,TOF._read,"+Str$(TOF_N_DATA)
    Open tmp$ As #1
  Else
    Print "| Using `Device Serialxx` to TOF unit ..."
  EndIf
  tof.isReady = 1
End Sub


Sub TOF._sendCmd cmd$, t_wait_ms
  If USE_HW_UART Then
    Print #1, cmd$
  Else
    Device SerialTX PIN_TOF_TX, TOF_BAUD, cmd$+"\r\n"
  EndIf
  If t_wait_ms > 0 Then Pause t_wait_ms
End Sub


Sub TOF.start
  ' Start ranging ...
  If Not(USE_TOFF) Then Exit Sub
  TOF._sendCmd "A"+Str$(TOF_TILT_DEG,3), 500
  TOF._sendCmd "R000"
End Sub


Sub TOF.stop
  ' Stop ranging
  If Not(USE_TOFF) Then Exit Sub
  TOF._sendCmd "S000", 500
End Sub


Sub TOF.Close
  ' Stops ranging and closes serial port to TOF unit
  If Not(USE_TOFF) Then Exit Sub
  Print "| Stop ranging and close connection to TOF."
  TOF.stop
  If USE_HW_UART Then
    Close #1
  EndIf
End Sub


Sub TOF._read
  ' Check TOF if new data is available, if so, sets `tof.newData` = 1
  ' and fills `tof.fr()` with pixel data (distance in cm; 255=invalid)
  'Static float t
  Static integer first = 1, pbuf, v(TOF_N_FR/8 -1), pv, w(TOF_N_FR-1)
  Static String buf$ length TOF_N_DATA+2
  't = Timer
  If first Then
    pbuf = Peek(VarAddr buf$)
    pv = Peek(VarAddr v())
    first = 0
  EndIf
  tof.newData = 0
  tof.len = 0

  If Not(tof.isReady) Then Exit Sub
  If USE_HW_UART Then
    If Loc(#1) >= TOF_N_DATA Then
      Line Input #1, buf$
      tof.len = Peek(Byte pbuf)
    EndIf
  Else
    Local integer status%
    Device SerialRX PIN_TOF_RX, TOF_BAUD, buf$, 500, status%,,"\r"
    Print status%, Peek(Byte pbuf)
    tof.len = Peek(Byte pbuf) -2
  EndIf

  If tof.len = TOF_N_DATA Then
    ' Complete tof distance data frame received
    tof.iFr = Val("&H"+Mid$(buf$,1,4))
    tof.dx  = Peek(Byte pbuf +5) -TOF_DATA_OFFS
    tof.dy  = Peek(Byte pbuf +6) -TOF_DATA_OFFS
    Memory Copy pbuf+7, pv, TOF_N_FR
    Memory Unpack v(), w(), TOF_N_FR, 8
    Math Add w(), 0, tof.fr()
    'Math Scale w(), 1, tof.fr()
    'Print "iFr=";tof.iFr;
    'Print " Data len=";tof.len;" (";tof.dx;"x";tof.dy;")"
    'Math M_print tof.fr()
    tof.newData = 1
  EndIf
  't = Timer -t
  'Print t
End Sub

' ============================================================================                                                                