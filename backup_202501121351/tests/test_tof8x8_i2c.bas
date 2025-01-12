' test_tof8x8_i2c.bas
'
Option base 0
Option explicit
Option Escape

Const USE_TOFF         = 1

' - - - - - - - - - - - - - - - - - - - - - - - -  - - - - - - - - - - - -
' CLIENT-SPECIFIC DEFINITIONS
' - - - - - - - - - - - - - - - - - - - - - - - -  - - - - - - - - - - - -
' Pico pins (I2C)
Const PIN_TOF_SDA      = MM.Info(PinNo GP1) ' I2C0
Const PIN_TOF_SCL      = MM.Info(PinNo GP0)

' TOF sensor definitions
' Data format (74 bytes):
'   bytes  0..7 : frame index (uint32, hexlified)
'           8,9 : dx, dy (uint8, binary) +DATA_OFFS
'        10..73 : pixels as distance in [cm] +DATA_OFFS (for 8x8)
'         74,75 : `\r\n`, which are remmoved by `Line Input`
'   with DATA_OFFS = 14 to avoid that escape characters (e.g., `\r`)
'   appear in the data section of a package.
' Pixel values of:
'   255 : invalid measurement
'
Const TOF_FR_DX        = 8
Const TOF_FR_DY        = 8
Const TOF_N_FR         = TOF_FR_DX *TOF_FR_DY
Const TOF_N_DATA       = 4 +TOF_N_FR +12
Const TOF_TILT_DEG     = 45

' TOF sensor-related variables and setup
Dim integer tof.dx, tof.dy, tof.len, tof.iFr
Dim integer tof.fr(TOF_FR_DX-1, TOF_FR_DY-1)
Dim integer tof.isReady = 0, tof.newData = 0
Dim integer running = 1
Dim integer i, j, v, p
Dim integer offs, cx, cy, bdx, colf, colb

' Setup screen
CLS
cx = Int(MM.HRes /2)
cy = Int(MM.VRes /2)
bdx = 15
offs = -Int(TOF_FR_DX /2)
colb = RGB(0,0,0)


SetPin PIN_TOF_SDA, PIN_TOF_SCL, I2C
I2C Open 400, 100

'I2C Write &H40, 0, 2, Asc("A"), 50
'I2C Write &H40, 0, 2, Asc("R"), 0
Pause 1000

Dim integer buf(TOF_N_DATA-1)
For i=0 To 5
  For j=0 To 4
    I2C Read &H40, 0, 16, buf()
    Print j;
    Math V_print buf()
  Next
  Print
  Pause 1000
Next

'I2C Write &H40, 0, 2, Asc("S"), 0
Pause 1000
I2C Close
End

' Start TOF sensor ranging
TOF.Open
TOF.start

' Main loop
' ---------
Do While running
  't_loop = Timer


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
          colf = RGB(0, 255 -v, 0)
        EndIf
        Box (i +offs)*bdx +cx, (j +offs)*bdx +cy, bdx,bdx,,colb, colf
      Next
      'Print
    Next
    'Print
    tof.newData = 0
  EndIf
Loop

TOF.Close
End

' ===========================================================================
' TOF sensor subroutines
' ---------------------------------------------------------------------------
Sub TOF.Open
  ' Open serial connection to TOF unit (a Tiny2040 w/ a VL53L5CX)
  If Not(USE_TOFF) Then Exit Sub
  Local string s$ = "COM" +Str$(TOF_COM)
  Print "| Opening " +s$ +" to TOF unit ..."
  SetPin PIN_TOF_RX, PIN_TOF_TX, COM1
  Open s$ +":"+Str$(TOF_BAUD)+",,TOF._read,"+Str$(TOF_N_DATA) As #1
  tof.isReady = 1
End Sub


Sub TOF.start
  ' Start ranging ...
  If Not(USE_TOFF) Then Exit Sub
  Print #1, "A"+Str$(TOF_TILT_DEG,3) : Pause 500
  Print #1, "R000" : Pause 500
End Sub


Sub TOF.stop
  ' Sto[Bp ranging
  If Not(USE_TOFF) Then Exit Sub
  Print #1, "S000" : Pause 500
End Sub


Sub TOF.Close
  ' Stops ranging and closes serial port to TOF unit
  If Not(USE_TOFF) Then Exit Sub
  TOF.stop
  Close #1
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
  If Not(tof.isReady) Then Exit Sub
  If Loc(#1) >= TOF_N_DATA Then
    Line Input #1, buf$
    tof.len = Peek(Byte pbuf)
    If tof.len = TOF_N_DATA Then
      ' Complete tof distance data frame received
      tof.iFr = Val("&H"+Mid$(buf$,1,8))
      tof.dx  = Peek(Byte pbuf +9) -TOF_DATA_OFFS
      tof.dy  = Peek(Byte pbuf +10) -TOF_DATA_OFFS
      Memory Copy pbuf+11, pv, TOF_N_FR
      Memory Unpack v(), w(), TOF_N_FR, 8
      Math Scale w(), 1, tof.fr()
      'Print "iFr=";tof.iFr;
      'Print " Data len=";tof.len;" (";tof.dx;"x";tof.dy;")"
      'Math M_print tof.fr()
      tof.newData = 1
    EndIf
  EndIf
  't = Timer -t
  'Print t
End Sub

' ============================================================================                                                        