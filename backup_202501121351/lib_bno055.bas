' lib_bno055.bas v0.1.1
' ---------------------
' BNO055 MNU driver
' 2024-02-04 - v0.1.1 - Fewer constant declarations to save global
'                       variable slots
' Example:
'   BNO055.init
'   BNO055.getEuler v()
'   Print "h="+Str$(v(0),3,0)+" p="+Str$(v(1),3,0)+" r="+Str$(v(2),3,0)
'   BNO055.getSystemStatus
'   BNO055.close
' ---------------------------------------------------------------------------
Option Base 0
Option Explicit
Option Escape
Option Default Float

' Constants 
Const BNO_ID              = &HA0 ' Device ID
Const BNO_ADDR            = &H28 ' I2C address
'Const BNO_CHIP_ID_ADDR   = &H00 ' Register addresses
'Const BNO_PAGE_ID_ADDR   = &H07
'Const BNO_OPR_MODE_ADDR  = &H3D
'Const BNO_PWR_MODE_ADDR  = &H3E
'Const BNO_SYS_TRIG_ADDR  = &H3F
'Const BNO_SYS_STAT_ADDR  = &H39
'Const BNO_STEST_RES_ADDR = &H36
'Const BNO_SYS_ERR_ADDR   = &H3A
'Const BNO_OPR_MODE_CONF  = &H00 ' Operation mode(s)
'Const BNO_OPR_MODE_NDOF  = &H0C
'Const BNO_PWR_MODE_NORM  = &H00 ' Power mode(s)

' Global variables
Dim integer BNO055.ready = 0, BNO055.mode = &H0C
Dim integer BNO055.sys_status, BNO055.test_res, BNO055.sys_err

' ---------------------------------------------------------------------------
Sub BNO055.init sda, scl
  ' Initialize BNO055 sensor
  Local integer res
  Print "Initializing BNO055 orientation sensor ..."
  ' Open I2C2 for sensor
  SetPin sda,scl, I2C2
  I2C2 Open 400, 1000
  Pause 850
  res = BNO055._read1(&H00)
  Print "| Device "+Choice(res = BNO_ID, "", "not ") +"found."
  BNO055.ready = res = BNO_ID
  If BNO055.ready Then
    ' Reset and configure
    BNO055._write1 &H3D, &H00
    Print "| Resetting ";
    BNO055._write1 &H3F, &H20 : Pause 30
    Do While Not(BNO055._read1(&H00) = BNO_ID)
      Print ".";
      Pause 50
    Loop
    Pause 50
    Print
    BNO055._write1 &H3E, &H00 : Pause 10
    BNO055._write1 &H07, &H00
    BNO055._write1 &H3F, &H00 : Pause 10
    BNO055._write1 &H3D, BNO055.mode : Pause 20
    Print "| Ready."
  EndIf
End Sub

Sub BNO055.getEuler v()
  ' Get Euler vector -> `v(2)` heading, pitch, roll
  Static integer buf(5) = (0,0,0,0,0,0), w(2)
  Local string s$ length 2
  I2C2 Write BNO_ADDR, 0, 1, &H1A
  I2C2 Read BNO_ADDR, 0, 6, buf()
  w(0) = buf(0) Or (buf(1) << 8)
  s$ = Chr$(buf(2))+Chr$(buf(3))
  w(1) = Str2bin(int16, s$)
  s$ = Chr$(buf(4))+Chr$(buf(5))
  w(2) = Str2bin(int16, s$)
  Math Scale w(), 1/16, v()
End Sub

Sub BNO055.getSystemStatus
  ' Get system status
  BNO055._write1 &H07, &H00
  BNO055.sys_status = BNO055._read1(&H39)
  BNO055.test_res = BNO055._read1(&H36)
  BNO055.sys_err = BNO055._read1(&H3A)
  Print BNO055.sys_status, BNO055.test_res, BNO055.sys_err
End Sub

Sub BNO055.close
  ' Close connection to BNO055 sensor
  I2C2 Close
  BNO055.ready = 0
End Sub

Function BNO055._read1(reg%) As integer
  I2C2 Write BNO_ADDR, 0, 1, reg%
  I2C2 Read  BNO_ADDR, 0, 1, BNO055._read1
End Function

Sub BNO055._write1 reg%, val%
  I2C2 Write BNO_ADDR, 0, 2, reg%, val%
End Sub

Function _b2s$(a%(), n) As String
  Local integer i
  Local string s$ = ""
  For i=0 To n-1 : _b2s$ = _b2s$ +Hex$(a%(i),2) :  Next
End Function

' ---------------------------------------------------------------------------