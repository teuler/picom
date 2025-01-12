' hexapod-calibrate v0.1.1
' ------------------------
'
' Copyright (c) 2023-24 Thomas Euler
' MIT Licence
'
' Options:
'   OPTION COLORCODE ON
'   OPTION DISPLAY 64,80
'
' Hardware (Server):
' - COM2 (tx=GP4, rx=GP5) as device #5 -> Maestro18 servo controller
'
' ---------------------------------------------------------------------------
Option Base 0
Option Explicit
Option Escape
Option Default Float
Option Heartbeat Off

' ---------------------------------------------------------------------------
' Initialization of global definitions
GoTo Start
Main:
  ' Initialize
  R.Init

  R.Shutdown
  End

' ===========================================================================
' Start of robot's definitions
' ---------------------------------------------------------------------------
Start:
  ' Set data read pointer here
  StartR: Restore StartR

  Const R.VERSION$       = "0.1.1"
  Const R.NAME$          = "hexapod|calibration-helper"

  ' - - - - - - - - - - - - - - - - - - - - - - - -  - - - - - - - - - - - -
  ' GLOBAL DEFINITIONS
  ' - - - - - - - - - - - - - - - - - - - - - - - -  - - - - - - - - - - - -
  Const DEB_SRV          = &H01
  Const DEB_VERB         = &H02
  Const DEB_GAIT         = &H04
  Const DEB_INP          = &H08
  Const DEB_IK           = &H10
 'Const DEB_SIM          = &H20
  Const DEB_COM          = &H40
 'Const DEB_SLNT         = &H80
  Dim integer DEBUG      = 0
 'Const CHK_SRV_PARAMS   = 0
 'Const NOT_AN_ANGLE     = -1

  ' Gait generator (GGN) states
  Const GGN_IDLE         = 0
  Const GGN_COMPUTE      = 1
  Const GGN_STAND_BY     = 2
  Const GGN_MOVING       = 3
  Const GGN_DO_SERVOS    = 4
  ' ...
  Const GGN_STATE$       = "Idle,Compute,Standby,Moving,DoServos"

  ' Error codes
  Const ERR_OK           = 0
  Const ERR_MOVE_OVERDUE = 1
  ' ...
  Const ERR_STR$         = "Ok,MoveOverdue"

  ' Leg-related definitions
  ' -----------------------
  Const LEG_N            = 6

  Const COX              = 0
  Const FEM              = 1
  Const TIB              = 2

  Const COX_LEN          = 48
  Const FEM_LEN          = 47
  Const TIB_LEN          = 84

  Const FEM2TIB2_DIF     = FEM_LEN^2 -TIB_LEN^2
  Const FEM_2LEN         = FEM_LEN *2
  Const FEMTIB_2LEN      = FEM_LEN *TIB_LEN *2

  ' Body-related definitions
  ' ------------------------
  Const TIB_CORR_ANG     = 50
  Const BODY_R           = 60      ' Body radius [mm]
  Const BODY_COX_ANG     = 60      ' Fixed leg offset angle around body

  Dim integer COX_OFF_ANG(LEG_N-1) ' Leg angles (in degree)
  Data  0, 60,120,180,240,300
  Read COX_OFF_ANG()

  Dim COX_OFF_XYZ(LEG_N-1, 2)      ' Leg positions relative to body
  R._calcLegPos BODY_COX_ANG, BODY_R, COX_OFF_XYZ()

  ' Movement-related definitions
  ' ----------------------------
  ' Limits within movements are considered finished
  Const TRAVEL_DEAD_Z    = 2
 'Const TURN_DEAD_Z      = 5
  Const LEGS_DOWN_DEAD_Z = 10      ' TODO

  ' Limits of movement control parameters
  ' (Rotations are in [degree], travel in [mm],
  '  and  delays in [ms])
  Const BODY_Y_OFFS_MIN  = 0
  Const BODY_Y_OFFS_MAX  = 70
  Const BODY_Y_SHFT_LIM  = 64

  Dim integer BODY_X_Z_POS_LIM(2)
  Data 15, BODY_Y_OFFS_MAX +BODY_Y_SHFT_LIM, 15
  Read BODY_X_Z_POS_LIM()

  Dim integer BODY_XYZ_ROT_LIM(2) = (8, 20, 8)
  Dim integer TRAVEL_X_Z_LIM(2) = (34, 0, 34) ' was 40,0,40

  Const TRAV_ROT_Y_LIM   = 25
  Const LEG_LIFT_MIN     = 20      ' 40
  Const LEG_LIFT_MAX     = 60      ' 80
  Const DELAY_INPUT_MIN  = 0
  Const DELAY_INPUT_MAX  = 255
  Const DELAY_SPEED_MAX  = 2000

  ' Start position of feet
  ' (Measured from beginning of coxa; leg coordinate system (?))
  Const FEM_STRT_ANG     = -20
  Const TIB_STRT_ANG     = 10
  Const COX_OFFS_FACT    = 1.0
  Dim FEET_INIT_XYZ(LEG_N-1, 2)
  R._calcFootPos FEM_STRT_ANG, TIB_STRT_ANG, BODY_COX_ANG, FEET_INIT_XYZ()

  ' Client-server communication
  ' ---------------------------
  ' Both messages start with:
  '   Byte(s)  0...7 : time since start in [ms] (uint32, hexlified)
  '                8 : command code
  '                9 : n data bytes (following)
  ' Client->server continues with:
  '               10 : bodyYOffs
  '               11 : legLiftH
  '            12,13 : x_zTravL; current travel length X,Z
  '               14 : travRotY; current travel rotation Y
  '               15 : delaySpeed; adjustible delay [ms/10]
  '               16 : buzzer signal (0=none, 1=beep, ...)
  '               17 : command extension (e.g., power on/off)
  '            18,19 : x_zBody
  '            20-22 : xyzBodyRot
  '               23 : unused
  ' Server->Client continues with:
  '               10 : servo battery voltage [V*10]
  '               11 : logic battery voltage [V*10]
  '            12,13 : compass heading [degree] =b(12)*10 +b(13)
  '            14,15 : compass pitch and roll [-90..90, degree]
  '               16 : GGN state
  '          17...24 : servo load for servos #0..7, 0..240 [a.u.]
  '                    WARNING - 1 more than currently reserved
  ' - All single-byte values +COM_DATA_OFFS to avoid that escape characters
  '   (e.g., `\r`) appear in the data section of a package.
  ' - For parameter limits, see `Movement-related definitions`
  '
  Const COM_DATA_OFFS    = 14
  Const COM_N_BYTEVAL    = 16   ' number of single-byte values
  Const COM_MSG_LEN      = 24   ' total length in bytes

  Const CMD_BODY         = 1    ' Commands/message types
  Const CMD_MOVE         = 2
  Const CMD_STOP         = 3
  Const CMD_STATE        = 4
  Const CMD_POWER        = 5
  Const CMD_SHOW         = 6
  Const CMD_STR$         = "Body,Move,Stop,State,Power,ShowOff"

  Const RES_NEW_MSG      = &H01 ' Message handling
  Const RES_HANDLED      = &H02
  Const RES_ERROR        = &H04

  Const HS_TOUT_MS       = 3000 ' Handshake time-out

  Dim integer com.isReady = 0, com.in_res = 0, com.in_t_ms
  Dim integer com.in(COM_N_BYTEVAL -1)

  ' - - - - - - - - - - - - - - - - - - - - - - - -  - - - - - - - - - - - -
  ' SERVER-SPECIFIC DEFINITIONS
  ' - - - - - - - - - - - - - - - - - - - - - - - -  - - - - - - - - - - - -
  ' Servo controller-related definitions
  Const SRV_PORT$        = "COM2:57600"
  Const SRV_MIN_US       = 600
  Const SRV_MAX_US       = 2400
  Const SRV_N            = 18
 'Const SRV_RES          = 4       ' 0.25 us resolution
 'Const SRV_SPEED        = 0       ' 0=fastest, 1=slowest ... fast
 'Const SRV_ACCEL        = 0       ' 0=fastest, 1=low ... high
  Const SRV_N_STEPS      = 10      ' 80
  Const SRV_N_LOAD_CH    = 4       ' max. load (MCP3208) channel

  ' Pico pins
  Const PIN_SCL          = MM.Info(PinNo GP19)
  Const PIN_SDA          = MM.Info(PinNo GP18)
  Const PIN_LED          = MM.Info(PinNo GP2)
  Const PIN_BUZZER       = MM.Info(PinNo GP6)
  Const PIN_SRV_TX       = MM.Info(PinNo GP4)
  Const PIN_SRV_RX       = MM.Info(PinNo GP5)
  Const PIN_AIN_SRV_BAT  = MM.Info(PinNo GP28) ' ADC2
  Const PIN_AIN_LGC_BAT  = MM.Info(PinNo GP27) ' ADC1
  Const PIN_SPI2_RX      = MM.Info(PinNo GP12)
  Const PIN_SPI2_TX      = MM.Info(PinNo GP11)
  Const PIN_SPI2_CLK     = MM.Info(PinNo GP10)
  Const PIN_MCP3208_CS   = MM.Info(PinNo GP13)
  Const PIN_WS2812       = MM.Info(PinNo GP3)
  Const PIN_HS_IN        = MM.Info(PinNo GP7)
  Const PIN_HS_OUT       = MM.Info(PinNo GP8)

  ' RGB LEDs
  Const N_WS2812         = 2
  Const RGB_I_PULSE      = 0
  Const RGB_N_STEP       = 80

  ' Status
  ' (0=shutdown, 1=idle, 2=running)
  Dim integer R.running = 1, R.connected = 0
  Dim integer R.showingOff = 0

  ' Jump to main
  GoTo Main

' ===========================================================================
' Initialization and shutdown
' ---------------------------------------------------------------------------
Sub R.Init
  ' Prepare handware
  Local integer j
  Local float af, at, ac
  InitR: Restore InitR
  Print "\r\n"+R.NAME$+" v"+R.VERSION$
  Print "| running on "+MM.Device$+" MMBasic v"+Str$(MM.Ver)
  Print "Initializing onboard hardware ..."
  Print "| CPU @ "+Str$(Val(MM.Info(CPUSPEED))/1E6)+" MHz"

  ' Open COM port to MiniMaestro 18 servo board
  Print "Connecting to Maestro servo controller ..."
  Print "| RX="+Str$(PIN_SRV_RX)+" TX="+Str$(PIN_SRV_TX);
  Print " options=`";SRV_PORT;"`"
  SetPin PIN_SRV_RX, PIN_SRV_TX, COM2
  Open SRV_PORT$ As #5
  Pause 200
  j = R.getServoErr()

  ' Leg servos
  ' ----------
  ' e.g., leg.srv(COX,LEG_RM) -> servo #4
  Dim integer leg.srv(2, LEG_N-1)
  Data  4,10,16  ' Leg 0 (RM) right-middle
  Data  5,11,17  ' Leg 1 (RF) right-front
  Data  0, 6,12  ' Leg 2 (LF) left-front
  Data  1, 7,13  ' Leg 3 (LM) left-middle
  Data  2, 8,14  ' Leg 4 (LR) left-rear
  Data  3, 9,15  ' Leg 5 (RR) right-rear
  Read leg.srv()

  Dim integer srv.dir(SRV_N-1)
  Data -1,-1,-1,  1, 1, 1
  Data  1, 1, 1,  1, 1, 1
  Data  1, 1, 1,  1, 1, 1
  Read srv.dir()

  ' Servo angle direction (by leg and limb)
  Dim srv.ang_f(LEG_N-1, 2)
  Data  1, 1, 1, 1, 1, 1
  Data  1, 1, 1, 1, 1, 1
  Data -1,-1,-1,-1,-1,-1
  Read srv.ang_f()

  ' Servo ranges (by servo ID, 0..17, as angle [deg])
  Dim integer srv.r_deg(1, SRV_N-1)
  Dim integer srv.da_deg(SRV_N-1)
  Data -45, +45,  -45, +45,  -45, +45
  Data -45, +45,  -45, +45,  -45, +45
  Data -55, +55,  -55, +55,  -55, +55
  Data -55, +55,  -55, +55,  -55, +55
  Data -90, +35,  -90, +35,  -90, +35
  Data -90, +35,  -90, +35,  -90, +35
  Read srv.r_deg()
  For j=0 To SRV_N-1
    srv.da_deg(j) = srv.r_deg(1,j) -srv.r_deg(0,j)
  Next j

  ' Servo ranges (by servo ID, 0..17, as timing in us)
  ' e.g., srv.ranges(0,5) -> minimum for servo #5
  Print "Reading servo calibration data ..."
  Dim float srv.r_us(1, SRV_N-1)
  Dim integer srv.dt_us(SRV_N-1)
  ' Coxa (-45, +45)
  Data  965, 1812,  1001, 1857
  Data  900, 1803,  1167, 2000
  Data 1030, 1970,  1074, 1931
  ' Femur (-55, +55)
  Data 1640-500, 1640+500,  1600-500, 1600+500
  Data 1570-500, 1570+500,  1360-500, 1360+500
  Data 1460-500, 1460+500,  1493-500, 1493+500
  ' Tibia (-90, +35)
  Data  782, (1847  -782) /90 *(35 -2) +1847
  Data  634, (1764  -634) /90 *(35 -7) +1764
  Data  846, (1850  -846) /90 *(35 +3) +1850
  Data  943, (2020  -943) /90 *(35   ) +2020
  Data 1027, (1995 -1027) /90 *(35 +4) +1995
  Data  840, (1900  -840) /90 *(35 -2) +1900
  Read srv.r_us()
  For j=0 To SRV_N-1
    srv.dt_us(j) = srv.r_us(1,j) -srv.r_us(0,j)
  Next

  ' Servo load-related
  Print "Reading servo load calibration data ..."
  Dim srv.loadRaw(SRV_N_LOAD_CH-1), srv.load(SRV_N_LOAD_CH-1)
  Dim srv.loadMin(SRV_N_LOAD_CH-1), srv.loadMax(SRV_N_LOAD_CH-1)
  Data  30,   35, 235, 297
  Read srv.loadMin()
  Data 696, 1131, 799, 850
  Read srv.loadMax()

  ' Variables for smooth servo movements (all in [ms]):
  Dim float srv.pCurr(SRV_N-1)
  Dim float srv.pSteps(SRV_N_STEPS-1, SRV_N-1)
  Dim integer srv.iStep = 0, srv.nSrv, srv.isMoveDone
  Dim float srv.nStep(SRV_N-1)
  Math Set SRV_N_STEPS, srv.nStep()

  ' List calibration values
  Print "Calibrated servo positions and ranges ..."
  Print "Servo min           max        dt[us] ds["+Chr$(186)+"]"
  Print "----- ----------    ---------- ------ -----"
  For j=0 To SRV_N-1
    Print " #"+Str$(j,2)+"  ";
    Print Str$(srv.r_us(0,j),4,0) +" ("+Str$(srv.r_deg(0,j),3,0)+")";
    Print " .. ";
    Print Str$(srv.r_us(1,j),4,0) +" ("+Str$(srv.r_deg(1,j),3,0)+")  ";
    Print Str$(srv.dt_us(j),4,0)+"  "+Str$(srv.da_deg(j),3,0)
  Next j
  Print "Ready."
End Sub

' - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
Sub R.Shutdown
  ' Shutdown hardware
  R.servosOff
  Print "| Done."
End Sub

' - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
Sub R._calcLegPos ac, br, xyz()
  ' Calculate leg positions relative to body
  Local integer i
  Math Set 0, xyz()
  For i=0 To LEG_N-1
    xyz(i,0) = +Cos(i*Rad(ac)) *br ' new
    xyz(i,2) = -Sin(i*Rad(ac)) *br
  Next
End Sub


Sub R._calcFootPos af, at, ac, xyz()
  ' Calculate starting positions for feet (from angles)
  Local float rf = Rad(af), rt = Rad(at), rc = Rad(ac), rx
  Local integer i
  Math Set 0, xyz()
  For i=0 To LEG_N-1
    rx = FEM_LEN *Cos(rf) -TIB_LEN *Sin(rt +rf) +COX_LEN*COX_OFFS_FACT
    xyz(i,0) = rx *Cos(rc *i)
    xyz(i,1) = FEM_LEN *Sin(rf) +TIB_LEN *Cos(rt -rf)
    xyz(i,2) = -rx *Sin(rc *i)
  Next
End Sub

' ===========================================================================
' Maestro controller-related routines
' ---------------------------------------------------------------------------
' All-servo routines
' - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
Sub R.Angles2Pos a_deg(), p()
  ' Converts all servo positions from angle (in degree) to position (in us)
  ' Note that is `a_deg(iLeg, COX|FEM|TIB)`
  Local integer i, j, k,  a
  For i=0 To LEG_N-1
    For j=0 To 2
      k = leg.srv(j,i)
      a = Min(srv.r_deg(1,k), Max(srv.r_deg(0,k), a_deg(i,j)))
      p(k) = srv.r_us(0,k) +srv.dt_us(k) *(a -srv.r_deg(0,k)) /srv.da_deg(k)
    Next j
  Next i
End Sub


Sub R.servosOff
  ' Switch all servos off
  Print #5, "\&9F"+Chr$(SRV_N)+Chr$(0)+String$(SRV_N*2, 0)
  Print "| Servos off (&H"+Hex$(R.getServoErr(), 4)+")"
End Sub

' ---------------------------------------------------------------------------
' Single-servo methods
' - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
Sub R.setServo_deg i%, a_deg%, wait%
  ' Set servo `i%` position as angle `a_deg%` in degree
  If i% >= 0 And i% < SRV_N Then
    Local integer _t = R._a2t(i%, a_deg%)
    R._SetServo i%, _t, wait%
  EndIf
End Sub

Sub R.setServo i%, t_us%, wait%
  ' Set servo `i%` position as timing `t_us%` (in us)
  If i% >= 0 And i% < SRV_N Then
    Local integer _t = Min(Max(t_us%, SRV_MIN_US), SRV_MAX_US)
    R._SetServo i%, _t, wait%
  EndIf
End Sub

Sub R._setServo i%, t_us%, wait%
  ' Set servo `i%` position as timing `t_us%` (in us)
  ' If `wait` <> 0, it is waited until the out buffer is empty, NOT until
  ' the servo has reached its position
  ' (NO PARAMETER CHECKING)
  Local integer hb, lb
  Local integer _t = t_us% *4
  hb = (_t And >> 7) And &H7F
  lb = _t And &H7F

  ' Send `set target` command
  If DEBUG And DEB_SRV Then
    Print "Servo #";i%;" to ";Str$(_t /4, 0);" us"
  EndIf
  Print #5, "\&84"+Chr$(i%)+Chr$(lb)+Chr$(hb)
  If wait% Then Do While Lof(#5) < 256 : Pause 1 : Loop
End Sub

Function R._a2t(i%, a_deg%) As integer
  ' Convert for servo `%i` the angle `a_deg%` into the position (in us)
  ' Considers the servo's range in deg and its calibration values
  Local integer t, a = Min(srv.r_deg(1,i%), Max(srv.r_deg(0,i%), a_deg%))
  t = srv.r_us(0,i%) +srv.dt_us(i%) *(a -srv.r_deg(0,i%)) /srv.da_deg(i%)
  R._a2t = t
  If Not(DEBUG And DEB_SRV) Then Exit Function
  Print "Servo #"+Str$(i%)+" "+Str$(a,3,0) +Chr$(186)+" -> ";
  Print Str$(t,4,0)+" us"
End Function

' - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
Function R.getServoErr() As integer
  ' Returns (and clears) last servo controller error
  Static string res$
  Static integer pRes = Peek(VARADDR res$)
  Local integer i

  ' Send `get errors` command and wait for reply
  If DEBUG And DEB_SRV Then Print "Get errors: ";
  Print #5, "\&A1"
  i = 20
  Do While Loc(#5) = 0 And i > 0
    Pause 2
    Inc i, -1
  Loop
  res$ = Input$(2, #5)

  ' Process reply
  If Peek(BYTE pRes) > 0 Then
    R.getServoErr = (Peek(BYTE pRes+1) +(Peek(BYTE pRes+2) << 8)) And &H1F
    If DEBUG And DEB_SRV Then Print "0x"+Hex$(R.getServoErr, 2)
  Else
    R.getServoErr = -1
    If DEBUG And DEB_SRV Then Print "No reply"
  EndIf
End Function


' - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
Sub _log.servos
  ' Print current angle (in degree) and position (in us) for each servo
  Local integer i
  For i=0 To SRV_N-1
    Print "Servo #"+Str$(i)+" "+Str$(srv.ang(i),3,0) +Chr$(186)+" -> ";
    Print Str$(srv.pos(i),4,0)+" us"
  Next
End Sub

' ---------------------------------------------------------------------------
