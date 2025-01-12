' hexapod-server v0.2.00
' ----------------------
' Makes the robot move and walk, monitors servo load, servo and logic battery,
' and reads the compass heading, In addition, it controls two RGB LEDs for
' feedback and a buzzer. Is controlled via a serial connection by the client
' (`hps_cli.bas`).
'
' Copyright (c) 2023-25 Thomas Euler
' MIT Licence
'
' v0.1.13 (2024-01-13)
' - bug with movement control fixed (jitter at start and end)
' - commnication with client (basic movement commands)
' - code cleaned up
' v0.1.14
' - fixed bug in server -> client communication
' v0.1.20 (2024-02-17)
' - body shift and rotation implemented
' v0.1.23 (2024-03-23)
' - CMD_SHOW implemented, to assume defined postures
' - custom (predefined) beeps
' - gamepad control improved
' v0.1.24 (2024-04-15)
' - Servo calibration mode started (press `C`)
' - Some cleaning up old code
' v0.1.25 (2024-11-23)
' - Testing servo load functions
' v0.2.00 (2025-01-01)
' - Fix a bug initializing MCP3208 library
' - Added the option to use I2C for server-client communication, to free one
'   hardware UART (e.g., for the TOF sensor)
'
' Options:
'   OPTION COLORCODE ON
'   OPTION DISPLAY 64,80
'   OPTION HEARTBEAT OFF
'   OPTION CPUSPEED 252000
'
' Robot's coordinate system:
'  x-axis points left
'  y-axis points down
'  z-axis points backwards
'
' Hardware (Server):
' - COM2 (tx=GP4, rx=GP5) as device #5 -> Maestro18 servo controller
' - Client-server:
'   (a) COM1 (tx=GP0, rx=GP1) as device #2 -> To connect to client
'   (b) I2C (sda=GP0, scl=GP1) as device &H42, with server being the master
' - Buzzer @ GP6
' - Yellow onboard LED @ GP2
' - Servo battery voltage @ GP28 (ADC2)
' - BNO055 sensor via I2C2 @ GP18, GP19
' - MCP3208 8-channel ADC via SPI2 @ GP12, GP11, GP10, GP13
' - WS2812 RGB LED(s) @ GP3
' - handshake GP7<-GP2 (client), GP8->GP3 (client)
'
' Required libraries:
' - `lib.bno055.bas`
' - `lib_mcp3208.bas`
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
  ' Debug options:
  ' (DEB_SRV, DEB_VERB, DEB_GAIT, DEB_INP, DEB_IK, DEB_COM)
  DEBUG = DEB_VERB Or DEB_COM

  ' Configuration
  Dim integer USE_I2C_COM = 1

  ' Control variables
  Dim string key$
  Dim i
  Dim t, t_loop, t_state, t_load = Timer

  ' Initialize
  R.Init
  R.resetGait
  R.resetInput
  R.resetGaitGen
  Pause 500
  R.Buzz 440, 100

  ' Set LED pulse color
  R.RGB_LED RGB_I_PULSE, ,,, 1

  ' Main loop
  ' ---------
  Do While R.running > 0
    t_loop = Timer

    ' Keep walk machine running and pulsing ...
    If R.running = 2 Then R.spinGaitGen
    R.updateHandshake
    R.RGB_LED_pulse

    ' Update sensors ...
    BNO055.getEuler IMU()

    /*
    If t_load +1000 < t_loop Then
      t = Timer
      MCP3208.readADCChans SRV_N_LOAD_CH, srv.loadRaw()
      Math C_Sub srv.loadRaw(), srv.loadMin(), srv.load()
      Math C_Div srv.load(), srv.loadMax(), srv.load()
      Math Scale srv.load(), 100, srv.load()
      t = Timer -t
      'Print "dt="+Str$(t, 5, 3)+" ";
      'For i=0 To SRV_N_LOAD_CH-1
      '  'Print Format$(srv.load(i), "% 4.0f")+" ";
      '  Print srv.loadRaw(i)," ";
      'Next
      'Print
      t_load = Timer
    EndIf
    */

    ' If connected to client, handle communication
    If R.connected Then
      ' If serial port is used for communication, send status actively and
      ' frequently. If I2c is used, wait for client to request status
      If Not(USE_I2C_COM) And (t_state +250 < t_loop) Then
        COM.sendState
        t_state = Timer
      EndIf

      ' Check for client command and handle it ...
      ' (If I2C is used for communication, `handleMsg` will also deal with
      '  status requests)
      COM.handleMsg
      If DEBUG And DEB_COM Then
        If com.in_res > 0 Then
          Print "Cli-> cmd="+Str$(com.in(0))+" t="+Str$(com.in_t_ms);
          Print " res=&H"+Hex$(com.in_res, 2)
        EndIf
      EndIf
    EndIf

    ' If a posture is being assumed, update this
    R.updatePosture

    ' Check for user key
    key$ = Inkey$
    If Len(key$) > 0 Then
      If Asc(LCase$(key$)) = 27 Then R.running = 0 : Exit : EndIf
      If LCase$(key$) = "c" Then R.doServoCalib
    EndIf

    R._timing t_loop
  Loop

  R._timing 0,1
  R.Power 0
  R.Shutdown
  End

' ===========================================================================
' Start of robot's definitions
' ---------------------------------------------------------------------------
Start:
  ' Set data read pointer here
  StartR: Restore StartR

  Const R.VERSION$       = "0.2.00"
  Const R.NAME$          = "hexapod|server"

  ' - - - - - - - - - - - - - - - - - - - - - - - -  - - - - - - - - - - - -
  ' GLOBAL DEFINITIONS
  ' - - - - - - - - - - - - - - - - - - - - - - - -  - - - - - - - - - - - -
  Const DEB_SRV          = &H01
  Const DEB_VERB         = &H02
  Const DEB_GAIT         = &H04
  Const DEB_INP          = &H08
  Const DEB_IK           = &H10
  Const DEB_COM          = &H40
 'Const DEB_SLNT         = &H80
  Dim integer DEBUG      = 0

  ' Gait generator (GGN) states
  Const GGN_IDLE         = 0
  Const GGN_COMPUTE      = 1
  Const GGN_STAND_BY     = 2
  Const GGN_MOVING       = 3
  Const GGN_DO_SERVOS    = 4
  Const GGN_STATE$       = "Idle,Compute,Standby,Moving,DoServos"

  ' Error codes
  Const ERR_OK           = 0
  Const ERR_MOVE_OVERDUE = 1
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
  ' Limits within movements are considered3 finished
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
  Const COM_I2C_ADDR     = &H42 ' Server address if I2C is used
  Const COM_I2C_SPEED    = 400
  Const COM_I2C_TOUT     = 200  ' ms

  Const CMD_BODY         = 1    ' Commands/message types
  Const CMD_MOVE         = 2
  Const CMD_STOP         = 3
  Const CMD_STATE        = 4
  Const CMD_POWER        = 5
  Const CMD_SHOW         = 6
  Const CMD_STR$         = "Body,Move,Stop,State,Power,Show"

  Const RES_NEW_MSG      = &H01 ' Message handling
  Const RES_HANDLED      = &H02
  Const RES_ERROR        = &H04

  Const HS_TOUT_MS       = 3000 ' Handshake time-out

  Dim integer com.isReady = 0, com.in_res = 0, com.in_t_ms
  Dim integer com.in(COM_N_BYTEVAL -1)

  Dim integer com.i2cRecv = 0   ' I2C client only
  Dim string com.i2cBuf$
  'Dim integer com.i2cBuf(COM_MSG_LEN -1)

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
  Const PIN_SCL          = MM.Info(PinNo GP19) ' I2C2
  Const PIN_SDA          = MM.Info(PinNo GP18) ' |
  Const PIN_LED          = MM.Info(PinNo GP2)
  Const PIN_BUZZER       = MM.Info(PinNo GP6)
  Const PIN_SRV_TX       = MM.Info(PinNo GP4)  ' COM2/UART1 (Servos)
  Const PIN_SRV_RX       = MM.Info(PinNo GP5)  ' |
  Const PIN_AIN_SRV_BAT  = MM.Info(PinNo GP28) ' ADC2
  Const PIN_AIN_LGC_BAT  = MM.Info(PinNo GP27) ' ADC1
  Const PIN_SPI2_RX      = MM.Info(PinNo GP12) ' MCP3208
  Const PIN_SPI2_TX      = MM.Info(PinNo GP11) ' |
  Const PIN_SPI2_CLK     = MM.Info(PinNo GP10) ' |
  Const PIN_MCP3208_CS   = MM.Info(PinNo GP13) ' |
  Const PIN_WS2812       = MM.Info(PinNo GP3)
  Const PIN_HS_IN        = MM.Info(PinNo GP7)  ' Handshake
  Const PIN_HS_OUT       = MM.Info(PinNo GP8)  ' |

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
  Print "| running on "+MM.Device$+" MMBasic v"+Str$(MM.Info(VERSION))
  Print "Initializing onboard hardware ..."
  Print "| CPU @ "+Str$(Val(MM.Info(CPUSPEED))/1E6)+" MHz"

  ' LED
  Print "| yellow LED"
  SetPin PIN_LED, DOUT
  Pin(PIN_LED) = 0

  ' Buzzer
  Print "| Buzzer"
  SetPin PIN_BUZZER, PWM3A

  ' RGB LED
  Print "| RGB LED(s)"
  Dim integer rgb.rgb(N_WS2812-1)
  Dim rgb.pstep(2), rgb.pnow(2), rgb.pcol(2)
  Dim integer rgb.pi = 0, rgb.conn = 0

  ' Setup hardware handshake
  Dim hs.t_last_ms = 0
  SetPin PIN_HS_IN, INTH, R._cbHandshake, PULLDOWN
  SetPin PIN_HS_OUT, DOUT : Pin(PIN_HS_OUT) = 0

  ' Init INU sensor
  Dim IMU(2)
  BNO055.init PIN_SDA, PIN_SCL

  ' Init MCP3208 for load sensing
  MCP3208.init PIN_SPI2_RX, PIN_SPI2_TX, PIN_SPI2_CLK, PIN_MCP3208_CS

  ' ADCs for battery voltages
  Print "Battery status:"
  SetPin PIN_AIN_SRV_BAT, AIN
  Print "| Servo battery V="+Str$(R.servoBattery_V(), 1, 2)+"V"
  SetPin PIN_AIN_LGC_BAT, AIN
  Print "| Logic battery V="+Str$(R.logicBattery_V(), 1, 2)+"V"

  ' Open COM port to MiniMaestro 18 servo board
  Print "Connecting to Maestro servo controller ..."
  Print "| RX="+Str$(PIN_SRV_RX)+" TX="+Str$(PIN_SRV_TX);
  Print " options=`";SRV_PORT;"`"
  SetPin PIN_SRV_RX, PIN_SRV_TX, COM2
  Open SRV_PORT$ As #5
  Pause 200
  j = R.getServoErr()

  ' Connect to client
  COM.open

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
  Data  965, 1812         ' 0
  Data 1001, 1857         ' 1
  Data  900, 1803         ' 2
  Data 1167, 2000         ' 3
  Data 1030, 1970         ' 4
  Data 1074, 1931         ' 5
  ' Femur (-55, +55)
  Data 1640-500, 1640+500 ' 6
  Data 1600-500, 1600+500 ' 7
  Data 1570-500, 1570+500 ' 8
  Data 1710-500, 1710+500 ' 9 new servo, 2024-04-18
  Data 1460-500, 1460+500 '10
  Data 1493-500, 1493+500 '11
  ' Tibia (-90, +35)
  Data  782, (1847  -782) /90 *(35 -2) +1847 '12
  Data  634, (1764  -634) /90 *(35 -7) +1764 '13
  Data  846, (1850  -846) /90 *(35 +3) +1850 '14
  Data  943, (2020  -943) /90 *(35   ) +2020 '15
  Data 1027, (1995 -1027) /90 *(35 +4) +1995 '16
  Data  840, (1900  -840) /90 *(35 -2) +1900 '17
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

  ' Set servo speed and acceleration at the Maestro level
 'Print "| Setting servo speed to "+Str$(SRV_SPEED)+" and ";
 'Print "acceleration limit to "+Str$(SRV_ACCEL)+" (0=maximum)"
 'For j=0 To SRV_N-1
 '  R.setServoSpeed j, SRV_SPEED
 '  R.setServoAccel j, SRV_ACCEL
 'Next j

  ' Posture data
  ' `state(postureID, step, i0, nSteps, last_t_ms)`
  Dim pst.state(3) = (-1, 0,0, 0)
  Dim integer pst.n(1,1)
  Data 0,4, 4,8
  Read pst.n()
  Dim pst.data(11,11)
  ' #1
  '    dt   bo_y, tl_x, tl_z, tr_y, lh, bp_x, bp_z, br_x, br_y, br_z, ds
  Data  500,  10,    0,  -20,    0, 48,    0,   10,   -6,    0,    0,  50
  Data 2000,  15,    0,    0,    0, 48,    0,   15,   -9,    0,    0, 160
  Data  750,   5,    0,    0,    0, 48,    0,    5,   -3,    0,    0, 160
  Data   50,   0,    0,    0,    0, 48,    0,    0,    0,    0,    0, 160
  ' #2
  Data 1000,   0,    0,   25,    0, 48,    0,    0,   -6,    0,    0,  50
  Data 2500,   0,    0,    0,    0, 48,    0,  -15,   -9,    0,    0, 100
  Data 1500,   0,    0,    0,    0, 48,    0,  -10,   -6,    0,    0, 160
  Data  600,   0,    0,    0,    0, 48,    0,   -8,   -5,    0,    0, 160
  Data  400,   0,    0,    0,    0, 48,    0,   -6,   -4,    0,    0, 160
  Data  300,   0,    0,    0,    0, 48,    0,   -4,   -3,    0,    0, 160
  Data  200,   0,    0,    0,    0, 48,    0,   -3,   -1,    0,    0, 160
  Data   50,   0,    0,    0,    0, 48,    0,    0,    0,    0,    0, 160
  Read pst.data()

  If DEBUG And DEB_VERB Then R.logServoCalibVals
  Print "Ready."
End Sub

' - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
Sub R.Shutdown
  ' Shutdown hardware
  Local integer res
  Print "Shutting down ..."
  SetTick 0,0,1
  Pin(PIN_LED) = 0
  SetPin PIN_LED, Off
  PWM 1, Off
  BNO055.close
  MCP3208.close
  R.RGB_LED 0, 0,0,0
  R.RGB_LED 1, 0,0,0
  R.servosOff
  COM.close
  Print "| Done."
End Sub


Sub R.Power _on
  ' Powers the robot up if `_on` is true, otherwise goes into resting
  Local integer n = 0
  If _on Then
    If R.running = 1 Then
      ' Startup sequence ...
      Print "Powering up ..."
      R.resetGait
      R.resetInput
      R.resetGaitGen
      Pause 500
      R.inputBody 0, 48
      R.inputWalk 0,0,0, 150
      R.switchGaitGen 1
      R.running = 2
      R.RGB_LED RGB_I_PULSE, ,,, 1
      Print "| Done"
      Exit Sub
    EndIf
  Else
    If R.running = 2 Then
      ' Powering down sequence ...
      R.inputBody 0, 48
      R.inputPosition 0,0, 0,0,0
      R.inputWalk 0,0,0, 150
      Print "Powering down ";
      Do
        R.spinGaitGen
        Print ".";
        Pause 200
        Inc n, Choice(R._areFeetDown(), 1, 0)
      Loop Until n > 10
      Print
      R.switchGaitGen 0
      Pause 500
      R.servosOff
      R.running = 1
      R.RGB_LED RGB_I_PULSE, ,,, 1
      Print "| Done"
      Exit Sub
    EndIf
  EndIf
End Sub

' - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
Sub R._cbHandshake
  ' Interrupt when handshake in-pin goes high (= client signalled)
  hs.t_last_ms = Timer
  Pulse PIN_HS_OUT, 0.5
End Sub


Sub R.updateHandshake
  ' Sets `R.connected` if last handshake was less than `HS_TOUT_MS` ago
  R.connected = (Timer -hs.t_last_ms) < HS_TOUT_MS
  If R.connected Xor (rgb.conn > 0) Then
    rgb.conn = Choice(R.connected, 15, 0)
    R.RGB_LED RGB_I_PULSE, ,,, 1
  EndIf
End Sub

' - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
Sub R._timing t_start, _rep
  ' Track and report loop timing
  Static dt_ms(499)
  Static integer p = 0, first = 1, n = 0
  If first Then
    Math Set 0, dt_ms()
    first = 0
  EndIf
  If _rep = 0 Then
    dt_ms(p) = Timer -t_start
    Inc p, 1 : If p >= 500 Then p = 0
    Inc n, 1
  Else
    Local m = Math(Mean dt_ms()), sd = Math(SD dt_ms())
    Print "Loop "+Str$(m,0,3)+" "+Chr$(177)+" "+Str$(sd,0,3)+" ms (";
    Print "n="+Str$(n)+", ";
    Print Str$(Math(Min dt_ms()),0,3)+".."+Str$(Math(Max dt_ms()),0,3)+")"
  EndIf
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

' - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
Sub R.doServoCalib
  ' Servo calibration helper
  Local a_deg(LEG_N-1, 2), p(SRV_N-1)
  Math Set 0, a_deg()
  Print "Calibrate servos"
  R.logServoCalibVals
  Print "| Moving all servos to 0 deg ..."
  R.Angles2Pos a_deg(), p()
  R._moveAllServos p()
  Print "| Done."
End Sub


Sub R.logServoCalibVals
  ' List calibration values
  Local integer j
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
End Sub

' ===========================================================================
' Postures & moves
' ---------------------------------------------------------------------------
Sub R.updatePosture
  ' Start, update, or finish posture
  If R.showingOff = 0 Then Exit Sub
  If pst.state(2) = 0 Then
    ' Posture sequence has ended
    R.showingOff = 0
    pst.state(0) = 0 ' posture ID
    Print "| Posture done."
    Exit Sub
  EndIf
  Local integer i = pst.state(1), t = pst.state(3)
  Local v(11)
  If t = 0 Or Timer > t Then
    ' Perform next step ...
    ' dt_ms, bo_y, tl_x, tl_z, tr_y, lh, bp_x, bp_z, br_x, br_y, br_z, ds
    Print "step=";i
    Math Slice pst.data(), ,i, v()
    R.inputBody v(1), v(5)
    R.inputPosition v(6), v(7), v(8), v(9), v(10)
    R.inputWalk v(2), v(3), v(4), v(11)
    Inc pst.state(1), 1
    Inc pst.state(2), -1
    pst.state(3) = Timer +pst.data(0,i)
  EndIf
End Sub

' ===========================================================================
' Communication w/ client
' ---------------------------------------------------------------------------
Sub COM.Open
  ' Open communication to client
  Print "Connecting to client via ";
  If Not(USE_I2C_COM) Then
    Print "COM1 ... "
    SetPin GP1, GP0, COM1
    Open "COM1:115200" As #2
  Else
    Print "as I2C device ... "
    SetPin GP0, GP1, I2C
    I2C Slave Open COM_I2C_ADDR, COM._send, COM._receive
  EndIf
  com.isReady = 1
  Print "| Ready."
End Sub


Sub COM.Close
  ' Close communication to client
  If Not(com.IsReady) Then Exit Sub
  If Not(USE_I2C_COM) Then
    Close #2
  Else
    I2C Slave Close
  EndIf
  com.isReady = 0
End Sub

' - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
' Used if `USE_I2C_COM` = 1
`
Sub COM._send
  ' Called when client (master) is expecting data
  ' Checks if last command was `CMD_STATE` and if so, send state data
  If DEBUG And DEB_COM Then Print "Client expects data."
  Local integer cmd = Asc(Mid$(com.i2cBuf$, 9, 1)) -COM_DATA_OFFS
  If cmd = CMD_STATE Then COM.sendState
End Sub


Sub COM._receive
  ' Called when data is received from the client (master)
  ' Read data and call `COM.handleMsg`, if data
  If DEBUG And DEB_COM Then Print "Client sent data."
  I2C Slave Read COM_MSG_LEN, com.i2cBuf$, com.i2cRecv
  If DEBUG And DEB_COM Then Print com.i2cRecv;" byte(s), MM.I2C=";MM.I2C;"."
End Sub

' - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
Sub COM.handleMsg
  ' Check if a new message from the client is available, and if so
  ' handle it, if possible
  Static integer first = 1, pbuf, pv, v(COM_MSG_LEN/8 -1)
  Static String buf$
  Local integer n, j1,nj, ip, ib, j
  If first Then
    pbuf = Peek(VarAddr buf$)
    pv = Peek(VarAddr v())
    first = 0
  EndIf
  com.in_res = 0

  ' Check if communication link is ready and if there is data waiting
  If Not(com.isReady) Then Exit Sub
  If Not(USE_I2C_COM) Then
    ' Read from serial port if data is waiting
    If Loc(#2) = 0 Then Exit Sub
    Line Input #2, buf$
  Else
    ' Get data read by I2C interrupt, if any
    If com.i2cRecv = 0 Then Exit Sub
    buf$ = com.i2cBuf$
    com.i2cRecv = 0
  EndIf

  ' Check data
  com.in_res = RES_NEW_MSG
  n = Peek(Byte pbuf)
  If n <> COM_MSG_LEN Then
    ' Error: Length invalid
    com.in_res = com.in_res Or RES_ERROR
    Exit Sub
  EndIf
  R.RGB_LED 1, 20,10,0

  ' Parse the message, starting with the time of message
  ' (as hexlified uint32)
  com.in_t_ms = Val("&H"+Mid$(buf$,1,8))

  ' Unpack single-byte values into an array
  Memory Copy pbuf+9, pv, COM_N_BYTEVAL
  Memory Unpack v(), com.in(), COM_N_BYTEVAL, 8
  Math Add com.in(), -COM_DATA_OFFS, com.in()

 'If DEBUG > 0 Then Print "<- "+Field$(CMD_STR$, com.in(0))
  If DEBUG And DEB_COM Then
    Print "|<Parameters"
    Math V_print com.in()
  EndIf

  ' Extract data array
  '      0 : command code
  '      1 : n data bytes (following), currenly 12
  '      2 : bodyYOffs
  '      3 : legLiftH
  '    4,5 : x_zTravL
  '      6 : travRotY
  '      7 : delaySpeed/10
  '      8 : buzzer signal
  '      9 : command extension
  '  10,11 : x_zBody
  '  12-14 : xyzBodyRot
  If com.in(0) = CMD_SHOW Then
    ip = com.in(9)
    ib = com.in(8)
    If ip > 0 Then
      ' Showing posture
      R.showingOff = 1
      pst.state(0) = ip   ' posture #
      j1 = pst.n(ip-1, 0)
      nj = pst.n(ip-1, 1)
      pst.state(1) = j1   ' start row in `pst.data()`
      pst.state(2) = nj   ' n steps
      pst.state(3) = 0    ' time of last step (ms)
      Print "|<Showing #"+Str$(ip)+" "+Str$(j1)+"..."+Str$(j1+nj-1)
    ElseIf ib > 0 Then
      ' Play beep
      For j=0 To 20
        'R.Buzz 110+Int(Math(Rand)*400), Int(Math(Rand)*50 +10)
        'R.Buzz 110+Int(Math(Rand)*220 +(100-j)*30), 2
        R.Buzz 110+Int(Math(Rand)*220), 30
      Next
    EndIf
    com.in_res = com.in_res Or RES_HANDLED

  ElseIf com.in(0) = CMD_BODY Then
    ' Body posture
    Inc com.in(2), -BODY_Y_OFFS_MIN
    Inc com.in(3), -LEG_LIFT_MIN
    R.inputBody com.in(2), com.in(3)
    Inc com.in(10), -BODY_X_Z_POS_LIM(0)
    Inc com.in(11), -BODY_X_Z_POS_LIM(2)
    Inc com.in(12), -BODY_XYZ_ROT_LIM(0)
    Inc com.in(13), -BODY_XYZ_ROT_LIM(1)
    Inc com.in(14), -BODY_XYZ_ROT_LIM(2)
    R.inputPosition com.in(10),com.in(11), com.in(12),com.in(13),com.in(14)
    com.in_res = com.in_res Or RES_HANDLED

  ElseIf com.in(0) = CMD_MOVE Then
    ' Movement
    Inc com.in(4), -TRAVEL_X_Z_LIM(0)
    Inc com.in(5), -TRAVEL_X_Z_LIM(2)
    Inc com.in(6), -TRAV_ROT_Y_LIM
    com.in(7) = com.in(7) *10
    R.inputWalk com.in(4), com.in(5), com.in(6), com.in(7)
    com.in_res = com.in_res Or RES_HANDLED

  ElseIf com.in(0) = CMD_STOP Then
    ' Stop all motion
    com.in(7) = com.in(7) *10
    R.inputWalk 0,0,0, com.in(7)
    com.in_res = com.in_res Or RES_HANDLED

  ElseIf com.in(0) = CMD_POWER Then
    ' Power on or off, depending on state
    If Not(com.in(9) = R.running) Then R.Power Choice(com.in(9) = 2, 1,0)
    com.in_res = com.in_res Or RES_HANDLED

  ElseIf com.in(0) = CMD_STATE Then
    ' Ignore
    Print "here"
    com.in_res = com.in_res Or RES_HANDLED
  EndIf

  If com.in_res And RES_HANDLED Then
  Else
    Print "|<Error: Invalid command ("+Str$(com.in(0))+")"
    com.in_res = com.in_res Or RES_ERROR
  EndIf
  R.RGB_LED 1, 0,0,0
End Sub

' - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
Sub COM.sendState
  ' Send state message to client
  Static integer first = 1, pbuf, pv
  Static string buf$ length COM_N_BYTEVAL
  Static string out$ length COM_MSG_LEN
  Static integer _out(COM_N_BYTEVAL-1), v(COM_N_BYTEVAL/8 -1)
  If first Then
    pbuf = Peek(VarAddr buf$)
    Poke Byte pbuf, COM_N_BYTEVAL
    pv = Peek(VarAddr v())
    first = 0
  EndIf
  If DEBUG And DEB_COM Then Print "Sending ..."
  R.RGB_LED 1, 0,10,20

  ' Compile data array
  ' (assumes that the parameters are already range checked)
  '      0 : command code
  '      1 : n data bytes (following), currenly 17
  '      2 : servo battery voltage [V*10]
  '      3 : logic battery voltage [V*10]
  '    4,5 : compass heading [degree]
  '    6,7 : compass pitch and role [-90..90, degree]
  '      8 : GGN state
  '  9..16 : servo load for servos #0..7, 0..240 [a.u.]
  Math Set 0, _out()
  _out(0) = CMD_STATE
  _out(1) = 11
  _out(2) = Int(R.servoBattery_V() *10)
  _out(3) = Int(R.logicBattery_V() *10)
  _out(4) = IMU(0) \10
  _out(5) = IMU(0) Mod 10
  _out(6) = IMU(1) +90
  _out(7) = IMU(2) +90
  _out(8) = ggn.state
  _out(9) = Int(srv.load(0) And &HEF)
  _out(10)= Int(srv.load(1) And &HEF)
  _out(11)= Int(srv.load(2) And &HEF)
  ' TODO: add servo load data to _out(9-16)
  If DEBUG And DEB_COM Then
    Print "|>Parameters (offset w/ limits)"
    Math V_print _out()
  EndIf

  ' Pack single-byte values into a string
  Math Add _out(), COM_DATA_OFFS, _out()
  Memory Pack _out(), v(), COM_N_BYTEVAL, 8
  Memory Copy pv, pbuf+1, COM_N_BYTEVAL

  ' Send message
  out$ = Hex$(Timer, 8) +buf$
  If Not(USE_I2C_COM) Then
    Print #2, out$
  Else
    I2C Slave Write COM_MSG_LEN, out$
  EndIf
  R.RGB_LED 1, 0,0,0
End Sub

' ===========================================================================
' Walk engine
' ---------------------------------------------------------------------------
Sub R.resetGaitGen
  ' Reset gait generator (ggn)
  Static integer first = 1
  If first Then
    ' Allocate variables
    first = 0
    Dim integer ggn.isOnPrev, ggn.state, ggn.nextState, ggn.feetDown
    Dim integer ggn.lastErr, ggn.IKErr
    Dim integer ggn.dtMov, ggn.dtPrevMov, ggn.dtNextMov, ggn.dtLastCalc
    Dim integer ggn.tWaitUntil, ggn.dtLastSpin, ggn.dtMoveLate
    Dim integer ggn.tMoveStart
    Dim ggn.xyzLeg(LEG_N-1, 2), ggn.xyzFoot(LEG_N-1, 2)
    Dim ggn.legAng(LEG_N-1, 2), ggn.srvAng(LEG_N-1, 2)
    Dim ggn.srvPos(SRV_N-1)
  EndIf
  ' Reset GGN control parameters
  ' (all times are in [ms])
  ggn.isOnPrev   = 0         ' Previous state of GGN on/off switch
  ggn.state      = GGN_IDLE  ' Current and next state of GGN
  ggn.nextState  = ggn.state
  ggn.dtMov      = 0         ' Durations [ms] ...
  ggn.dtPrevMov  = 0
  ggn.dtNextMov  = 0
  ggn.dtLastCalc = 0
  ggn.dtLastSpin = 0
  ggn.dtMoveLate = 0
  ggn.tWaitUntil = 0         ' Wait until this time
  ggn.tMoveStart = 0         ' Time when move last was started
  ggn.feetDown   = 1
  ggn.lastErr    = ERR_OK
  ggn.IKErr      = 0

  ' Reset input parameters
  R.resetInput

  ' Current leg (`ggn.xyzLeg`) and foot (`ggn.xyzFoot`) positions
  Math Scale FEET_INIT_XYZ(), 1, ggn.xyzLeg()
  Math Set 0, ggn.xyzFoot()

  ' Current leg angles (`ggn.legAng`, from IK), converted into servo
  ' angles (`ggn.srvAng`) and timing (`ggn.srvPos`)
  Math Set 0, ggn.legAng()
  Math Set 0, ggn.srvAng()
  Math Set 0, ggn.srvPos()
End Sub

' - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
Sub R.switchGaitGen state%
  ' Start (`state% = 1`) or stop gait generator
  in.doEmergStop = 0
  in.isOn = state% <> 0
  ggn.state = Choice(in.isOn, GGN_COMPUTE, GGN_STAND_BY)
End Sub

' - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
Sub R.spinGaitGen
  ' Keeps everything running; needs to be called frequencly
  Local t = Timer, _t
  If ggn.state = GGN_MOVING Then
    ' Still executing move
    If ggn.tWaitUntil > 0 And t >= ggn.tWaitUntil And srv.isMoveDone  Then
      ' Trigger execution of next move
      ggn.state = GGN_DO_SERVOS
      ggn.tWaitUntil = 0
    EndIf
  ElseIf ggn.state = GGN_DO_SERVOS Then
    ' Previous move is finished, now commit next move
   '_t = Timer
    R._executeMove
   '_t = Timer -_t : Print "exec="+Str$(_t,0,3)
  ElseIf ggn.state = GGN_COMPUTE Then
    ' Calculate next move and set timer for move execution
   '_t = Timer
    R._computeMove
   '_t = Timer -_t : Print "comp="+Str$(_t,0,3)
  EndIf
End Sub

' - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
Sub R._computeMove
  ' Compute next move
  Local xyz1(2), xyz2(2), xyz3(2)
  Local integer iL
  Local t1, t2, dt, _t

  ' Initialize
  If DEBUG And DEB_VERB Then Print "R._computeMove"
  R.LED 1
  t1 = Timer
  ggn.lastErr = ERR_OK
  ggn.IKErr = 0

  ' Update input parameters (e.g., blending) and legs to their initial
  ' positions
  R.inputUpdate
  Math Scale FEET_INIT_XYZ(), 1, ggn.xyzLeg()

  ' Calculate update of gate sequence
  g.isInMot = R._isGaitInMotion(1)
  R.generateGaitSeq

  ' Do IK for all legs
  '  ggn.xyzLeg(LEG_N-1,2)  (lgxyz)
  '  in.xyzBody_v(2)        (boxyz)
  '  in.xyzBodyRot_v(2)     (brxyz)
  '  g.xyz(LEG_N-1,2)       (gaxyz)
  '  COX_OFF_XYZ(LEG_N-1,2) (coxyz)
  '_t = Timer
  For iL=0 To LEG_N-1
    ' Target foot positions `xyz1()`
    xyz1(0) = -ggn.xyzLeg(iL,0) +in.xyzBody_v(0) +g.xyz(iL,0)
    xyz1(1) =  ggn.xyzLeg(iL,1) +in.xyzBody_v(1) +g.xyz(iL,1)
    xyz1(2) =  ggn.xyzLeg(iL,2) +in.xyzBody_v(2) +g.xyz(iL,2)

    ' Rotation matrix (body) `xyz2()`
    R._bodyIK iL, xyz1(), xyz2()
    ' >==
    'Print "iL=";iL
    'Print "xyz1=";
    'Math V_print xyz1()
    'Print "xyz2=";
    'Math V_print xyz2()
    ' <==

    xyz3(0) =  ggn.xyzLeg(iL,0) -in.xyzBody_v(0) +xyz2(0) -g.xyz(iL,0)
    xyz3(1) =  ggn.xyzLeg(iL,1) +in.xyzBody_v(1) -xyz2(1) +g.xyz(iL,1)
    xyz3(2) =  ggn.xyzLeg(iL,2) +in.xyzBody_v(2) -xyz2(2) +g.xyz(iL,2)
    Math Insert ggn.xyzFoot(), iL,, xyz3()
    R._legIK iL, xyz3()
    ' >==
    'Print "xyz3=";
    'Math V_print xyz3()
    ' <==
  Next
  '_t = Timer -_t : Print "loop="+Str$(_t,0,3)
  ' >==
  'Print "ggn.legAng()="
  'Math M_print ggn.legAng()
  ' <==

  ' Check and report IK errors
  If ggn.IKErr > 0 Then
    If DEBUG And DEB_IK Then Print "ggn.IKErr=";ggn.IKErr
  EndIf

  ' Drive servos, if GGN is on
  If in.isOn Then
    ' Calculate time the next move will take
    If R._isGaitInMotion(2) Then
      ggn.dtNextMov = g.nomSpeed
      Inc ggn.dtNextMov, in.delayInput_v*2 +in.delaySpeed_v
    Else
      ' Movement speed if not walking
      ggn.dtNextMov = 200 +in.delaySpeed_v
    EndIf
    t2 = Timer
    ggn.dtLastCalc = t2 -t1

    ' Send new leg positions to the servo manager
    If (ggn.tMoveStart = 0) Or ((ggn.tMoveStart +ggn.dtMov) < t2) Then
      ' Is first move or next servo update is overdue, in any case execute
      ' the move immediately
      R._executeMove
      ggn.lastErr = ERR_MOVE_OVERDUE
    Else
      ' The last move is not yet finished, we need to wait ...
      ' (for this the timer is used, so that other sections of the main
      '  loop get processing time)
      ggn.state = GGN_MOVING
      ggn.nextState = GGN_DO_SERVOS
      ggn.tWaitUntil = Timer +Max(ggn.dtNextMov -ggn.dtLastCalc, 1)
    EndIf
  Else
    ' Turn GGN off
    If Not(R._areFeetDown()) Then
      ggn.dtMov = 600
      R._executeMove()
    EndIf
  EndIf

  R.LED 0
  ' >==
  'Print "#_dtNextMov=";ggn.dtNextMov;" dtMov=";ggn.dtMov;
  'Print " dtLastCalc=";ggn.dtLastCalc;" tMoveStart=";ggn.tMoveStart;
  'Print " tWaitUntil=";ggn.tWaitUntil
  'Print Field$(ERR_STR$, ggn.LastErr+1, ",")
  'Print "inMotion=";R._isGaitInMotion();" feetDown=";R._areFeetDown()
  ' <==
End Sub

' - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
Sub R._bodyIK iL%, xyzIn(), xyzOut()
  ' Calculates IK of the body for leg `iL%`, with foot position(`xyzIn(2)`).
  ' Uses `COX_OFF_XYZ(iL%,2)`, `g.rotY(iL%)`, and `in.xyzBodyRot_v(2)`
  ' -> `xyzOut(2)`
  Local d(2), w(2)
  Local rx, ry, rz, sx, sy, sz, cx, cy, cz, _t
 'If DEBUG And DEB_VERB Then Print "R._bodyIK (iL=";iL%;")"

  ' Calculate total offset between center of body and foot
  d(0) = -COX_OFF_XYZ(iL%,0) +xyzIn(0)
  d(1) =  COX_OFF_XYZ(iL%,1) +xyzIn(1)
  d(2) =  COX_OFF_XYZ(iL%,2) +xyzIn(2)

  ' Calculate sin and cos for each rotation
  rx = Rad(in.xyzBodyRot_v(0))
  ry = Rad(in.xyzBodyRot_v(1) +g.rotY(iL%))
  rz = Rad(in.xyzBodyRot_v(2))
  sx = Sin(rx) : sy = Sin(ry) : sz = Sin(rz)
  cx = Cos(rx) : cy = Cos(ry) : cz = Cos(rz)

  ' Calculate rotation matrix (`xyzOut(2)`)
  '_t = Timer
  w(0) = d(0)*cy*cz -d(2)*cz*sy +xyzIn(1)*sz
  w(1) = d(0)*sy*sx -d(0)*cy*cx*sz +d(2)*cy*sx +d(2)*cx*sy*sz +xyzIn(1)*cz*cx
  w(2) = d(0)*cx*sy +d(0)*cy*sz*sx +d(2)*cy*cx -d(2)*sy*sz*sx -xyzIn(1)*cz*sx
  Math C_Sub d(), w(), xyzOut()
  '_t = Timer -_t : Print "rot_="+Str$(_t,0,3)
End Sub

' - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
Sub R._legIK iL%, xyzIn()
  ' Calculates the angles of coxa, femur and tibia (`ggn.legAng(LEG_N-1,2)`)
  ' for the given foot position (`xyzIn(2)`) of leg (`iL%`).
  ' All intermediate angles are in radians.
  ' -> ggn.legAng(iL%, )
  Local dCF, aCx, IKSW, IKA1, IKA2, v, w, a, b
 'If DEBUG And DEB_VERB Then Print "R._legIK (iL=";iL%;")"

  ' Calculate coxa angle
  ' (w/ `dCF` the length between coxa and foot; make sure that angle is
  '  between +/- 180)
  dCF = Sqr(xyzIn(0)^2 +xyzIn(2)^2)
  aCx = Atan2(xyzIn(2), xyzIn(0))
  a = Deg(aCx) +COX_OFF_ANG(iL%)
  ggn.legAng(iL%,COX) = Choice(Abs(a) > 180, a-Sgn(a)*360, a)
  ' >==
  'Print Format$(xyzIn(0),"%8.3f");" ";Format$(xyzIn(2),"%8.3f")
  'Print Format$(dCF,"%8.3f");" ";Format$(aCx,"%8.3f");" ";
  'Print Format$(ggn.legAng(iL%,COX),"%8.3f")
  ' <==

  ' Solving `IKA1`, `IKA2` and `IKSW`
  ' w/ `IKA1` the angle between SW line and the ground in radians
  '    `IKSW` the distance between femur axis and tars
  '    `IKA2` the angle between the line S>W with respect to femur
  b = dCF -COX_LEN
  IKSW = Sqr(xyzIn(1)^2 +b^2)
  IKA1 = Atan2(b, xyzIn(1))
  v = FEM2TIB2_DIF +IKSW^2
  w = v/(FEM_2LEN *IKSW)
  If w < -1 Or w > 1 Then Inc ggn.IKErr, 10 : w = Sgn(w) : EndIf
  IKA2 = ACos(w)
  ' >==
  'Print Format$(xyzIn(1),"%8.3f");" ";Format$(IKSW,"%8.3f");" ";
  'Print Format$(IKA1,"%8.3f");" "
  ' <==

  ' Calculate femur and tibia angles
  ggn.legAng(iL%,FEM) = -Deg(IKA1 +IKA2) +90
  w = v /FEMTIB_2LEN
  If w < -1 Or w > 1 Then Inc ggn.IKErr, 1 : w = Sgn(w) : EndIf
  ggn.legAng(iL%,TIB) = TIB_CORR_ANG -Deg(ACos(w))
End Sub

' - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
Sub R._executeMove
  ' Executes the move based on leg angles (`ggn.legAng(LEG_N-1, 2)`)
  ' and updates the GGN status and timing parameters
  Local t
  If DEBUG And DEB_VERB Then Print "R._executeMove"

  ' Convert leg angles to servo angles and then to positions (in [ms])
  Math C_MULT ggn.legAng(), srv.ang_f(), ggn.srvAng()
  R.Angles2Pos ggn.srvAng(), ggn.srvPos()
  If DEBUG And DEB_SRV Then
    Print "---Servos---"
    Math M_print ggn.srvAng()
  EndIf

  ' Send positions to servos
  ' (duration of move `ggn.dtMov`)
  R._moveAllServos ggn.srvPos(), ggn.dtMov

  ' Keep track of timing
  t = Timer
  ggn.dtMoveLate = t -ggn.tMoveStart -ggn.dtPrevMov
  ggn.tMoveStart = t
  ggn.dtPrevMov = ggn.dtMov
  ggn.dtMov = ggn.dtNextMov
  ggn.state = GGN_COMPUTE

  ' >==
  'Print "exe _dtMoveLate=";ggn.dtMoveLate;" dtPrevMov=";ggn.dtPrevMov
  ' <==
End Sub

' - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
Function R._areFeetDown()
  ' Check if all feet are down
  ' (consider body position)
  Local v(LEG_N-1, 2), w(LEG_N-1, 2), u(LEG_N-1)
  Math Set 0, w()
  Math Set in.xyzBody_v(0), u()
  Math Insert w(), ,0, u()
  Math Set in.xyzBody_v(1), u()
  Math Insert w(), ,1, u()
  Math Set in.xyzBody_v(2), u()
  Math Insert w(), ,2, u()
  Math C_Sub ggn.xyzFoot(), FEET_INIT_XYZ(), v()
  Math C_Sub v(), w(), v()
  R._areFeetDown = Abs(Math(SUM v())) <= LEGS_DOWN_DEAD_Z
End Function


Function R._isGaitInMotion(zFactor)
  ' Returns True if one of the travel lengths are outside the range
  ' considered as the dead zone, that means basically in motion
  Local integer res
  res = Abs(in.x_zTravL_v(0)) > TRAVEL_DEAD_Z
  res = res Or (Abs(in.x_zTravL_v(2)) > TRAVEL_DEAD_Z)
  R._isGaitInMotion = res Or (Abs(in.travRotY_v *zFactor) > TRAVEL_DEAD_Z)
End Function

' ===========================================================================
' Input-related routines
' ---------------------------------------------------------------------------
Sub R.resetInput
  ' Define input control variables
  Static integer first = 1
  ResetIn: Restore ResetIn
  If first Then
    ' Allocate variables
    first = 0
    Dim integer in.isOn
    Dim integer in.doEmergStop

    ' Each parameter is described in addition by an array, with:
    ' (target_value, value_inc, n_steps_to_target, min, max)
    ' Note: The first 3 parameters are used for value "blending".
    Dim in.bodyYOffs_v, in.bodyYOffs(4)
    Dim in.bodyYShft_v, in.bodyYShft(4)
    Dim in.x_zTravL_v(2), in.x_zTravL(4,2)
    Dim in.travRotY_v, in.travRotY(4)
    Dim in.legLiftH_v, in.legLiftH(4)
    Dim in.xyzBody_v(2), in.xyzBody(4,2)
    Dim in.xyzBodyRot_v(2), in.xyzBodyRot(4,2)
    Dim in.delaySpeed_v, in.delaySpeed(4)
    Dim integer in.delayInput_v, in.delayInput(4)
   'Dim in._data(1,11)
  EndIf
  ' Status
  in.isOn = 0
  in.doEmergStop = 0

  ' Body offset (0=down, 35=default up) and y-shift
  in.bodyYOffs_v = 0
  Data 0, 0, 0, BODY_Y_OFFS_MIN, BODY_Y_OFFS_MAX
  Read in.bodyYOffs()
  in.bodyYShft_v = g.bodyYOffs
  Data 0, 0, 0, -BODY_Y_SHFT_LIM, BODY_Y_SHFT_LIM
  Read in.bodyYShft()

  ' Current travel length (x,z), rotation (y), and height
  Math Set 0, in.x_zTravL_v()
  Data 0, 0, 0, -TRAVEL_X_Z_LIM(0), TRAVEL_X_Z_LIM(0)
  Data 0, 0, 0, -TRAVEL_X_Z_LIM(1), TRAVEL_X_Z_LIM(1)
  Data 0, 0, 0, -TRAVEL_X_Z_LIM(2), TRAVEL_X_Z_LIM(2)
  Read in.x_zTravL()
  in.travRotY_v = 0
  Data 0, 0, 0, -TRAV_ROT_Y_LIM, TRAV_ROT_Y_LIM
  Read in.travRotY()
  in.legLiftH_v = g.legLiftHDef
  Data 0, 0, 0, LEG_LIFT_MIN, LEG_LIFT_MAX
  Read in.legLiftH()

  ' Global input for body position (xyzBodyPos[1] is calculated),
  ' pitch (x), rotation (y), and roll (z)
  Math Set 0, in.xyzBody_v()
  Data 0, 0, 0, -BODY_X_Z_POS_LIM(0), BODY_X_Z_POS_LIM(0)
  Data 0, 0, 0, -BODY_X_Z_POS_LIM(1), BODY_X_Z_POS_LIM(1)
  Data 0, 0, 0, -BODY_X_Z_POS_LIM(2), BODY_X_Z_POS_LIM(2)
  Read in.xyzBody()
  Math Set 0, in.xyzBodyRot_v()
  Data 0, 0, 0, -BODY_XYZ_ROT_LIM(0), BODY_XYZ_ROT_LIM(0)
  Data 0, 0, 0, -BODY_XYZ_ROT_LIM(1), BODY_XYZ_ROT_LIM(1)
  Data 0, 0, 0, -BODY_XYZ_ROT_LIM(2), BODY_XYZ_ROT_LIM(2)
  Read in.xyzBodyRot()

  ' Movement speed via adjustible delay [ms] and input-depdent delay
  in.delaySpeed_v = 80
  Data 0, 0, 0, 0, DELAY_SPEED_MAX
  Read in.delaySpeed()
  in.delayInput_v = 0
  Data 0, 0, 0, DELAY_INPUT_MIN, DELAY_INPUT_MAX
  Read in.delayInput()
End Sub

' - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
Sub R.inputUpdate
  ' Some input parameters are ramped slowly to their target values; this
  ' is done my frequently calling this routine
  ' TODO: Blending
  ' ...
  in.xyzBody_v(1) = Max(in.bodyYOffs_v +in.bodyYShft_v, 0)
  If Not(DEBUG And DEB_INP) Then Exit Sub
End Sub

/*
Sub R._inputCopyToData _i
  ' Copies current input data into row `_i` of `in._data`
  If _i < 0 Or _i > 1 Then Exit Sub
  in._data(_i,1)  = in.bodyYOffs_v
  in._data(_i,2)  = in.x_zTravL_v(0)
  in._data(_i,3)  = in.x_zTravL_v(2)
  in._data(_i,4)  = in.travRotY_v
  in._data(_i,5)  = in.legLiftH_v
  in._data(_i,6)  = in.xyzBody_v(0)
  in._data(_i,7)  = in.xyzBody_v(2)
  in._data(_i,8)  = in.xyzBodyRot_v(0)
  in._data(_i,9)  = in.xyzBodyRot_v(1)
  in._data(_i,10) = in.xyzBodyRot_v(2)
  in._data(_i,11) = in.delaySpeed_v
End Sub
*/

Sub R.inputLog _as_data
  ' Log the current input data
  Local integer j
  Local v(11), w(11), sum
  If _as_data Then
    ' Log data if somewhat different from the last data
    ' last data=`in._data(0)`
    /*
    R._inputCopyToData 1
    Math Slice in._data(), 0,, w()
    Math Slice in._data(), 1,, v()
    Math C_Sub w(), v(), w()
    Math C_Mult w(), w(), w()
    sum = Math(Sum w())
    If sum > 2 Then
      Print "Data ";
      For j=0 To 11
        Print Str$(in._data(1,j),2,1)+Choice(j<11, ",", "\r\n");
      Next
    EndIf
    Math Insert in._data(), 0,, v()
    */
    Exit Sub
  EndIf
  Print "in.bodyYOffs =";Str$(in.bodyYOffs_v)
  Print "in.bodyYShft =";Str$(in.bodyYShft_v)
  Print "in.x_zTravL  =";Str$(in.x_zTravL_v(0));",";Str$(in.x_zTravL_v(2))
  Print "in.travRotY  =";Str$(in.travRotY_v)
  Print "in.legLiftH  =";Str$(in.legLiftH_v)
  Print "in.xyzBody   =";Str$(in.xyzBody_v(0));",";Str$(in.xyzBody_v(1));
  Print ","Str$(in.xyzBody_v(2))
  Print "in.xyzBodyRot=";Str$(in.xyzBodyRot_v(0));",";Str$(in.xyzBodyRot_v(1));
  Print ",";Str$(in.xyzBodyRot_v(2))
  Print "in.delaySpeed=";Str$(in.delaySpeed_v)
  Print "in.delayInput=";Str$(in.delayInput_v)
End Sub

' - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
Sub R.inputBody bo_y, lh
  ' Change body-related parameters
  '   bo_y := bodyYOffs; 0=down, 35=default up
  '   lh   := legLiftH; current travel height
  ' TODO: Blending
  in.bodyYOffs_v = Min(Max(bo_y, in.bodyYOffs(3)), in.bodyYOffs(4))
  in.legLiftH_v  = Min(Max(lh, in.legLiftH(3)), in.legLiftH(4))
  If Not(DEBUG And DEB_INP) Then Exit Sub
  Print "R.inputBody| ";
  Print "bo_y=";in.bodyYOffs_v;" lh=";in.legLiftH_v
End Sub


Sub R.inputPosition ps_x, ps_z, pr_x, pr_y, pr_z
  ' Change position-related parameters
  '   ps_x, ps_z       := relative body position in the plane
  '   pr_x, pr_y, pr_z := rotation around x,y,z axes
  in.xyzBody_v(0) = Min(Max(ps_x, in.xyzBody(3,0)), in.xyzBody(4,0))
  in.xyzBody_v(2) = Min(Max(ps_z, in.xyzBody(3,2)), in.xyzBody(4,2))
  in.xyzBodyRot_v(0) = Min(Max(pr_x, in.xyzBodyRot(3,0)), in.xyzBodyRot(4,0))
  in.xyzBodyRot_v(1) = Min(Max(pr_y, in.xyzBodyRot(3,1)), in.xyzBodyRot(4,1))
  in.xyzBodyRot_v(2) = Min(Max(pr_z, in.xyzBodyRot(3,2)), in.xyzBodyRot(4,2))
  If Not(DEBUG And DEB_INP) Then Exit Sub
  Print "R.inputBody| ";
  Print "ps_x,z=";in.xyzbody_v(0);",";in.xyzbody_v(2);" ";
  Print "pr_x,y,z=";in.xyzbodyRot_v(0);",";in.xyzbodyRot_v(1)",";
  Print in.xyzbodyRot_v(2)
End Sub


Sub R.inputWalk tl_x, tl_z, tr_y, ds
  ' Change walking-related parameters
  '   tl_x, tl_z := x_zTravL; current travel length X,Z
  '   tr_y       := travRotY; current travel rotation Y
  '   ds         : delaySpeed; adjustible delay [ms]
  ' TODO: Blending
  in.doEmergStop = 0
  in.x_zTravL_v(0) = Min(Max(tl_x, in.x_zTravL(3,0)), in.x_zTravL(4,0))
  in.x_zTravL_v(2) = Min(Max(tl_z, in.x_zTravL(3,2)), in.x_zTravL(4,2))
  in.travRotY_v    = Min(Max(tr_y, in.travRotY(3)), in.travRotY(4))
  in.delaySpeed_v  = Min(Max(ds, in.delaySpeed(3)), in.delaySpeed(4))
  If Not(DEBUG And DEB_INP) Then Exit Sub
  Print "R.inputWalk| ";
  Print "tl_x_z=";in.x_zTravL_v(0);",";in.x_zTravL_v(2);" ";
  Print "tr_y=";in.travRotY_v;" ds=";in.delaySpeed_v
End Sub

' ===========================================================================
' Gait-related routines
' ---------------------------------------------------------------------------
Sub R.resetGait
  ' Define gait
  Static integer first = 1
  resetG: Restore resetG
  If first Then
    first = 0
    Dim integer g.iSt, g.iLegIn, g.isInMot, g.isLastLeg
    Dim integer g.nSt, g.isHalfLiftH, g.TLDivFact, g.nPosLift
    Dim g.legLiftHDef, g.bodyYOffs, g.nomSpeed
    Dim g.xyz(LEG_N-1, 2)
    Dim g.legNr(LEG_N-1), g.rotY(LEG_N-1)
  EndIf
  ' Control variables
  g.iSt         = 1  ' Index of current gait step
  g.iLegIn      = 0  ' Input number of the leg
  g.isInMot     = 0  ' 1=gait is in motion
  g.isLastLeg   = 0  ' 1=current leg is last leg of sequence

  ' Relative x,y,z positions and y rotations
  Math Set 0, g.xyz()
  Math Set 0, g.rotY()

  ' Gait-specific
  g.nSt         = 8  ' Number of steps in gait
  g.legLiftHDef = 65 ' Default travel height (50)
  g.bodyYOffs   = 10
  g.isHalfLiftH = 1  ' Outer positions of lifted half height
  g.TLDivFact   = 4  ' n steps w/ leg on the floor (walking)
  g.nPosLift    = 3  ' n positions single leg is lifted
  g.nomSpeed    = 70 ' Nominal speed of the gait
  Data 5,1,5, 1,5,1  ' Initial leg positions
  Read g.legNr()
End Sub

' - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
Sub R.generateGaitSeq
  ' Generate the gait sequence for all legs
  ' Uses `g.xxx`  - gait-related variables
  '      `in.xxx` - input parameters (movement)
  Local integer iL, lNr, res1, res2, iS, nL, j
  Local float x, y, z, yr, lh
  Local integer stopped

  ' If gait is not in motion, reset travel parameters and gait sequence
  stopped = Not(g.isInMot) And R._areFeetDown()
  If stopped  Then
    Math Set 0, in.x_zTravL_v()
    in.travRotY_v = 0
    g.iSt = 1
  EndIf
  iS  = g.iSt
  nL  = g.nPosLift

  ' Proceed with gait ...
  For iL=0 To LEG_N-1
    x   = g.xyz(iL,0)
    y   = g.xyz(iL,1)
    z   = g.xyz(iL,2)
    yr  = g.rotY(iL)
    lNr = g.legNr(iL)

    ' Leg middle up position
    res1 = g.isInMot And (nL=1 Or nL=3) And iS=lNr
    res2 = Not(g.isInMot) And (iS=lNr)
    res2 = res2 And ((Abs(x)>2) Or (Abs(z)>2) Or (Abs(yr)>2))
    If res1 Or res2 Then
      ' Up ...
      g.xyz(iL,0) = 0
      g.xyz(iL,1) = -in.legLiftH_v
      g.xyz(iL,2) = 0
      g.rotY(iL)  = 0
    Else
      ' Optional half heigth rear
      res1 = (nL=2 And iS=lNr) Or (nL=3 And ((iS=lNr-1) Or (iS=lNr+g.nSt-1)))
      If g.isInMot And res1 Then
        lh = in.legLiftH_v
        g.xyz(iL,0) = -in.x_zTravL_v(0) /2
        g.xyz(iL,1) = Choice(g.isHalfLiftH, -lh /2, -lh)
        g.xyz(iL,2) = -in.x_zTravL_v(2) /2
        g.rotY(iL)  = -in.travRotY_v /2
      Else
        ' Optional half heigth front
        If g.isInMot And nL>=2 And ((iS=lNr+1) Or (iS=lNr-(g.nSt-1))) Then
          lh = in.legLiftH_v
          g.xyz(iL,0) = in.x_zTravL_v(0) /2
          g.xyz(iL,1) = Choice(g.isHalfLiftH, -lh /2, -lh)
          g.xyz(iL,2) = in.x_zTravL_v(2) /2
          g.rotY(iL)  = in.travRotY_v /2
        Else
          ' Leg front down position
          If (iS=(lNr+nL) Or iS=(lNr-(g.nSt-nL))) And y<0 Then
            g.xyz(iL,0) = in.x_zTravL_v(0) /2
            g.xyz(iL,1) = 0
            g.xyz(iL,2) = in.x_zTravL_v(2) /2
            g.rotY(iL)  = in.travRotY_v /2
          Else
            ' Move body forward
            g.xyz(iL,0) = x -in.x_zTravL_v(0) /g.TLDivFact
            g.xyz(iL,1) = 0
            g.xyz(iL,2) = z -in.x_zTravL_v(2) /g.TLDivFact
            Inc g.rotY(iL), -in.travRotY_v /g.TLDivFact
          EndIf
        EndIf
      EndIf
    EndIf
  Next
  If Not(stopped) Then
    ' Advance to the next step
    Inc g.iSt, 1
    If g.iSt > g.nSt Then g.iSt = 1
  EndIf

  ' Exit, if debug info is not requested
  If Not(DEBUG And DEB_GAIT) Then Exit Sub
  Print "g.iSt =";g.iSt
  For j=0 To LEG_N-1
    Print "  iLeg=";g.legNr(j);" xyz=";Str$(g.xyz(j,0),3,1);",";
    Print Str$(g.xyz(j,1),3,1);",";Str$(g.xyz(j,2),3,1);" ";
    Print "yrot=";Str$(g.rotY(j),3,1)
  Next j
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

' - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
Sub R._moveAllServos t_us(), dur_ms, n_srv%, wait%, no_move%
  ' Move the first `n_srv%` servos, or, if 0, all `SRV_N` servos to positions
  ' `t_us%()`. If `dur_ms` > 0, then it will attempt to move the servos such
  ' that they arrive at their targets at the same time (after `dur_ms`).
  ' Otherwise, servos are moved immediately.
  ' If `no_move%`, only internal position is updated.
  ' (NO PARAMETER CHECKING, starts with servo #0)
  If DEBUG And DEB_VERB Then Print "R._moveAllServos"
  Static String cmd$ length 40
  Static integer t(SRV_N-1), lhb, caddr = 0
  Static v(SRV_N-1), w(SRV_N-1)
  Local integer i, j, n = Choice(n_srv% = 0, SRV_N, n_srv%)
  Local dt

  ' Prepare if first call
  If caddr = 0 Then
    cmd$ = "\&9F"+Chr$(n)+Chr$(0)+Space$(SRV_N*2)
    caddr = Peek(VARADDR cmd$) +4
  EndIf

  ' Inactivate timer interrupt
  SetTick 0,0,1
  srv.isMoveDone = 1
  srv.iStep = -1

  ' Check if target position is current position
  Math C_Sub t_us(), srv.pCurr(), w()
  Math C_Mult w(), w(), w()
  If Math(Sum w()) < 1 Then Exit Sub

  If no_move% Then
    ' Just update internal servo positions
    Math Scale t_us(), 1, srv.pCurr()
    Exit Sub
  EndIf
  If dur_ms = 0 Then
    ' No move duration is given, just move to target
    Math Scale t_us(), 1, srv.pCurr()
    Math Scale t_us(), 4, t()
  Else
    ' Generate a sequence of steps
    Math C_Sub t_us(), srv.pCurr(), v()
    Math C_Div v(), srv.nStep(), v()
    Math Scale srv.pCurr(), 1, w()
    For i=0 To SRV_N_STEPS-1
      Math C_Add w(), v(), w()
      Math Insert srv.pSteps(), i,, w()
    Next
    Math Slice srv.pSteps(), 0,, w()
    Math Scale w(), 4, t()
    dt = dur_ms /SRV_N_STEPS
    srv.iStep = 1
    srv.nSrv = n
    srv.isMoveDone = 0
  EndIf

  ' Move to position `t()`
  j = caddr
  For i=0 To n-1
    lhb = (((t(i) >> 7) And &H7F) << 8) Or (t(i) And &H7F)
    Poke SHORT j, lhb : Inc j, 2
  Next
  Print #5, cmd$

  ' If step sequence, then start interrupt routine
  If srv.iStep >= 0 Then SetTick dt, R._cbServo, 1
  If wait% Then Do While Lof(#5) < 256 : Pause 1 : Loop
End Sub


Sub R._cbServo
  ' INTERNAL: Timer callback for smooth servo movements
  Static integer t(SRV_N-1), i, j, lhb, caddr = 0
  Static String cmd$ length 40
  Static w(SRV_N-1)
  ' Prepare if first call
  If caddr = 0 Then
    cmd$ = "\&9F"+Chr$(SRV_N)+Chr$(0)+Space$(SRV_N*2)
    caddr = Peek(VARADDR cmd$) +4
  EndIf

  ' Get positions and send to servo controller
  Math Slice srv.pSteps(), srv.iStep,, w()
  Math Scale w(), 4, t()
  j = caddr
  For i=0 To srv.nSrv-1
    lhb = (((t(i) >> 7) And &H7F) << 8) Or (t(i) And &H7F)
    Poke SHORT j, lhb : Inc j, 2
  Next
  Print #5, cmd$

  ' Check if move is done ...
  Inc srv.iStep, 1
  If srv.iStep = SRV_N_STEPS Then
    ' Stop timer and set target position as new current position
    SetTick 0,0,1
    Math Slice srv.pSteps(), SRV_N_STEPS-1,, srv.pCurr()
    srv.isMoveDone = 1
    srv.iStep = -1
  EndIf
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

' ---------------------------------------------------------------------------
' Routines to change servo parameters and retrieve information
' - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
Sub R.setServoSpeed i%, speed%
  ' Sets speed of servo `i%`
  Local integer hb, lb
  If i% < 0 Or i% > SRV_N-1 Then Exit Sub
  hb = (speed% >> 7) And &H7F
  lb = speed% And &H7F
  Print #5, "\&87"+Chr$(i%)+Chr$(lb)+Chr$(hb)
End Sub


Sub R.setServoAccel i%, accel%
  ' Sets acceleration of servo `i%`
  Local integer hb, lb
  If i% < 0 And i% > SRV_N-1 Then Exit Sub
  hb = (accel% >> 7) And &H7F
  lb = accel% And &H7F
  Print #5, "\&89"+Chr$(i%)+Chr$(lb)+Chr$(hb)
End Sub

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

' ===========================================================================
' Hexapod board-related routines
' ---------------------------------------------------------------------------
Sub R.LED state%
  ' Set/reset onboard (yellow) LED
  Pin(PIN_LED) = state% <> 0
End Sub

Sub R.Buzz freq%, dur_ms%
  ' Buzz at `freq%` Hz for `dur_ms%` milliseconds
  Local integer f = Max(Min(freq%, 10000), 0)
  PWM 3, f,50
  If dur_ms% > 0 Then Pause dur_ms% : PWM 3, f,0 : EndIf
End Sub

Function R.servoBattery_V()
  ' Return servo battery voltage in [V]
  R.servoBattery_V = Pin(PIN_AIN_SRV_BAT) *3.54
End Function

Function R.logicBattery_V()
  ' Return logic battery voltage in [V]
  ' U2 = U/(10k+20k)*20k
  R.logicBattery_V = Pin(PIN_AIN_LGC_BAT) *1.45
End Function

' - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
Sub R.RGB_LED i, r,g,b, _auto
  ' Set RGB LED number `i` (`r`,`g`,`b` in 0..255) or automatically, based
  ' on state if `_auto` > 0
  Local integer c
  If _auto > 0 Then
    If R.running = 1 Then
      r = rgb.conn : g = 0 : b =30
    ElseIf R.running = 2 Then
      r = rgb.conn : g =50 : b = 0
    ElseIf
      r = rgb.conn : g = 0 : b = 0
    EndIf
  EndIf
  If i = RGB_I_PULSE Then
    rgb.pcol(0) = r : rgb.pcol(1) = g : rgb.pcol(2) = b
    Math Scale rgb.pcol(), 1, rgb.pnow()
    Math Scale rgb.pcol(), -1/RGB_N_STEP, rgb.pstep()
    rgb.pi = RGB_N_STEP
    c = ((Int(rgb.pnow(0)) And &HFF) << 16)
    c = c Or ((Int(rgb.pnow(1)) And &HFF) << 8) Or (Int(rgb.pnow(2)) And &HFF)
  Else
    c = ((r And &HFF) << 16) Or ((g And &HFF) << 8) Or (b And &HFF)
  EndIf
  rgb.rgb(i) = c
  Device WS2812 O, PIN_WS2812, N_WS2812, rgb.rgb()
End Sub


Sub R.RGB_LED_pulse
  ' Pulse LED using the last color set
  Local integer c, v(2)
  If rgb.pi > 0 Then
    Math C_Add rgb.pnow(), rgb.pstep(), rgb.pnow()
    Inc rgb.pi, -1
  Else
    Math Scale rgb.pstep(), -1, rgb.pstep()
    rgb.pi = RGB_N_STEP
  EndIf
  Math Scale rgb.pnow(), 1, v()
  c = ((v(0) And &HFF) << 16) Or ((v(1) And &HFF) << 8) Or (v(2) And &HFF)
  rgb.rgb(RGB_I_PULSE) = c
  Device WS2812 O, PIN_WS2812, N_WS2812, rgb.rgb()
End Sub

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