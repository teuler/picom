Const PIN_SPI2_RX      = MM.Info(PinNo GP12)
Const PIN_SPI2_TX      = MM.Info(PinNo GP11)
Const PIN_SPI2_CLK     = MM.Info(PinNo GP10)
Const PIN_MCP3208_CS   = MM.Info(PinNo GP13)

Dim v(3)
Dim integer i
MCP3208.init PIN_SPI2_RX, PIN_SPI2_TX, PIN_SPI2_CLK
For i=0 To 100
  MCP3208.readADCChans 4, v()
  Math V_print v()
  Pause 500
Next

MCP3208.close
