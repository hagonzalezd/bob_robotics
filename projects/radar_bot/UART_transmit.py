
import serial
import io
import time


#serDATA = serial.Serial('/dev/ttyACM1', 921600, timeout=10) # Equivalent to COM7
serConfig = serial.Serial('/dev/ttyACM0', 115200, parity=serial.PARITY_NONE, timeout=10) # Equivalent to COM8 # serial.PARITY_NONE # ACM0
sioConfig = io.TextIOWrapper(io.BufferedRWPair(serConfig, serConfig),newline="\n") 

counter = 0
cadena = ''

while (1):	
	time.sleep(.1) # Give some time to receive more characters
	s=serConfig.readline().decode().strip() # readline is -> b'\n'
	if not s:
		break
	else:
		cadena = s
		counter = counter + 1	
		print("Data is: " + s)

print("counter", counter)

# The commands to be sent to the Radar are the lines in the configuration file. 28 strings in an array

cliCfg = [] # List of commands
cliCfg.append("sensorStop\n")
cliCfg.append("flushCfg\n")
cliCfg.append("dfeDataOutputMode 1\n")
cliCfg.append("channelCfg 15 3 0\n")
cliCfg.append("adcCfg 2 1\n")
cliCfg.append("adcbufCfg -1 0 0 1 0\n")
cliCfg.append("profileCfg 0 77 7 7 58 0 0 68 1 256 5500 0 0 30\n")
cliCfg.append("chirpCfg 0 0 0 0 0 0 0 1\n")
cliCfg.append("chirpCfg 1 1 0 0 0 0 0 2\n")
cliCfg.append("bpmCfg -1 0 0 1\n")
cliCfg.append("frameCfg 0 1 32 0 100 1 0\n")
cliCfg.append("lowPower 0 1\n")
cliCfg.append("guiMonitor -1 1 1 1 0 0 1\n")
cliCfg.append("cfarCfg -1 0 2 8 4 4 0 5120\n")
cliCfg.append("cfarCfg -1 1 0 8 4 4 0 5120\n")
cliCfg.append("peakGrouping -1 1 1 0 1 224\n")
cliCfg.append("multiObjBeamForming -1 1 0.5\n")
cliCfg.append("calibDcRangeSig -1 1 -5 8 256\n")
cliCfg.append("extendedMaxVelocity -1 0\n")
cliCfg.append("clutterRemoval -1 0\n")
cliCfg.append("compRangeBiasAndRxChanPhase 0.0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0\n")
cliCfg.append("measureRangeBiasAndRxChanPhase 0 1.5 0.2\n")
cliCfg.append("nearFieldCfg -1 0 0 20\n")
cliCfg.append("CQRxSatMonitor 0 3 4 127 0\n")
cliCfg.append("CQSigImgMonitor 0 63 8\n")
cliCfg.append("analogMonitor 1 1\n")
cliCfg.append("lvdsStreamCfg -1 0 0 0\n")
cliCfg.append("sensorStart\n")

print("")

for k in range(len(cliCfg)):
	print("Command: " + cliCfg[k])
	#serConfig.write(cliCfg[k].encode())
	
	sioConfig.write(cliCfg[k])
	#sioConfig.write(cliCfg[k].encode())
	sioConfig.flush()

	# One alternative is to place a while here until we receive something
	time.sleep(.1) # Give some time to receive the command
	while(1):
		time.sleep(.1) # Give some time to receive the command

                
		s=serConfig.readline().decode().rstrip()
		cadena = s 
		print("Response is: " + s)
		if "Done" in s:
			break # Break exits the complete while
		elif "not recognized as a CLI command" in s:
			print("Error! The command is not recognized as CLI command")

serConfig.close()


