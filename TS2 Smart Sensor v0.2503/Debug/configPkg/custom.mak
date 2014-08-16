## THIS IS A GENERATED FILE -- DO NOT EDIT
.configuro: .libraries,e430X linker.cmd package/cfg/main_pe430X.oe430X

linker.cmd: package/cfg/main_pe430X.xdl
	$(SED) 's"^\"\(package/cfg/main_pe430Xcfg.cmd\)\"$""\"C:/Users/sam_000/Dropbox/Turtle Sense/TI workspace/TS2 Smart Sensor v0.2503/Debug/configPkg/\1\""' package/cfg/main_pe430X.xdl > $@
