# invoke SourceDir generated makefile for main.pe430X
main.pe430X: .libraries,main.pe430X
.libraries,main.pe430X: package/cfg/main_pe430X.xdl
	$(MAKE) -f C:\Users\sam_000\Dropbox\TURTLE~1\TIWORK~1\TS2SMA~1.250/src/makefile.libs

clean::
	$(MAKE) -f C:\Users\sam_000\Dropbox\TURTLE~1\TIWORK~1\TS2SMA~1.250/src/makefile.libs clean

