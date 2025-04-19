FILENAME = tx_client
APPNAME = $(FILENAME)

ifeq ($(OS),Windows_NT)
TARGET = $(APPNAME).exe
LIBS=-lws2_32
CC=gcc
else
TARGET = $(APPNAME)
LIBS=
endif


.PHONY = build release debug profile

build release debug profile: $(TARGET)

$(TARGET): $(FILENAME).c
	$(CC) $(CFLAGS) $< -o $@ $(LIBS)

clean:
	rm -f $(FILENAME).o $(APPNAME) $(APPNAME).exe
