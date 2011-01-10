lib_LTLIBRARIES = libmodbus.la
libmodbus_la_SOURCES = \
        modbus.c \
        modbus.h \
        modbus-data.c \
        modbus-private.h \
        modbus-rtu.c \
        modbus-rtu.h \
        modbus-rtu-private.h \
        modbus-tcp.c \
        modbus-tcp.h \
        modbus-tcp-private.h \
        modbus-version.h



libmodbus_la_LDFLAGS = $(AM_LDFLAGS) -no-undefined -version-info $(LIBMODBUS_LT_VERSION_INFO)

# Header files to install
libmodbusincludedir = $(includedir)/modbus
libmodbusinclude_HEADERS = modbus.h modbus-version.h modbus-rtu.h modbus-tcp.h

if RTAI_ENABLED
libmodbus_la_SOURCES += \
modbus-rtai.c \
modbus-rtai.h \
modbus-rtai-private.h

AM_CPPFLAGS = -I$(RTAI)

libmodbusinclude_HEADERS += modbus-rtai.h
endif


if OS_WIN32
libmodbus_la_LIBADD = -lwsock32
endif


DISTCLEANFILES = modbus-version.h
EXTRA_DIST = modbus-version.h.in
CLEANFILES = *~
